"""Geodetic/CARLA coordinate conversion helpers for dataset generation.

For SiGMA custom maps, prefer the exact ``+proj=tmerc`` OpenDRIVE
georeference used to build the CARLA map.  The older local tangent-plane
conversion supplies a fallback when an OpenDRIVE ``geoReference`` is
unavailable. SiGMA's runtime convention is X=east, Y=south. Forward and
inverse tmerc helpers share that Y-sign convention to resolve CARLA
south-positive Y and OpenDRIVE northing consistently.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET

import numpy as np


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
WGS84_EP2 = WGS84_E2 / (1.0 - WGS84_E2)


def _geo_tuple(geo):
    return np.array([
        float(getattr(geo, 'latitude')),
        float(getattr(geo, 'longitude')),
        float(getattr(geo, 'altitude')),
    ], dtype=np.float64)


def geolocation_to_carla_location(carla_map, carla_module, latitude, longitude, altitude, sample_span_m=100.0):
    """Approximate inverse of ``carla.Map.transform_to_geolocation``.

    This supplies a fallback for custom maps with an unavailable or unusable
    OpenDRIVE ``+proj=tmerc`` georeference. For SiGMA generated maps,
    use :func:`geolocation_to_xodr_tmerc_carla_location`.
    """
    if carla_map is None or not hasattr(carla_map, 'transform_to_geolocation'):
        raise ValueError('CARLA map does not expose transform_to_geolocation()')
    if carla_module is None or not hasattr(carla_module, 'Location'):
        raise ValueError('carla module with Location class is required')

    span = float(sample_span_m)
    if span <= 0.0:
        span = 100.0

    origin_loc = carla_module.Location(x=0.0, y=0.0, z=0.0)
    x_loc = carla_module.Location(x=span, y=0.0, z=0.0)
    y_loc = carla_module.Location(x=0.0, y=span, z=0.0)
    z_loc = carla_module.Location(x=0.0, y=0.0, z=span)

    origin_geo = _geo_tuple(carla_map.transform_to_geolocation(origin_loc))
    basis_geo = np.stack([
        (_geo_tuple(carla_map.transform_to_geolocation(x_loc)) - origin_geo) / span,
        (_geo_tuple(carla_map.transform_to_geolocation(y_loc)) - origin_geo) / span,
        (_geo_tuple(carla_map.transform_to_geolocation(z_loc)) - origin_geo) / span,
    ], axis=1)

    target_geo = np.array([float(latitude), float(longitude), float(altitude)], dtype=np.float64)
    delta_geo = target_geo - origin_geo
    xyz, residuals, rank, singular_values = np.linalg.lstsq(basis_geo, delta_geo, rcond=None)

    loc = carla_module.Location(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
    recon_geo = _geo_tuple(carla_map.transform_to_geolocation(loc))
    err = recon_geo - target_geo
    meta = {
        'method': 'finite_difference_affine_inverse',
        'sample_span_m': span,
        'origin_geolocation': origin_geo.tolist(),
        'target_geolocation': target_geo.tolist(),
        'reconstructed_geolocation': recon_geo.tolist(),
        'error': {
            'latitude_deg': float(err[0]),
            'longitude_deg': float(err[1]),
            'altitude_m': float(err[2]),
        },
        'jacobian_rank': int(rank),
        'jacobian_singular_values': [float(v) for v in singular_values],
    }
    if residuals is not None and len(residuals):
        meta['least_squares_residual'] = [float(v) for v in residuals]
    else:
        meta['least_squares_residual'] = []
    return loc, meta


def _meters_per_degree(latitude_deg):
    lat = np.deg2rad(float(latitude_deg))
    meters_per_lat = (
        111132.92
        - 559.82 * np.cos(2.0 * lat)
        + 1.175 * np.cos(4.0 * lat)
        - 0.0023 * np.cos(6.0 * lat)
    )
    meters_per_lon = (
        111412.84 * np.cos(lat)
        - 93.5 * np.cos(3.0 * lat)
        + 0.118 * np.cos(5.0 * lat)
    )
    return float(meters_per_lat), float(meters_per_lon)


def geolocation_to_local_carla_location(
        carla_module, latitude, longitude, altitude, *,
        origin_latitude, origin_longitude, origin_altitude=0.0,
        x_axis='east', y_axis='south'):
    """Convert WGS84 lat/lon to a local tangent-plane CARLA approximation."""
    if carla_module is None or not hasattr(carla_module, 'Location'):
        raise ValueError('carla module with Location class is required')

    origin_lat = float(origin_latitude)
    origin_lon = float(origin_longitude)
    origin_alt = float(origin_altitude or 0.0)
    meters_per_lat, meters_per_lon = _meters_per_degree(origin_lat)
    east_m = (float(longitude) - origin_lon) * meters_per_lon
    north_m = (float(latitude) - origin_lat) * meters_per_lat

    x_axis = str(x_axis or 'east').lower()
    y_axis = str(y_axis or 'south').lower()
    if x_axis in ('east', '+east', 'e'):
        x = east_m
    elif x_axis in ('west', '+west', 'w'):
        x = -east_m
    else:
        raise ValueError(f'Unsupported local geo x_axis={x_axis!r}')

    if y_axis in ('south', '+south', 's'):
        y = -north_m
    elif y_axis in ('north', '+north', 'n'):
        y = north_m
    else:
        raise ValueError(f'Unsupported local geo y_axis={y_axis!r}')

    z = float(altitude) - origin_alt
    loc = carla_module.Location(x=float(x), y=float(y), z=float(z))
    meta = {
        'method': 'local_tangent_plane',
        'origin_geolocation': [origin_lat, origin_lon, origin_alt],
        'target_geolocation': [float(latitude), float(longitude), float(altitude)],
        'meters_per_degree': {
            'latitude': float(meters_per_lat),
            'longitude': float(meters_per_lon),
        },
        'axes': {'x': x_axis, 'y': y_axis},
        'local_enu_m': {'east': float(east_m), 'north': float(north_m)},
        'roundtrip_status': 'not_evaluated_without_carla_map',
    }
    return loc, meta


def _meridional_arc(phi: float) -> float:
    e2 = WGS84_E2
    e4 = e2 * e2
    e6 = e4 * e2
    return WGS84_A * (
        (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * phi
        - (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0) * math.sin(2.0 * phi)
        + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * math.sin(4.0 * phi)
        - (35.0 * e6 / 3072.0) * math.sin(6.0 * phi)
    )


def _proj_parameter(text: str, name: str, default=None):
    match = re.search(r'(?:^|\s)\+{}=([^\s]+)'.format(re.escape(name)), str(text or ''))
    if match:
        return float(match.group(1))
    if default is None:
        raise ValueError(f'OpenDRIVE geoReference is missing +{name}')
    return float(default)


def parse_xodr_tmerc_georeference(opendrive_text: str):
    """Parse SiGMA ``+proj=tmerc`` parameters from an OpenDRIVE document.

    Returns ``None`` for a missing or unusable transverse-Mercator georeference.
    The implementation uses the Python standard library and mirrors the
    projection equations of the migrated site map builder.
    """
    try:
        root = ET.fromstring(str(opendrive_text))
        header = root.find('header')
        if header is None:
            return None
        georef = (header.findtext('geoReference') or '').strip()
    except Exception:
        return None
    if '+proj=tmerc' not in georef:
        return None
    try:
        lat_0 = _proj_parameter(georef, 'lat_0')
        lon_0 = _proj_parameter(georef, 'lon_0')
        k = _proj_parameter(georef, 'k', _proj_parameter(georef, 'k_0', 1.0))
        x_0 = _proj_parameter(georef, 'x_0', 0.0)
        y_0 = _proj_parameter(georef, 'y_0', 0.0)
    except Exception:
        return None
    return {
        'proj': 'tmerc',
        'lat_0': float(lat_0),
        'lon_0': float(lon_0),
        'k': float(k),
        'x_0': float(x_0),
        'y_0': float(y_0),
        'raw': georef,
    }


def _tmerc_forward(latitude_deg: float, longitude_deg: float, *, lat_0: float,
                   lon_0: float, k: float = 1.0, x_0: float = 0.0,
                   y_0: float = 0.0):
    """WGS84 transverse Mercator matching the DTT OpenDRIVE preparation code."""
    phi = math.radians(float(latitude_deg))
    lam = math.radians(float(longitude_deg))
    lat0 = math.radians(float(lat_0))
    lon0 = math.radians(float(lon_0))
    m0 = _meridional_arc(lat0)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    tan_phi = math.tan(phi)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)
    t = tan_phi * tan_phi
    c = WGS84_EP2 * cos_phi * cos_phi
    a = cos_phi * (lam - lon0)
    m = _meridional_arc(phi)
    x = n * (
        a
        + (1.0 - t + c) * a**3 / 6.0
        + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * WGS84_EP2) * a**5 / 120.0
    )
    y = (m - m0) + n * tan_phi * (
        a * a / 2.0
        + (5.0 - t + 9.0 * c + 4.0 * c * c) * a**4 / 24.0
        + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * WGS84_EP2) * a**6 / 720.0
    )
    return float(x_0) + float(k) * x, float(y_0) + float(k) * y


def geolocation_to_xodr_tmerc_carla_location(
        carla_module, latitude, longitude, altitude, *, projection,
        origin_altitude=0.0):
    """Convert WGS84 to the exact SiGMA OpenDRIVE/CARLA local frame.

    OpenDRIVE tmerc returns X=east, Y=north.  CARLA/UE uses X=east, Y=south for
    SiGMA maps, so the projected northing is negated.
    """
    if carla_module is None or not hasattr(carla_module, 'Location'):
        raise ValueError('carla module with Location class is required')
    proj = dict(projection or {})
    x_east, y_north = _tmerc_forward(
        float(latitude), float(longitude),
        lat_0=float(proj['lat_0']),
        lon_0=float(proj['lon_0']),
        k=float(proj.get('k', 1.0)),
        x_0=float(proj.get('x_0', 0.0)),
        y_0=float(proj.get('y_0', 0.0)),
    )
    z = float(altitude) - float(origin_altitude or 0.0)
    loc = carla_module.Location(x=float(x_east), y=float(-y_north), z=float(z))
    meta = {
        'method': 'xodr_tmerc',
        'projection': {
            'lat_0': float(proj['lat_0']),
            'lon_0': float(proj['lon_0']),
            'k': float(proj.get('k', 1.0)),
            'x_0': float(proj.get('x_0', 0.0)),
            'y_0': float(proj.get('y_0', 0.0)),
        },
        'target_geolocation': [float(latitude), float(longitude), float(altitude)],
        'projected_m': {'east': float(x_east), 'north': float(y_north)},
        'axes': {'x': 'east', 'y': 'south'},
        'roundtrip_status': 'not_evaluated_without_carla_map',
    }
    return loc, meta


def _tmerc_inverse(easting_m: float, northing_m: float, *, lat_0: float,
                   lon_0: float, k: float = 1.0, x_0: float = 0.0,
                   y_0: float = 0.0):
    """Inverse of :func:`_tmerc_forward` for the SiGMA WGS84 tmerc frame."""
    k = float(k)
    if abs(k) < 1e-12:
        raise ValueError('tmerc scale k must be non-zero')
    x = (float(easting_m) - float(x_0)) / k
    y = (float(northing_m) - float(y_0)) / k
    lat0 = math.radians(float(lat_0))
    lon0 = math.radians(float(lon_0))
    m0 = _meridional_arc(lat0)
    m = m0 + y
    e2 = WGS84_E2
    e4 = e2 * e2
    e6 = e4 * e2
    mu = m / (WGS84_A * (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0))
    e1 = (1.0 - math.sqrt(1.0 - e2)) / (1.0 + math.sqrt(1.0 - e2))
    j1 = 3.0 * e1 / 2.0 - 27.0 * e1**3 / 32.0
    j2 = 21.0 * e1**2 / 16.0 - 55.0 * e1**4 / 32.0
    j3 = 151.0 * e1**3 / 96.0
    j4 = 1097.0 * e1**4 / 512.0
    fp = mu + j1 * math.sin(2.0 * mu) + j2 * math.sin(4.0 * mu) + j3 * math.sin(6.0 * mu) + j4 * math.sin(8.0 * mu)
    sin_fp = math.sin(fp)
    cos_fp = math.cos(fp)
    tan_fp = math.tan(fp)
    c1 = WGS84_EP2 * cos_fp * cos_fp
    t1 = tan_fp * tan_fp
    n1 = WGS84_A / math.sqrt(1.0 - e2 * sin_fp * sin_fp)
    r1 = WGS84_A * (1.0 - e2) / (1.0 - e2 * sin_fp * sin_fp) ** 1.5
    d = x / n1
    lat = fp - (n1 * tan_fp / r1) * (
        d**2 / 2.0
        - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1**2 - 9.0 * WGS84_EP2) * d**4 / 24.0
        + (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * t1**2 - 252.0 * WGS84_EP2 - 3.0 * c1**2) * d**6 / 720.0
    )
    lon = lon0 + (
        d
        - (1.0 + 2.0 * t1 + c1) * d**3 / 6.0
        + (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * c1**2 + 8.0 * WGS84_EP2 + 24.0 * t1**2) * d**5 / 120.0
    ) / max(abs(cos_fp), 1e-15)
    return math.degrees(lat), math.degrees(lon)


def correct_carla_gnss_to_site_geolocation(
        latitude, longitude, altitude, *, projection):
    """Correct native CARLA GNSS for SiGMA's south-positive map Y axis.

    CARLA's GNSS sensor evaluates OpenDRIVE tmerc northing using CARLA Y as
    north-positive. SiGMA-generated maps intentionally store Y as
    south-positive. Reflecting the measured projected northing around y_0
    restores the authoritative map georeference while retaining GNSS noise.
    """
    proj = dict(projection or {})
    missing = [key for key in ('lat_0', 'lon_0') if key not in proj]
    if missing:
        raise ValueError(
            'GNSS tmerc correction requires projection fields: '
            + ', '.join(missing)
        )
    params = {
        'lat_0': float(proj['lat_0']),
        'lon_0': float(proj['lon_0']),
        'k': float(proj.get('k', 1.0)),
        'x_0': float(proj.get('x_0', 0.0)),
        'y_0': float(proj.get('y_0', 0.0)),
    }
    raw_lat = float(latitude)
    raw_lon = float(longitude)
    raw_alt = float(altitude)
    if not all(math.isfinite(value) for value in (raw_lat, raw_lon, raw_alt)):
        raise ValueError('CARLA GNSS measurement contains non-finite coordinates')

    easting, native_northing = _tmerc_forward(raw_lat, raw_lon, **params)
    corrected_northing = 2.0 * params['y_0'] - native_northing
    corrected_lat, corrected_lon = _tmerc_inverse(
        easting, corrected_northing, **params)
    corrected = (float(corrected_lat), float(corrected_lon), raw_alt)
    metadata = {
        'method': 'sigma_xodr_tmerc_y_south',
        'projection': params,
        'axes': {'x': 'east', 'y': 'south'},
        'raw_projected_m': {
            'easting': float(easting),
            'northing': float(native_northing),
        },
        'corrected_projected_m': {
            'easting': float(easting),
            'northing': float(corrected_northing),
        },
    }
    return corrected, metadata


def xodr_tmerc_carla_location_to_geolocation(
        location, *, projection, origin_altitude=0.0):
    """Convert SiGMA CARLA X=east/Y=south back to WGS84."""
    proj = dict(projection or {})
    lat, lon = _tmerc_inverse(
        float(location.x), -float(location.y),
        lat_0=float(proj['lat_0']), lon_0=float(proj['lon_0']),
        k=float(proj.get('k', 1.0)),
        x_0=float(proj.get('x_0', 0.0)),
        y_0=float(proj.get('y_0', 0.0)),
    )
    alt = float(location.z) + float(origin_altitude or 0.0)
    return float(lat), float(lon), float(alt)
