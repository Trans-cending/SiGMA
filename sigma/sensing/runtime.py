"""Official CARLA 0.9.16 connection, map loading and synchronous settings.

The server is launched outside SiGMA with ``./CarlaUE4.sh -RenderOffScreen
-carla-rpc-port=2000``. SiGMA manages its client connection and world settings.
``sigma doctor`` imports this lightweight module independently of the CARLA wheel.
"""
from pathlib import Path

CARLA_VERSION = '0.9.16'


def check_versions(client):
    """Reject mismatched wheels/servers before accessing or changing the world."""
    local = str(client.get_client_version())
    remote = str(client.get_server_version())
    from importlib import metadata
    try:
        distribution = metadata.version('carla')
    except metadata.PackageNotFoundError:
        distribution = None
    # Official clean source builds may expose the release commit as a version.
    # The 0.9.16 release points at 294096e; acceptance requires the release tag
    # or that clean commit paired with the matching distribution version.
    clean_release_commit = (distribution == CARLA_VERSION
                            and local.startswith('294096e')
                            and all(c in '0123456789abcdef' for c in local))
    supported = local == CARLA_VERSION or clean_release_commit
    if local != remote or not supported or distribution not in (None, CARLA_VERSION):
        raise RuntimeError(
            f'Official CARLA {CARLA_VERSION} client and server are required; '
            f'client={local!r}, server={remote!r}. Install the matching release wheel '
            'and start the matching official server.')
    return {'client': local, 'server': remote}


def load_world(client, carla_module, config, config_dir='.'):
    """Load a cooked 0.9.16 scene, or explicitly request road-only OpenDRIVE."""
    spec = config.get('map') or {}
    if spec.get('opendrive'):
        path = Path(spec['opendrive'])
        if not path.is_absolute():
            path = Path(config_dir) / path
        if not path.is_file():
            raise FileNotFoundError(f'OpenDRIVE file does not exist: {path}')
        params = carla_module.OpendriveGenerationParameters(
            vertex_distance=float(spec.get('vertex_distance', 2.0)),
            max_road_length=float(spec.get('max_road_length', 50.0)),
            wall_height=0.0, additional_width=0.6,
            smooth_junctions=True, enable_mesh_visibility=True)
        return client.generate_opendrive_world(path.read_text(encoding='utf-8'), params)
    name = str(spec.get('name') or '')
    if not name:
        raise ValueError('stage2.map.name or stage2.map.opendrive is required')
    if not name.rsplit('/', 1)[-1].startswith('Town') and spec.get('assets_version') != CARLA_VERSION:
        raise ValueError('Custom maps require stage2.map.assets_version: 0.9.16 after reimport/cook')
    available = list(client.get_available_maps())
    if not any(x == name or x.rsplit('/', 1)[-1] == name for x in available):
        raise RuntimeError(f'Map {name!r} is not installed on the CARLA server; reimport/cook '
                           'its FBX, textures and XODR with official CARLA 0.9.16 first')
    return client.load_world(name)


def synchronous_settings(world, dt):
    """Use a single tick owner and physics steps of at most 10 ms."""
    import math
    previous = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = float(dt)
    settings.no_rendering_mode = False
    settings.substepping = True
    settings.max_substep_delta_time = 0.01
    settings.max_substeps = max(1, int(math.ceil(float(dt) / 0.01)))
    world.apply_settings(settings)
    return previous
