"""Stage1 geometric LoS selection reference, independent of camera/RF links."""


def normalize_los_reference(vehicles, ris=None, rsu_ids=None):
    value = vehicles.get('los_reference', {'node_kind': 'rsu', 'node_id': 'any'})
    if not isinstance(value, dict) or set(value) - {'node_kind', 'node_id'}:
        raise ValueError('vehicles.los_reference accepts node_kind and node_id only')
    kind = value.get('node_kind', 'rsu')
    node_id = value.get('node_id', 'any')
    if kind not in ('rsu', 'ris') or not isinstance(node_id, str) or not node_id:
        raise ValueError('los_reference requires node_kind=rsu|ris and a nonempty node_id')
    if kind == 'ris':
        ids = [n['id'] for n in (ris or {}).get('nodes', [])]
        if not (ris or {}).get('enabled') or node_id not in ids:
            raise ValueError('los_reference RIS node_id must name an enabled RIS node')
    elif rsu_ids is not None and node_id != 'any' and node_id not in rsu_ids:
        raise ValueError('los_reference RSU node_id does not exist')
    ratio = vehicles.get('min_los_ratio', 0.0)
    if not isinstance(ratio, (int, float)) or not 0 <= ratio <= 1:
        raise ValueError('vehicles.min_los_ratio must be in [0,1]')
    return {'node_kind': kind, 'node_id': node_id}
