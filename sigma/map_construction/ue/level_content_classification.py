"""Pure classification helpers shared by generated-level editor passes."""

import re

BUILDING_LIGHT_BLUEPRINT_TOKENS = (
    "bp_lights",
    "seeintobuilding",
    "see_into_building",
)
BUILDING_EMISSIVE_TOKENS = (
    "window",
    "facade",
    "emissive",
    "emission",
)
BUILDING_CONTEXT_TOKENS = (
    "/static/buildings/",
    "/carla/static/building/",
    "building",
)
BUILDING_GLASS_ASSET_NAMES = {
    "mi_glassext_master",
}
TRAFFIC_CONTROL_TOKENS = (
    "trafficlight", "traffic_light", "traffic light",
    "trafficsign", "traffic_sign", "traffic sign",
    "speedlimit", "speed_limit", "stop_sign", "stopsign",
    "yield_sign", "yieldsign",
)
STREETLIGHT_RUNTIME_CONTROL_TOKENS = (
    "mosaicstreetlightmodeactor",
    "mosaic_dttstreetlightmodecontroller",
)
STREETLIGHT_TOKENS = (
    "/game/carla/static/pole/streetlights/",
    "/static/pole/streetlights/",
    "/static/streetlights/",
    "/static/streetlight/",
    "/streetlights/",
    "/streetlight/",
    "bp_streetlight",
    "streetlight",
    "street_light",
    "street light",
    "streetlamp",
    "street_lamp",
    "street lamp",
    "lamppost",
    "lamp_post",
    "lamp post",
    "lightpole",
    "light_pole",
    "light pole",
)


def normalized_context(*values):
    return " ".join(str(value) for value in values).lower()


def is_generated_building_mesh(mesh_path, map_name):
    map_token = str(map_name).strip().lower()
    roots = (
        "/game/custommaps/{}/static/buildings/".format(map_token),
        "/game/carla/static/building/{}/".format(map_token),
    )
    package = str(mesh_path).split(".", 1)[0].lower()
    return package.startswith(roots)


def is_building_glass_material(material_path, material_name=""):
    """Match Glass through explicit material asset and slot names."""
    text = normalized_context(material_path, material_name)
    tokens = re.split(r"[^a-z0-9]+", text)
    package_name = str(material_path).split(".", 1)[0].rsplit("/", 1)[-1]
    asset_names = {package_name.lower(), str(material_name).lower()}
    return bool(asset_names & BUILDING_GLASS_ASSET_NAMES) or any(
        re.fullmatch(r"(?:mi|m)?glass\d*", token) for token in tokens
    )


def is_osm_streetlamp(label, actor_path):
    return str(label).startswith("MOSAIC_OSMStreetLamp_") or (
        "MOSAIC_OSMStreetLamp_" in str(actor_path)
    )


def is_traffic_control_context(context):
    return any(token in context for token in TRAFFIC_CONTROL_TOKENS)


def building_night_lighting_classification(
    context,
    has_light_component=False,
    semantic_tags=(),
):
    if any(token in context for token in BUILDING_LIGHT_BLUEPRINT_TOKENS):
        return "building_light_blueprint"
    if any(token in context for token in BUILDING_EMISSIVE_TOKENS):
        return "building_emissive_candidate"
    if has_light_component and (
        "Building" in set(semantic_tags or ())
        or any(token in context for token in BUILDING_CONTEXT_TOKENS)
    ):
        return "building_owned_light_component"
    return ""


def is_procedural_streetlight_context(context):
    return (
        not any(token in context for token in STREETLIGHT_RUNTIME_CONTROL_TOKENS)
        and any(token in context for token in STREETLIGHT_TOKENS)
    )


def streetlight_kind(
    context,
    actor_label,
    actor_path,
    has_light_component=False,
    semantic_tags=(),
):
    if is_osm_streetlamp(actor_label, actor_path):
        return "osm"
    if is_traffic_control_context(context):
        return None
    if building_night_lighting_classification(
        context,
        has_light_component=has_light_component,
        semantic_tags=semantic_tags,
    ):
        return None
    if is_procedural_streetlight_context(context):
        return "procedural"
    return None


SEMANTIC_CONTEXT_RULES = (
    (
        "Building",
        (
            "/static/roofs/", "/static/buildings/",
            "/carla/static/building/", "roofmesh", "roof_mesh",
            "buildingmesh", "building_mesh", "buildingactor",
            "building_actor", "building",
        ),
    ),
    (
        "RoadLine",
        (
            "/carla/static/roadline/", "/static/roadline/",
            "/static/roadlines/", "/static/lanemark/", "lanemark",
            "lane_mark", "lane marking", "roadline", "road_line",
            "roadmark", "road_mark", "road marking",
        ),
    ),
    (
        "SideWalk",
        (
            "/carla/static/sidewalk/", "/static/sidewalk/",
            "/static/sidewalks/", "sidewalk", "side_walk",
        ),
    ),
    (
        "Road",
        (
            "/carla/static/road/", "/static/road/", "/static/roads/",
            "/static/drivinglane/", "drivinglane", "driving_lane",
            "driving lane", "roadmesh", "road_mesh", "roadsurface",
            "road_surface", "road surface",
        ),
    ),
    (
        "Terrain",
        (
            "/carla/static/terrain/", "/static/terrain/",
            "landscapemesh", "landscape_mesh", "terrain",
        ),
    ),
    (
        "Vegetation",
        (
            "/carla/static/vegetation/", "vegetation", "foliage",
            "/trees/", "/tree", "tree_", "sm_tree", "/bush",
            "bush_", "bush", "plant", "shrub",
        ),
    ),
    ("TrafficControl", TRAFFIC_CONTROL_TOKENS),
    (
        "Pole",
        (
            "/carla/static/pole/", "/static/pole/", "polemesh",
            "pole_mesh", "poleactor", "pole_actor", "streetlight",
            "street_light",
        ),
    ),
)


def classify_semantic_context(
    mesh_path,
    actor_path,
    actor_label,
    component_name,
):
    context = normalized_context(
        mesh_path,
        actor_path,
        actor_label,
        component_name,
    )
    for semantic_class, tokens in SEMANTIC_CONTEXT_RULES:
        if any(token in context for token in tokens):
            return semantic_class
    return None
