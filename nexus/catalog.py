"""
Catalogue of named layout objects: racks, production equipment, labelled
areas and safety zones. Served to the GUI (/api/catalog) so the editor and
renderer use the same definitions as the planner and simulation.

Families
    storage, production, facility   blocking: robots route around them
    area                            labelled region; robots may drive its lanes
    zone                            safety zone with a robot speed limit
"""

from typing import Any, Dict

OBJECT_TYPES: Dict[str, Dict[str, Any]] = {
    # storage
    "rack": {"label": "Rack", "family": "storage", "blocking": True},
    "stores_cage": {"label": "Stores Cage", "family": "storage", "blocking": True},
    # production
    "smt_line": {"label": "SMT Line", "family": "production", "blocking": True},
    "assembly_bench": {"label": "Assembly Bench", "family": "production", "blocking": True},
    "test_bay": {"label": "Test & QC Bay", "family": "production", "blocking": True},
    "machine": {"label": "Machine", "family": "production", "blocking": True},
    "conveyor": {"label": "Conveyor", "family": "production", "blocking": True},
    "packing_station": {"label": "Packing Station", "family": "production", "blocking": True},
    # facility
    "office": {"label": "Office", "family": "facility", "blocking": True},
    "control_room": {"label": "Control Room", "family": "facility", "blocking": True},
    "forklift_parking": {"label": "Forklift Parking", "family": "facility", "blocking": True},
    # areas (labels only)
    "kitting_area": {"label": "Kitting Area", "family": "area", "blocking": False},
    "staging_area": {"label": "Staging Area", "family": "area", "blocking": False},
    "inspection_area": {"label": "Incoming Inspection", "family": "area", "blocking": False},
    "esd_area": {"label": "ESD Protected Area", "family": "area", "blocking": False},
    "quarantine_area": {"label": "Quarantine", "family": "area", "blocking": False},
    # safety zones
    "crosswalk": {"label": "Crosswalk", "family": "zone", "blocking": False, "speed_limit": 0.3},
    "slow_zone": {"label": "Slow Zone", "family": "zone", "blocking": False, "speed_limit": 0.5},
}

FAMILIES = {
    "storage": "Storage",
    "production": "Production",
    "facility": "Facility",
    "area": "Areas",
    "zone": "Safety zones",
}


def object_type(name: str) -> Dict[str, Any]:
    return OBJECT_TYPES.get(name, {"label": name, "family": "area", "blocking": False})


def catalog() -> Dict[str, Any]:
    return {"object_types": OBJECT_TYPES, "families": FAMILIES}


__all__ = ["OBJECT_TYPES", "FAMILIES", "object_type", "catalog"]
