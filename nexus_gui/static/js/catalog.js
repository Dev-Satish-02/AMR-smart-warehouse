// Object catalogue (racks, equipment, areas, safety zones), loaded once from
// /api/catalog so the GUI matches nexus/catalog.py.

export const CATALOG = { object_types: {}, families: {}, station_types: [] };

export async function loadCatalog() {
  const res = await fetch("/api/catalog");
  Object.assign(CATALOG, await res.json());
  return CATALOG;
}

export function objectType(type) {
  return CATALOG.object_types[type] || { label: type, family: "area", blocking: false };
}

export function objectCells(obj) {
  const cells = [];
  for (let dy = 0; dy < obj.h; dy++) for (let dx = 0; dx < obj.w; dx++) cells.push([obj.x + dx, obj.y + dy]);
  return cells;
}

// Effective speed limit of an object (its own override or the type default).
export function speedLimit(obj) {
  return obj.speed_limit ?? objectType(obj.type).speed_limit ?? null;
}
