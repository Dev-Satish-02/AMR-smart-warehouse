// Pure formatting / colour helpers shared by the operations pages (no DOM).

// ------------------------------------------------------------------ theme

export function currentTheme() {
  return typeof document !== "undefined" && document.documentElement.dataset.theme === "light" ? "light" : "dark";
}

// Robot identity palette: the same eight hues, stepped per theme (validated
// for CVD separation and contrast against each theme's map surface).
export const ROBOT_PALETTE = {
  dark: ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300", "#9085e9", "#e66767"],
  light: ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"],
};

// Colour for a robot in the current theme. Layout colours arrive as the dark
// steps; custom colours pass through unchanged.
export function displayColor(hex, theme = currentTheme()) {
  if (!hex) return hex;
  const i = ROBOT_PALETTE.dark.indexOf(hex.toLowerCase());
  return i >= 0 ? ROBOT_PALETTE[theme][i] : hex;
}

// ------------------------------------------------------------------ time categories

// Fleet time breakdown, in stacking order. Colours are categorical slots in
// fixed order (validated adjacent-pair set); idle is a neutral grey.
export const TIME_CATEGORIES = [
  { key: "moving", label: "Moving", dark: "#3987e5", light: "#2a78d6" },
  { key: "handling", label: "Loading / unloading", dark: "#d95926", light: "#eb6834" },
  { key: "waiting", label: "Waiting in traffic", dark: "#199e70", light: "#1baf7a" },
  { key: "charging", label: "Charging", dark: "#c98500", light: "#eda100" },
  { key: "held", label: "Held by operator", dark: "#d55181", light: "#e87ba4" },
  { key: "stopped", label: "E-stop", dark: "#9085e9", light: "#4a3aa7" },
  { key: "idle", label: "Idle / parked", dark: "#5b616b", light: "#b9bec6" },
];

export function categoryColor(key, theme = currentTheme()) {
  const c = TIME_CATEGORIES.find((x) => x.key === key);
  return c ? c[theme] : "#888";
}

// Share of each category in a { key: seconds } map, in stacking order.
export function breakdown(timeIn) {
  const total = TIME_CATEGORIES.reduce((sum, c) => sum + (timeIn?.[c.key] || 0), 0);
  return TIME_CATEGORIES.map((c) => ({
    ...c,
    seconds: timeIn?.[c.key] || 0,
    share: total > 0 ? (timeIn?.[c.key] || 0) / total : 0,
  }));
}

// ------------------------------------------------------------------ numbers

export function fmtDuration(seconds) {
  if (seconds === null || seconds === undefined || Number.isNaN(seconds)) return "—";
  const s = Math.max(0, Math.round(seconds));
  if (s < 60) return `${s} s`;
  const m = Math.floor(s / 60), r = s % 60;
  if (m < 60) return r ? `${m} min ${r} s` : `${m} min`;
  const h = Math.floor(m / 60), mm = m % 60;
  return `${h} h ${String(mm).padStart(2, "0")} min`;
}

// Simulation clock as hh:mm:ss.
export function fmtClock(seconds) {
  const s = Math.max(0, Math.floor(seconds || 0));
  const hh = Math.floor(s / 3600), mm = Math.floor((s % 3600) / 60), ss = s % 60;
  return [hh, mm, ss].map((v) => String(v).padStart(2, "0")).join(":");
}

export function fmtPct(fraction, digits = 0) {
  if (fraction === null || fraction === undefined || Number.isNaN(fraction)) return "—";
  return `${(fraction * 100).toFixed(digits)}%`;
}

export function fmtNumber(value, digits = 0) {
  if (value === null || value === undefined || Number.isNaN(value)) return "—";
  return Number(value).toLocaleString("en-US", { minimumFractionDigits: digits, maximumFractionDigits: digits });
}

// Axis maximum rounded up to 1, 2 or 5 x 10^n, plus evenly spaced ticks.
export function niceScale(maxValue, ticks = 4) {
  if (!(maxValue > 0)) return { max: 1, ticks: [0, 1] };
  const raw = maxValue / ticks;
  const power = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 5, 10].map((m) => m * power).find((s) => s >= raw);
  const max = Math.round(Math.ceil(maxValue / step) * step * 1e9) / 1e9;
  const out = [];
  for (let v = 0; v <= max + 1e-9; v += step) out.push(Math.round(v * 1e9) / 1e9);
  return { max, ticks: out };
}

// ------------------------------------------------------------------ robot status

// status -> [palette class, icon, label]
export const STATUS = {
  MOVING: ["st-good", "s-moving", "Moving"],
  TURNING: ["st-good", "s-turning", "Turning"],
  REROUTING: ["st-serious", "s-reroute", "Rerouting"],
  WAITING: ["st-warning", "s-waiting", "Waiting"],
  YIELDING: ["st-warning", "s-yield", "Yielding"],
  BACKING_OFF: ["st-serious", "s-backoff", "Backing off"],
  DOCKED: ["st-neutral", "s-docked", "Docked"],
  ARRIVED: ["st-neutral", "s-arrived", "Arrived"],
  IDLE: ["st-neutral", "s-idle", "Idle"],
  PARKED: ["st-neutral", "s-parked", "Parked"],
  CHARGING: ["st-good", "s-charging", "Charging"],
  HELD: ["st-serious", "s-hold", "Held"],
  E_STOP: ["st-critical", "s-error", "E-stop"],
  ERROR: ["st-critical", "s-error", "Error"],
};

// What a dispatched robot is doing (shown with its motion status).
export const ACTIVITY = {
  IDLE: "Parked at home",
  RETURNING: "Returning home",
  TO_PICKUP: "To pickup",
  LOADING: "Loading",
  TO_DROPOFF: "Delivering",
  UNLOADING: "Unloading",
  TO_CHARGER: "To charger",
  CHARGING: "Charging",
};
export function statusChip(status, extra = "") {
  const [cls, icon, label] = STATUS[status] || STATUS.IDLE;
  return `<span class="status-chip ${cls}"><svg><use href="#${icon}"/></svg>${label}${extra}</span>`;
}

// ------------------------------------------------------------------ alerts

export const SEVERITY = {
  critical: { label: "Critical", icon: "s-error", cls: "st-critical", rank: 0 },
  warning: { label: "Warning", icon: "s-yield", cls: "st-warning", rank: 1 },
  info: { label: "Info", icon: "s-idle", cls: "st-neutral", rank: 2 },
};

export const ALERT_CODES = {
  ESTOP: "E-stop engaged",
  ROBOT_ERROR: "Robot fault",
  BATTERY_CRIT: "Battery critical",
  BATTERY_LOW: "Battery low",
  BLOCKED: "Robot blocked",
  CHARGER_WAIT: "Waiting for charger",
  BACKLOG: "Order backlog",
  HOLD: "Operator hold",
  SAFETY: "Safety distance",
  NO_ROUTE: "No route",
  DEADLOCK: "Deadlock resolved",
};

export function escapeHtml(text) {
  return String(text ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}
