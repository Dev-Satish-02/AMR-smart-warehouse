// Operations UI helpers (nexus_gui/static/js/format.js).
// Run:  node --test test_ops_format.mjs

import test from "node:test";
import assert from "node:assert/strict";

import {
  ROBOT_PALETTE, displayColor, TIME_CATEGORIES, categoryColor, breakdown,
  fmtDuration, fmtClock, fmtPct, fmtNumber, niceScale, escapeHtml,
} from "./nexus_gui/static/js/format.js";

test("robot colours switch to the light-theme steps of the same hue", () => {
  assert.equal(displayColor("#3987e5", "light"), "#2a78d6");
  assert.equal(displayColor("#3987E5", "dark"), "#3987e5");
  assert.equal(displayColor("#123456", "light"), "#123456"); // custom colour passes through
  assert.equal(ROBOT_PALETTE.dark.length, ROBOT_PALETTE.light.length);
});

test("time categories have a colour per theme and add up to 100%", () => {
  for (const c of TIME_CATEGORIES) {
    assert.match(c.dark, /^#[0-9a-f]{6}$/);
    assert.match(c.light, /^#[0-9a-f]{6}$/);
  }
  assert.equal(categoryColor("moving", "light"), "#2a78d6");
  const parts = breakdown({ moving: 60, waiting: 20, idle: 20 });
  assert.deepEqual(parts.map((p) => p.key), TIME_CATEGORIES.map((c) => c.key)); // fixed order
  assert.ok(Math.abs(parts.reduce((s, p) => s + p.share, 0) - 1) < 1e-9);
  assert.equal(parts.find((p) => p.key === "moving").share, 0.6);
  assert.ok(breakdown({}).every((p) => p.share === 0));
});

test("durations and clock read naturally", () => {
  assert.equal(fmtDuration(null), "—");
  assert.equal(fmtDuration(42.4), "42 s");
  assert.equal(fmtDuration(192), "3 min 12 s");
  assert.equal(fmtDuration(600), "10 min");
  assert.equal(fmtDuration(3725), "1 h 02 min");
  assert.equal(fmtClock(3725.9), "01:02:05");
});

test("percentages and numbers", () => {
  assert.equal(fmtPct(0.634), "63%");
  assert.equal(fmtPct(0.634, 1), "63.4%");
  assert.equal(fmtPct(undefined), "—");
  assert.equal(fmtNumber(12345.6), "12,346");
  assert.equal(fmtNumber(3.14159, 2), "3.14");
});

test("axis scales round up to 1-2-5 steps", () => {
  assert.deepEqual(niceScale(7), { max: 8, ticks: [0, 2, 4, 6, 8] });
  assert.deepEqual(niceScale(23), { max: 30, ticks: [0, 10, 20, 30] });
  assert.deepEqual(niceScale(19), { max: 20, ticks: [0, 5, 10, 15, 20] });
  assert.deepEqual(niceScale(0), { max: 1, ticks: [0, 1] });
  assert.equal(niceScale(0.3).max, 0.3); // no floating-point wobble
});

test("html escaping", () => {
  assert.equal(escapeHtml('<b>"R1" & co</b>'), "&lt;b&gt;&quot;R1&quot; &amp; co&lt;/b&gt;");
  assert.equal(escapeHtml(null), "");
});
