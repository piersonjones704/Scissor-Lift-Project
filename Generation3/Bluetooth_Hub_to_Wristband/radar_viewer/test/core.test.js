const test = require('node:test');
const assert = require('node:assert');
const { parseLine, LineSplitter } = require('../radar_core.js');

test('parseLine parses a valid frame', () => {
  const fr = parseLine('{"f":12,"p":[[1.0,2.0,0.1,-0.5,30],[0,3,0,0.2,10]]}');
  assert.strictEqual(fr.f, 12);
  assert.strictEqual(fr.p.length, 2);
  assert.deepStrictEqual(fr.p[0], { x: 1, y: 2, z: 0.1, v: -0.5, snr: 30 });
});

test('parseLine keeps optional t', () => {
  assert.strictEqual(parseLine('{"f":1,"t":2.5,"p":[]}').t, 2.5);
});

test('parseLine rejects junk without throwing', () => {
  for (const s of ['', '# [ok] version', '{"f":1,"p":[[1,2', 'nan', '{"f":"x","p":[]}',
                   '{"f":1,"p":5}', null, undefined, 42]) {
    assert.strictEqual(parseLine(s), null, String(s));
  }
});

test('parseLine drops malformed points but keeps the frame', () => {
  const fr = parseLine('{"f":1,"p":[[1,2,0,0.3,5],[1,2],["a",2,0,0,0],[null,1,0,0,0]]}');
  assert.strictEqual(fr.p.length, 1);
});

test('LineSplitter reassembles lines split across chunks', () => {
  const s = new LineSplitter();
  assert.deepStrictEqual(s.push('{"f":1,'), []);
  assert.deepStrictEqual(s.push('"p":[]}\n#hi\r\n{"f"'), ['{"f":1,"p":[]}', '#hi']);
  assert.deepStrictEqual(s.push(':2,"p":[]}\n'), ['{"f":2,"p":[]}']);
});

test('LineSplitter discards runaway buffer without newline', () => {
  const s = new LineSplitter();
  s.push('x'.repeat(70000));
  assert.deepStrictEqual(s.push('{"f":1,"p":[]}\n'), ['{"f":1,"p":[]}']);
});

const { clusterPoints } = require('../radar_core.js');

test('clusterPoints groups nearby points and separates far ones', () => {
  const pts = [
    { x: 0, y: 3 }, { x: 0.3, y: 3.1 }, { x: 0.6, y: 3.0 },   // chain, each link <= 0.5
    { x: 4, y: 7 },
  ];
  const groups = clusterPoints(pts, 0.5);
  assert.strictEqual(groups.length, 2);
  assert.deepStrictEqual(groups.map((g) => g.length).sort(), [1, 3]);
});

test('clusterPoints handles empty and single input', () => {
  assert.deepStrictEqual(clusterPoints([], 0.5), []);
  assert.strictEqual(clusterPoints([{ x: 1, y: 1 }], 0.5).length, 1);
});

const { StaticMap, DEFAULTS } = require('../radar_core.js');

test('StaticMap shows a cell after repeated hits and reports its center', () => {
  const m = new StaticMap(DEFAULTS);
  for (let i = 0; i < 100; i++) m.add([{ x: 2, y: 4 }], i * 0.1);
  const cells = m.visible(10);
  assert.strictEqual(cells.length, 1);
  assert.strictEqual(cells[0].x, 2.125);
  assert.strictEqual(cells[0].y, 4.125);
});

test('StaticMap hides a single stray hit', () => {
  const m = new StaticMap(DEFAULTS);
  m.add([{ x: 1, y: 1 }], 0);
  assert.strictEqual(m.visible(0).length, 0);
});

test('StaticMap decays away when no longer hit', () => {
  const m = new StaticMap(DEFAULTS);
  for (let i = 0; i < 100; i++) m.add([{ x: 2, y: 4 }], i * 0.1);
  assert.strictEqual(m.visible(10 + 600).length, 0);
});

test('StaticMap.reset clears cells and allows time to restart', () => {
  const m = new StaticMap(DEFAULTS);
  for (let i = 0; i < 50; i++) m.add([{ x: 2, y: 4 }], 100 + i * 0.1);
  m.reset();
  assert.strictEqual(m.visible(0).length, 0);
  m.add([{ x: 2, y: 4 }], 0);   // earlier time than before reset: must not throw
});

const { RadarEngine } = require('../radar_core.js');

const mk = (pts) => ({ f: 0, p: pts.map(([x, y, v]) => ({ x, y, z: 0, v, snr: 20 })) });
// three-point blob at (cx, cy), all with Doppler v
const blob = (cx, cy, v) => mk([[cx - 0.1, cy, v], [cx, cy, v], [cx + 0.1, cy, v]]);

test('walking person gets a stable id and the person label', () => {
  const e = new RadarEngine();
  let out;
  for (let i = 0; i < 6; i++) out = e.update(blob(0, 3 + 0.05 * i, 0.5), i * 0.1);
  assert.strictEqual(out.tracks.length, 1);
  assert.strictEqual(out.tracks[0].id, 1);
  assert.strictEqual(out.tracks[0].label, 'person');
});

test('a person who stops becomes "person (still)" with the same id, then expires', () => {
  const e = new RadarEngine();
  let t = 0, out;
  for (let i = 0; i < 6; i++, t += 0.1) out = e.update(blob(0, 3, 0.5), t);
  const id = out.tracks[0].id;
  for (let i = 0; i < 30; i++, t += 0.1) out = e.update(mk([[0, 3, 0.07]]), t);   // micro-motion only
  assert.strictEqual(out.tracks.length, 1);
  assert.strictEqual(out.tracks[0].id, id);
  assert.strictEqual(out.tracks[0].label, 'person (still)');
  for (let i = 0; i < 100; i++, t += 0.1) out = e.update(mk([]), t);               // ~10 s of nothing: still held
  assert.strictEqual(out.tracks.length, 1);
  for (let i = 0; i < 100; i++, t += 0.1) out = e.update(mk([]), t);               // past stillHold
  assert.strictEqual(out.tracks.length, 0);
});

test('a still person who starts walking again keeps the same id', () => {
  const e = new RadarEngine();
  let t = 0, out;
  for (let i = 0; i < 6; i++, t += 0.1) out = e.update(blob(0, 3, 0.5), t);
  const id = out.tracks[0].id;
  for (let i = 0; i < 30; i++, t += 0.1) out = e.update(mk([[0, 3, 0.07]]), t);
  for (let i = 0; i < 3; i++, t += 0.1) out = e.update(blob(0.2, 3, 0.5), t);
  assert.strictEqual(out.tracks.length, 1);
  assert.strictEqual(out.tracks[0].id, id);
  assert.strictEqual(out.tracks[0].label, 'person');
});

test('fast mover is an object and expires on empty frames', () => {
  const e = new RadarEngine();
  let t = 0, out;
  for (let i = 0; i < 6; i++, t += 0.1) out = e.update(blob(0, 5, 5.0), t);
  assert.strictEqual(out.tracks[0].label, 'object');
  for (let i = 0; i < 20; i++, t += 0.1) out = e.update(mk([]), t);
  assert.strictEqual(out.tracks.length, 0);
});

test('two separated movers become two tracks', () => {
  const e = new RadarEngine();
  let out;
  for (let i = 0; i < 6; i++) out = e.update(mk([[-2, 3, 0.5], [2, 6, -0.5]]), i * 0.1);
  assert.strictEqual(out.tracks.length, 2);
});

test('out-of-range and non-finite points are ignored', () => {
  const e = new RadarEngine();
  const out = e.update({ f: 1, p: [
    { x: 0, y: 50, z: 0, v: 1, snr: 1 },
    { x: NaN, y: 2, z: 0, v: 1, snr: 1 },
    { x: 0, y: Infinity, z: 0, v: 1, snr: 1 },
  ] }, 0);
  assert.strictEqual(out.points.length, 0);
  assert.strictEqual(out.tracks.length, 0);
});

test('static points accumulate into staticCells and are classified static', () => {
  const e = new RadarEngine();
  let out;
  for (let i = 0; i < 20; i++) out = e.update(mk([[1, 5, 0.0]]), i * 0.1);
  assert.strictEqual(out.points[0].cls, 'static');
  assert.strictEqual(out.staticCells.length, 1);
  assert.strictEqual(out.tracks.length, 0);
});

test('reset clears tracks and ids', () => {
  const e = new RadarEngine();
  for (let i = 0; i < 6; i++) e.update(blob(0, 3, 0.5), i * 0.1);
  e.reset();
  const out = e.update(blob(0, 3, 0.5), 0);
  assert.strictEqual(out.tracks[0].id, 1);
});

test('tracks report mean height (z) of their cluster', () => {
  const e = new RadarEngine();
  const withZ = (z) => ({ f: 0, p: [[-0.1, 1], [0, 1], [0.1, 1]].map(([x, y], i) => ({ x, y, z: z + i * 0.1, v: 0.5, snr: 20 })) });
  let out;
  for (let i = 0; i < 4; i++) out = e.update(withZ(0.2), i * 0.1);
  assert.ok(Math.abs(out.tracks[0].z - 0.3) < 1e-9, 'z=' + out.tracks[0].z);
});

test('maxRange can be changed at runtime via engine.cfg', () => {
  const e = new RadarEngine();
  e.cfg.maxRange = 2;
  const out = e.update(mk([[0, 1.5, 0.5], [0, 3, 0.5]]), 0);
  assert.strictEqual(out.points.length, 1);
});

test('tracks report z min/mean/max over a sliding window', () => {
  const e = new RadarEngine({ zWindow: 1.0 });
  // cluster z values: frame i has points at z = i*0.1 and i*0.1 + 0.2
  const f = (i) => ({ f: 0, p: [[-0.1, 1, i * 0.1], [0, 1, i * 0.1 + 0.2], [0.1, 1, i * 0.1 + 0.1]]
    .map(([x, y, z]) => ({ x, y, z, v: 0.5, snr: 20 })) });
  let out;
  for (let i = 0; i < 4; i++) out = e.update(f(i), i * 0.1);
  let tr = out.tracks[0];
  assert.ok(Math.abs(tr.zMin - 0.0) < 1e-9, 'zMin=' + tr.zMin);
  assert.ok(Math.abs(tr.zMax - 0.5) < 1e-9, 'zMax=' + tr.zMax);
  assert.ok(Math.abs(tr.zMean - 0.25) < 1e-9, 'zMean=' + tr.zMean);
  // 2 s later the early samples have aged out of the 1 s window
  for (let i = 4; i < 30; i++) out = e.update(f(10), i * 0.1);
  tr = out.tracks[0];
  assert.ok(Math.abs(tr.zMin - 1.0) < 1e-9, 'zMin=' + tr.zMin);
});

const { horizDist, alertPoints } = require('../radar_core.js');
const deg = (d) => d * Math.PI / 180;
// point at sensor-frame elevation e (deg) and range r in the forward plane (x = 0)
const atElev = (e, r, v = 0.5) => ({ x: 0, y: r * Math.cos(deg(e)), z: r * Math.sin(deg(e)), v, snr: 20, cls: 'moving' });

test('horizDist with no tilt is the plain ground distance', () => {
  assert.ok(Math.abs(horizDist({ x: 3, y: 4, z: 7 }, 0) - 5) < 1e-9);
});

test('horizDist on the tilted boresight is range * cos(tilt)', () => {
  const r = 2;
  assert.ok(Math.abs(horizDist(atElev(0, r), 35) - r * Math.cos(deg(35))) < 1e-9);
});

test('horizDist at -40 deg elevation with 35 deg tilt is range * cos(75)', () => {
  const r = 2;
  assert.ok(Math.abs(horizDist(atElev(-40, r), 35) - r * Math.cos(deg(75))) < 1e-9);
});

test('horizDist keeps lateral offset', () => {
  assert.ok(Math.abs(horizDist({ x: 3, y: 0, z: 0 }, 35) - 3) < 1e-9);
});

test('lower elevation alerts from farther away (1 ft = 0.3048 m)', () => {
  const D = 0.3048;
  // at -40 deg: threshold range = D / cos(75) ~ 1.178 m
  assert.strictEqual(alertPoints([atElev(-40, 1.1)], 35, D).alerting.length, 1);
  assert.strictEqual(alertPoints([atElev(-40, 1.3)], 35, D).alerting.length, 0);
  // at 0 deg: threshold range = D / cos(35) ~ 0.372 m
  assert.strictEqual(alertPoints([atElev(0, 0.3)], 35, D).alerting.length, 1);
  assert.strictEqual(alertPoints([atElev(0, 0.5)], 35, D).alerting.length, 0);
});

test('alertPoints ignores non-moving points and reports nearest', () => {
  const D = 0.3048;
  const still = Object.assign(atElev(-40, 0.5, 0), { cls: 'static' });
  const a = atElev(-40, 0.8), b = atElev(-40, 1.0);
  const res = alertPoints([still, b, a], 35, D);
  assert.strictEqual(res.alerting.length, 2);
  assert.ok(Math.abs(res.nearest - horizDist(a, 35)) < 1e-9);
  assert.strictEqual(alertPoints([], 35, D).nearest, null);
});

// ---------- ToF parsing (Task 1) ----------
const tofLine = (sensor, d, extra = '') => '{"tof":' + sensor + extra + ',"d":' + JSON.stringify(d) + '}';
const d64 = (fn = () => 1000) => Array.from({ length: 64 }, (_, i) => fn(i));

test('parseLine parses a ToF line', () => {
  const m = parseLine(tofLine(2, d64(), ',"t":1.5'));
  assert.strictEqual(m.kind, 'tof');
  assert.strictEqual(m.sensor, 2);
  assert.strictEqual(m.d.length, 64);
  assert.strictEqual(m.t, 1.5);
});

test('parseLine rejects malformed ToF lines', () => {
  const bad = [
    tofLine(1, d64().slice(0, 63)),
    tofLine(1, d64().concat([5])),
    tofLine(1, d64((i) => (i === 3 ? 'x' : 5))),
    tofLine(1, d64((i) => (i === 3 ? null : 5))),
    tofLine(3, d64()),
    '{"tof":"1","d":' + JSON.stringify(d64()) + '}',
    '{"tof":1}',
  ];
  for (const s of bad) assert.strictEqual(parseLine(s), null, s.slice(0, 40));
});

test('radar lines carry kind radar and keep their fields', () => {
  const m = parseLine('{"f":12,"p":[[1,2,0.1,-0.5,30]]}');
  assert.strictEqual(m.kind, 'radar');
  assert.strictEqual(m.f, 12);
  assert.deepStrictEqual(m.p[0], { x: 1, y: 2, z: 0.1, v: -0.5, snr: 30 });
});

test('LineSplitter + parseLine handle interleaved radar and ToF lines split mid-line', () => {
  const text = '{"f":1,"p":[]}\n' + tofLine(1, d64()) + '\n# log\n{"f":2,"p":[]}\n';
  const cuts = [7, 60, 200, text.length - 3];
  const s = new LineSplitter(), kinds = [];
  let prev = 0;
  for (const c of cuts.concat([text.length])) {
    for (const line of s.push(text.slice(prev, c))) { const m = parseLine(line); if (m) kinds.push(m.kind); }
    prev = c;
  }
  assert.deepStrictEqual(kinds, ['radar', 'tof', 'radar']);
});

test('firmware sample lines parse', () => {
  const r = parseLine('{"f":57,"p":[[0.12,2.31,0.05,0.00,18],[-0.40,1.80,-0.10,0.31,22]]}');
  const t = parseLine('{"tof":1,"d":[' + d64((i) => (i % 5 === 0 ? -1 : 842)).join(',') + ']}');
  assert.strictEqual(r.kind, 'radar');
  assert.strictEqual(t.kind, 'tof');
  assert.strictEqual(t.d[0], -1);
  assert.strictEqual(t.d[1], 842);
});

// ---------- ToF geometry (Task 2) ----------
const { tofToPoints, tofNearest } = require('../radar_core.js');
const only = (idx, mm) => { const d = new Array(64).fill(-1); d[idx] = mm; return d; };
const near = (a, b, tol = 1e-6) => Math.abs(a - b) < tol;

test('corner zone at yaw 0 points up-left', () => {
  const pts = tofToPoints(only(0, 1000), 0, 0);
  assert.strictEqual(pts.length, 1);
  const az = deg(-19.6875), el = deg(19.6875);
  assert.ok(near(pts[0].x, Math.sin(az) * Math.cos(el)));
  assert.ok(near(pts[0].y, Math.cos(az) * Math.cos(el)));
  assert.ok(near(pts[0].z, Math.sin(el)));
  assert.strictEqual(pts[0].zone, 0);
  assert.strictEqual(pts[0].dist, 1000);
});

test('center zones are symmetric', () => {
  const a = tofToPoints(only(27, 1000), 0, 0)[0], b = tofToPoints(only(36, 1000), 0, 0)[0];
  assert.ok(near(a.x, -b.x));
  assert.ok(near(a.z, -b.z));
});

test('yaw 90 rotates to +x', () => {
  const p = tofToPoints(only(36, 1000), 90, 0)[0];
  assert.ok(p.x > 0.9);
  assert.ok(Math.abs(p.y) < 0.1);
});

test('pitch raises z', () => {
  const lo = tofToPoints(only(27, 1000), 0, 0)[0], hi = tofToPoints(only(27, 1000), 0, 10)[0];
  assert.ok(hi.z > lo.z);
});

test('invalid zones are skipped', () => {
  for (const bad of [-1, 0, NaN, Infinity]) assert.deepStrictEqual(tofToPoints(only(5, bad), 0, 0), []);
  assert.deepStrictEqual(tofToPoints(new Array(64).fill(-1), 0, 0), []);
});

test('tofNearest returns the smallest distance in metres, null when empty', () => {
  assert.ok(near(tofNearest([{ dist: 1500 }, { dist: 800 }, { dist: 2000 }]), 0.8));
  assert.strictEqual(tofNearest([]), null);
});
