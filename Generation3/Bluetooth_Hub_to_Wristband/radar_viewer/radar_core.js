(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.RadarCore = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  const DEFAULTS = {
    maxRange: 10,          // m, ignore points beyond
    staticV: 0.05,         // m/s, |v| below = static
    movingV: 0.1,          // m/s, |v| at/above = moving; between = micro-motion
    clusterEps: 0.5,       // m, link distance for clustering moving points
    gate: 0.8,             // m, max distance to associate a cluster with a track
    cellSize: 0.25,        // m, static map grid
    staticTau: 30,         // s, static map decay time constant
    staticMin: 2.5,        // accumulated weight needed to show a static cell
    personExtentMax: 1.2,  // m, max cluster extent to count as a person
    personSpeedMin: 0.1,   // m/s
    personSpeedMax: 3.0,   // m/s
    personMinHits: 3,      // frames of consistent person-like motion before labeling
    lostAfter: 1.5,        // s without a moving cluster before a track is "not moving"
    stillHold: 15,         // s a stopped person track is kept (refreshed by micro-motion)
    zWindow: 5,            // s, window for per-track z min/mean/max
    microRadius: 0.6,      // m, micro-motion within this of a still person refreshes it
  };

  function parseLine(line) {
    if (typeof line !== 'string') return null;
    const s = line.trim();
    if (s[0] !== '{') return null;
    let o;
    try { o = JSON.parse(s); } catch (e) { return null; }
    if (!o || typeof o !== 'object') return null;
    const t = typeof o.t === 'number' ? o.t : undefined;
    if ('tof' in o) {
      if (o.tof !== 1 && o.tof !== 2) return null;
      if (!Array.isArray(o.d) || o.d.length !== 64) return null;
      if (!o.d.every((n) => typeof n === 'number' && Number.isFinite(n))) return null;
      return { kind: 'tof', sensor: o.tof, t, d: o.d };
    }
    if (typeof o.f !== 'number' || !Array.isArray(o.p)) return null;
    const p = [];
    for (const q of o.p) {
      if (!Array.isArray(q) || q.length < 4) continue;
      const [x, y, z, v, snr] = q;
      if (![x, y, z, v].every((n) => typeof n === 'number' && Number.isFinite(n))) continue;
      p.push({ x, y, z, v, snr: Number.isFinite(snr) ? snr : 0 });
    }
    return { kind: 'radar', f: o.f, t, p };
  }

  class LineSplitter {
    constructor() { this.buf = ''; }
    push(chunk) {
      this.buf += chunk;
      const parts = this.buf.split('\n');
      this.buf = parts.pop();
      if (this.buf.length > 65536) this.buf = '';
      return parts.map((s) => s.replace(/\r$/, ''));
    }
  }

  function clusterPoints(points, eps) {
    const n = points.length;
    const parent = points.map((_, i) => i);
    const find = (i) => {
      while (parent[i] !== i) { parent[i] = parent[parent[i]]; i = parent[i]; }
      return i;
    };
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        if (Math.hypot(points[i].x - points[j].x, points[i].y - points[j].y) <= eps) {
          parent[find(i)] = find(j);
        }
      }
    }
    const groups = new Map();
    for (let i = 0; i < n; i++) {
      const r = find(i);
      if (!groups.has(r)) groups.set(r, []);
      groups.get(r).push(points[i]);
    }
    return [...groups.values()];
  }

  class StaticMap {
    constructor(cfg) { this.cfg = cfg; this.reset(); }
    reset() { this.cells = new Map(); this.t = null; }
    _decay(t) {
      if (this.t !== null && t > this.t) {
        const k = Math.exp(-(t - this.t) / this.cfg.staticTau);
        for (const [key, c] of this.cells) {
          c.w *= k;
          if (c.w < 0.05) this.cells.delete(key);
        }
      }
      this.t = t;
    }
    add(points, t) {
      this._decay(t);
      const cs = this.cfg.cellSize;
      for (const p of points) {
        const ix = Math.floor(p.x / cs), iy = Math.floor(p.y / cs);
        const key = ix + ',' + iy;
        let c = this.cells.get(key);
        if (!c) { c = { ix, iy, w: 0 }; this.cells.set(key, c); }
        c.w += 1;
      }
    }
    visible(t) {
      this._decay(t);
      const cs = this.cfg.cellSize, out = [];
      for (const c of this.cells.values()) {
        if (c.w >= this.cfg.staticMin) out.push({ x: (c.ix + 0.5) * cs, y: (c.iy + 0.5) * cs, w: c.w });
      }
      return out;
    }
  }

  function summarize(group) {
    let sx = 0, sy = 0, sz = 0, sv = 0;
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;
    for (const p of group) {
      sx += p.x; sy += p.y; sz += p.z; sv += Math.abs(p.v);
      minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
      minZ = Math.min(minZ, p.z); maxZ = Math.max(maxZ, p.z);
    }
    const n = group.length;
    return { x: sx / n, y: sy / n, z: sz / n, zMin: minZ, zMax: maxZ, speed: sv / n, extent: Math.hypot(maxX - minX, maxY - minY) };
  }

  class RadarEngine {
    constructor(cfg = {}) {
      this.cfg = Object.assign({}, DEFAULTS, cfg);
      this.map = new StaticMap(this.cfg);
      this.reset();
    }

    reset() {
      this.map.reset();
      this.tracks = [];
      this.nextId = 1;
    }

    update(frame, t) {
      const c = this.cfg;
      const points = [];
      for (const q of frame.p) {
        if (!(Math.hypot(q.x, q.y) <= c.maxRange)) continue;   // also rejects NaN/Infinity
        const a = Math.abs(q.v);
        points.push(Object.assign({}, q, { cls: a < c.staticV ? 'static' : a < c.movingV ? 'micro' : 'moving' }));
      }
      const stat = points.filter((p) => p.cls === 'static');
      const micro = points.filter((p) => p.cls === 'micro');
      const moving = points.filter((p) => p.cls === 'moving');

      this.map.add(stat, t);
      this._track(clusterPoints(moving, c.clusterEps).map(summarize), micro, t);

      return {
        points,
        tracks: this.tracks.map((tr) => Object.assign({ id: tr.id, x: tr.x, y: tr.y, z: tr.z, speed: tr.speed, extent: tr.extent, label: tr.label }, this._zStats(tr))),
        staticCells: this.map.visible(t),
      };
    }

    _zStats(tr) {
      let lo = Infinity, hi = -Infinity, sum = 0;
      for (const h of tr.zHist) { lo = Math.min(lo, h.zMin); hi = Math.max(hi, h.zMax); sum += h.z; }
      return { zMin: lo, zMax: hi, zMean: sum / tr.zHist.length };
    }

    _track(clusters, micro, t) {
      const c = this.cfg;
      const pairs = [];
      clusters.forEach((cl, ci) => this.tracks.forEach((tr, ti) => {
        const d = Math.hypot(cl.x - tr.x, cl.y - tr.y);
        if (d <= c.gate) pairs.push({ ci, ti, d });
      }));
      pairs.sort((a, b) => a.d - b.d);
      const usedC = new Set(), usedT = new Set();
      for (const { ci, ti } of pairs) {
        if (usedC.has(ci) || usedT.has(ti)) continue;
        usedC.add(ci); usedT.add(ti);
        const tr = this.tracks[ti], cl = clusters[ci];
        tr.x += 0.5 * (cl.x - tr.x);
        tr.y += 0.5 * (cl.y - tr.y);
        tr.z += 0.5 * (cl.z - tr.z);
        tr.speed = cl.speed; tr.extent = cl.extent;
        tr.hits++; tr.lastSeen = t;
        tr.zHist.push({ t, z: cl.z, zMin: cl.zMin, zMax: cl.zMax });
        if (tr.hits >= c.personMinHits && cl.extent <= c.personExtentMax &&
            cl.speed >= c.personSpeedMin && cl.speed <= c.personSpeedMax) tr.wasPerson = true;
      }
      clusters.forEach((cl, ci) => {
        if (usedC.has(ci)) return;
        this.tracks.push({
          id: this.nextId++, x: cl.x, y: cl.y, z: cl.z, speed: cl.speed, extent: cl.extent,
          hits: 1, lastSeen: t, lastMicro: -Infinity,
          zHist: [{ t, z: cl.z, zMin: cl.zMin, zMax: cl.zMax }], wasPerson: false, label: 'object',
        });
      });

      for (const tr of this.tracks) while (tr.zHist.length > 1 && t - tr.zHist[0].t > c.zWindow) tr.zHist.shift();
      this.tracks = this.tracks.filter((tr) => {
        if (t - tr.lastSeen <= c.lostAfter) {
          tr.label = tr.wasPerson ? 'person' : 'object';
          return true;
        }
        if (!tr.wasPerson) return false;
        if (micro.some((p) => Math.hypot(p.x - tr.x, p.y - tr.y) <= c.microRadius)) tr.lastMicro = t;
        tr.label = 'person (still)';
        return t - Math.max(tr.lastSeen, tr.lastMicro) <= c.stillHold;
      });
    }
  }

  // Horizontal distance (m) from the vertical line through the sensor, for a sensor-frame point
  // (x lateral, y boresight, z up) when the sensor is pitched down by tiltDeg about its x axis.
  function horizDist(p, tiltDeg) {
    const th = tiltDeg * Math.PI / 180;
    const yw = p.y * Math.cos(th) + p.z * Math.sin(th);
    return Math.hypot(p.x, yw);
  }

  // Moving points within D metres (horizontal) of the lift. nearest is null when there are none.
  function alertPoints(points, tiltDeg, D) {
    const alerting = [];
    let nearest = null;
    for (const p of points) {
      if (p.cls !== 'moving') continue;
      const h = horizDist(p, tiltDeg);
      if (h < D) {
        alerting.push(p);
        if (nearest === null || h < nearest) nearest = h;
      }
    }
    return { alerting, nearest };
  }

  // ToF zone distances (mm, row-major 8x8, -1 invalid) -> points in the radar sensor frame.
  // Zone (r, c): azimuth yaw + ((c+0.5)/8*45 - 22.5), elevation pitch + (22.5 - (r+0.5)/8*45), degrees.
  function tofToPoints(d, yawDeg, pitchDeg) {
    const rad = Math.PI / 180, out = [];
    for (let i = 0; i < 64; i++) {
      const dist = d[i];
      if (!Number.isFinite(dist) || dist <= 0) continue;
      const r = Math.floor(i / 8), c = i % 8;
      const az = (yawDeg + ((c + 0.5) / 8 * 45 - 22.5)) * rad;
      const el = (pitchDeg + (22.5 - (r + 0.5) / 8 * 45)) * rad;
      const m = dist / 1000;
      out.push({ x: m * Math.sin(az) * Math.cos(el), y: m * Math.cos(az) * Math.cos(el), z: m * Math.sin(el), dist, zone: i });
    }
    return out;
  }

  function tofNearest(points) {
    let best = null;
    for (const p of points) if (best === null || p.dist < best) best = p.dist;
    return best === null ? null : best / 1000;
  }

  return { DEFAULTS, parseLine, LineSplitter, clusterPoints, StaticMap, RadarEngine, horizDist, alertPoints, tofToPoints, tofNearest };
});
