// Generates demo.jsonl: static wall + a person who walks, stops, walks back + one fast object.
const fs = require('fs');
const path = require('path');

const DT = 0.1, DURATION = 40;
const lines = [];
const r2 = (n) => Math.round(n * 100) / 100;

for (let i = 0; i * DT < DURATION; i++) {
  const t = i * DT;
  const p = [];
  // static wall at y = 6, x from -3 to 3
  for (let x = -3; x <= 3; x += 1.5) p.push([x + (Math.random() - 0.5) * 0.1, 6, 0.3, 0, 25]);
  // person: walks right along y=3 (0-10 s), stands still (10-20 s), walks left (20-30 s)
  let px, pv;
  if (t < 10) { px = -2 + 0.4 * t; pv = 0.4; }
  else if (t < 20) { px = 2; pv = 0.03 + 0.05 * Math.abs(Math.sin(t * 2)); }   // breathing/fidget
  else if (t < 30) { px = 2 - 0.4 * (t - 20); pv = -0.4; }
  else { px = null; }
  if (px !== null) {
    for (const dx of [-0.15, 0, 0.15]) {
      p.push([r2(px + dx + (Math.random() - 0.5) * 0.05), r2(3 + (Math.random() - 0.5) * 0.1), 0.9, r2(pv), 30]);
    }
  }
  // fast object crosses between 25 and 28 s at y=8
  if (t >= 25 && t < 28) p.push([r2(-4 + 2.5 * (t - 25)), 8, 0.5, 4.2, 20]);
  lines.push(JSON.stringify({ f: i, t: r2(t), p: p.map((q) => q.map(r2)) }));
}
// ToF: flat wall perpendicular to the boresight at y = 2.5 m, 15 Hz, both sensors pointing forward (yaw 0)
const tofD = [];
for (let r = 0; r < 8; r++) {
  for (let c = 0; c < 8; c++) {
    const az = ((c + 0.5) / 8 * 45 - 22.5) * Math.PI / 180, el = (22.5 - (r + 0.5) / 8 * 45) * Math.PI / 180;
    tofD.push(Math.round(2500 / (Math.cos(az) * Math.cos(el))));
  }
}
const entries = lines.map((l) => ({ t: JSON.parse(l).t, l }));
for (let k = 0; k / 15 < DURATION; k++) {
  const t = r2(k / 15);
  for (const sensor of [1, 2]) entries.push({ t, l: JSON.stringify({ tof: sensor, t, d: tofD }) });
}
entries.sort((a, b) => a.t - b.t);
fs.writeFileSync(path.join(__dirname, '..', 'demo.jsonl'), entries.map((e) => e.l).join('\n') + '\n');
console.log('wrote', lines.length, 'radar frames +', entries.length - lines.length, 'ToF lines');
