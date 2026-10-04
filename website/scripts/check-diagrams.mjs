// Check the SVG diagrams on every page for text that leaves the drawing, overlaps other text,
// or is crossed by a line, path or shape edge.
// Usage: start the dev server (npm run dev), then: npm run check:diagrams
// Needs Google Chrome. Set CHROME to its path if it is not in the default macOS location.
import { spawn } from 'node:child_process';
import { mkdtempSync, readFileSync, readdirSync, statSync, existsSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

const BASE = process.env.SITE ?? 'http://127.0.0.1:4321/mycobot-280-lab/';
const CHROME = process.env.CHROME ?? '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';

// Pages: every .md/.mdx file under src/content/docs.
const docs = new URL('../src/content/docs/', import.meta.url).pathname;
const walk = (d) => readdirSync(d).flatMap((f) => (statSync(join(d, f)).isDirectory() ? walk(join(d, f)) : [join(d, f)]));
const pages = walk(docs).filter((f) => /\.mdx?$/.test(f)).map((f) => f.slice(docs.length).replace(/\.mdx?$/, '').replace(/(^|\/)index$/, ''))
  .sort().map((p) => BASE + (p ? p + '/' : ''));

const profile = mkdtempSync(join(tmpdir(), 'diagram-check-'));
const chrome = spawn(CHROME, ['--headless=new', '--disable-gpu', '--remote-debugging-port=0', `--user-data-dir=${profile}`, 'about:blank'], { stdio: 'ignore' });
const portFile = join(profile, 'DevToolsActivePort');
for (let i = 0; i < 100 && !existsSync(portFile); i++) await new Promise((r) => setTimeout(r, 100));
const [port, path] = readFileSync(portFile, 'utf8').trim().split('\n');

const sock = new WebSocket(`ws://127.0.0.1:${port}${path}`);
let id = 0; const pending = new Map();
sock.onmessage = (m) => { const d = JSON.parse(m.data); if (d.id && pending.has(d.id)) { pending.get(d.id)(d); pending.delete(d.id); } };
const send = (method, params = {}, sessionId) => new Promise((r) => { const i = ++id; pending.set(i, r); sock.send(JSON.stringify({ id: i, method, params, sessionId })); });
await new Promise((r) => (sock.onopen = r));
const { result: { targetId } } = await send('Target.createTarget', { url: 'about:blank' });
const { result: { sessionId } } = await send('Target.attachToTarget', { targetId, flatten: true });
await send('Emulation.setDeviceMetricsOverride', { width: 1280, height: 900, deviceScaleFactor: 1, mobile: false }, sessionId);
const probe = `(() => {
  const out = [];
  const segRect = (a, b, r) => { // segment a-b intersects rect r (shrunk by 1px)?
    const x0=r.left+1,x1=r.right-1,y0=r.top+1,y1=r.bottom-1; let t0=0,t1=1; const dx=b.x-a.x, dy=b.y-a.y;
    for (const [p,q] of [[-dx,a.x-x0],[dx,x1-a.x],[-dy,a.y-y0],[dy,y1-a.y]]) { if (p===0) { if (q<0) return false; } else { const t=q/p; if (p<0) { if (t>t1) return false; if (t>t0) t0=t; } else { if (t<t0) return false; if (t<t1) t1=t; } } }
    return true; };
  document.querySelectorAll('svg.diagram, svg[role=img]').forEach((svg, si) => {
    const sr = svg.getBoundingClientRect(); if (!sr.width) return;
    const name = (svg.getAttribute('aria-label') || '').slice(0, 50);
    const texts = [...svg.querySelectorAll('text')].filter(t => t.textContent.trim());
    const rects = texts.map(t => t.getBoundingClientRect());
    texts.forEach((t, i) => {
      const r = rects[i]; const s = t.textContent.trim().slice(0, 40);
      if (r.left < sr.left - 1 || r.right > sr.right + 1 || r.top < sr.top - 1 || r.bottom > sr.bottom + 1) out.push({ svg: si, name, kind: 'OUTSIDE', text: s, by: Math.round(Math.max(sr.left - r.left, r.right - sr.right, sr.top - r.top, r.bottom - sr.bottom)) });
      for (let j = i + 1; j < texts.length; j++) { const q = rects[j];
        const ox = Math.min(r.right, q.right) - Math.max(r.left, q.left), oy = Math.min(r.bottom, q.bottom) - Math.max(r.top, q.top);
        if (ox > 2 && oy > 3) out.push({ svg: si, name, kind: 'TEXT-TEXT', text: s + ' | ' + texts[j].textContent.trim().slice(0, 30) }); }
    });
    svg.querySelectorAll('line, polyline').forEach((ln) => {
      const m = ln.getScreenCTM(); if (!m) return;
      let pts = [];
      if (ln.tagName === 'line') pts = [[+ln.getAttribute('x1'), +ln.getAttribute('y1')], [+ln.getAttribute('x2'), +ln.getAttribute('y2')]];
      else pts = (ln.getAttribute('points')||'').trim().split(/[\\s,]+/).map(Number).reduce((a,v,k,arr)=>(k%2?a:[...a,[v,arr[k+1]]]),[]);
      const P = pts.map(([x,y]) => ({ x: m.a*x + m.c*y + m.e, y: m.b*x + m.d*y + m.f }));
      for (let k = 0; k + 1 < P.length; k++) texts.forEach((t, i) => { if (segRect(P[k], P[k+1], rects[i])) out.push({ svg: si, name, kind: 'LINE-TEXT', text: t.textContent.trim().slice(0, 40) }); });
    });
    // stroked paths (arrows, curves): sample points along them
    svg.querySelectorAll('path, polyline, polygon, rect, circle, ellipse').forEach((el) => {
      const cs = getComputedStyle(el);
      if (el.closest('defs, marker, text')) return;
      const stroked = cs.stroke !== 'none' && parseFloat(cs.strokeWidth) > 0 && cs.strokeOpacity !== '0';
      const m = el.getScreenCTM(); if (!m || !el.getTotalLength) return;
      const filled = cs.fill !== 'none' && cs.fillOpacity !== '0';
      const er = el.getBoundingClientRect();
      if (stroked) {
        let L = 0; try { L = el.getTotalLength(); } catch (e) { return; }
        const n = Math.min(400, Math.max(20, Math.ceil(L / 2)));
        const hit = new Set();
        for (let k = 0; k <= n; k++) { const q = el.getPointAtLength(L * k / n); const x = m.a*q.x + m.c*q.y + m.e, y = m.b*q.x + m.d*q.y + m.f;
          rects.forEach((r, i) => { if (x > r.left + 1 && x < r.right - 1 && y > r.top + 2 && y < r.bottom - 2) hit.add(i); }); }
        hit.forEach((i) => { const r = rects[i];
          const inside = r.left >= er.left - 1 && r.right <= er.right + 1 && r.top >= er.top - 1 && r.bottom <= er.bottom + 1;
          out.push({ svg: si, name, kind: (el.tagName === 'path' ? 'PATH' : el.tagName.toUpperCase()) + '-STROKE-TEXT' + (inside ? '' : '*'), text: texts[i].textContent.trim().slice(0, 40) }); });
      } else if (filled && el.tagName !== 'path') {
        rects.forEach((r, i) => {
          const ox = Math.min(r.right, er.right) - Math.max(r.left, er.left), oy = Math.min(r.bottom, er.bottom) - Math.max(r.top, er.top);
          const inside = r.left >= er.left - 1 && r.right <= er.right + 1 && r.top >= er.top - 1 && r.bottom <= er.bottom + 1;
          if (ox > 2 && oy > 3 && !inside) out.push({ svg: si, name, kind: 'EDGE-' + el.tagName.toUpperCase() + '-TEXT', text: texts[i].textContent.trim().slice(0, 40) });
        });
      }
    });
  });
  return JSON.stringify(out);
})()`;
let count = 0;
for (const url of pages) {
  await send('Page.navigate', { url }, sessionId);
  await new Promise((r) => setTimeout(r, 1500));
  const title = await send('Runtime.evaluate', { expression: 'document.title', returnByValue: true }, sessionId);
  if (/Error/.test(title.result?.result?.value ?? '')) {
    console.log(`\n## ${url.slice(BASE.length - 1)}\n  PAGE FAILED: ${title.result.result.value} (restart the dev server after adding or renaming a page)`);
    count += 1;
    continue;
  }
  const res = await send('Runtime.evaluate', { expression: probe, returnByValue: true }, sessionId);
  const issues = JSON.parse(res.result?.result?.value || '[]');
  const uniq = [...new Map(issues.map((x) => [JSON.stringify(x), x])).values()];
  count += uniq.length;
  if (uniq.length) { console.log('\n## ' + url.slice(BASE.length - 1)); uniq.forEach((x) => console.log(`  [svg ${x.svg}] ${x.kind}${x.by ? ' +' + x.by + 'px' : ''}: ${x.text}   (${x.name})`)); }
}
console.log(`\n${pages.length} pages, ${count} problems.`);
sock.close();
chrome.kill();
process.exitCode = count ? 1 : 0;
