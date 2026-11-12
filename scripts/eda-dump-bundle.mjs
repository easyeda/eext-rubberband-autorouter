const __edaShim = (globalThis.eda ?? null);

// src/router/polygon-parse.ts
function parsePolygonPath(polygon) {
  if (!Array.isArray(polygon) || polygon.length < 3)
    return [];
  const head = typeof polygon[0] === "string" ? String(polygon[0]).toUpperCase() : "";
  if (head === "R" && polygon.length >= 5) {
    const X = Number(polygon[1]), Y = Number(polygon[2]);
    const w = Number(polygon[3]), h = Number(polygon[4]);
    const rot = (Number(polygon[5]) || 0) * Math.PI / 180;
    const c = Math.cos(rot), s = Math.sin(rot);
    const toWorld = (dx, dy) => ({ x: X + dx * c - dy * s, y: Y + dx * s + dy * c });
    const pts2 = [toWorld(0, 0), toWorld(w, 0), toWorld(w, -h), toWorld(0, -h)];
    pts2.push({ ...pts2[0] });
    return pts2;
  }
  if (head === "CIRCLE" && polygon.length >= 4) {
    const cx = Number(polygon[1]), cy = Number(polygon[2]), r = Number(polygon[3]);
    const pts2 = [];
    for (let k = 0; k <= 24; k++) {
      const a = k / 24 * Math.PI * 2;
      pts2.push({ x: cx + r * Math.cos(a), y: cy + r * Math.sin(a) });
    }
    return pts2;
  }
  const pts = [];
  const ARC_STEP = Math.PI / 12;
  let i = 0;
  let curX = 0, curY = 0;
  while (i < polygon.length) {
    const tok = polygon[i];
    if (typeof tok === "number" && typeof polygon[i + 1] === "number") {
      curX = tok;
      curY = polygon[i + 1];
      pts.push({ x: curX, y: curY });
      i += 2;
      continue;
    }
    const kind = typeof tok === "string" ? tok.toUpperCase() : "";
    if (kind === "L") {
      i += 1;
      continue;
    }
    if (kind === "C") {
      const c1x = Number(polygon[i + 1]), c1y = Number(polygon[i + 2]);
      const c2x = Number(polygon[i + 3]), c2y = Number(polygon[i + 4]);
      const ex = Number(polygon[i + 5]), ey = Number(polygon[i + 6]);
      const sx = curX, sy = curY;
      for (let k = 1; k <= 8; k++) {
        const t = k / 8, u = 1 - t;
        pts.push({
          x: u * u * u * sx + 3 * u * u * t * c1x + 3 * u * t * t * c2x + t * t * t * ex,
          y: u * u * u * sy + 3 * u * u * t * c1y + 3 * u * t * t * c2y + t * t * t * ey
        });
      }
      curX = ex;
      curY = ey;
      i += 7;
      continue;
    }
    if (kind === "ARC" || kind === "CARC") {
      const angle = (Number(polygon[i + 1]) || 0) * Math.PI / 180;
      const ex = Number(polygon[i + 2]), ey = Number(polygon[i + 3]);
      const sx = curX, sy = curY;
      const chord = Math.hypot(ex - sx, ey - sy);
      if (chord > 1e-6 && Math.abs(Math.sin(angle / 2)) > 1e-6) {
        const R = chord / (2 * Math.abs(Math.sin(angle / 2)));
        const sgn = angle > 0 ? 1 : -1;
        const hh = sgn * R * Math.cos(angle / 2);
        const nx = -(ey - sy) / chord, ny = (ex - sx) / chord;
        const ccx = (sx + ex) / 2 + hh * nx, ccy = (sy + ey) / 2 + hh * ny;
        const a0 = Math.atan2(sy - ccy, sx - ccx);
        const steps = Math.max(2, Math.ceil(Math.abs(angle) / ARC_STEP));
        for (let k = 1; k <= steps; k++) {
          const a = a0 + angle * k / steps;
          pts.push({ x: ccx + R * Math.cos(a), y: ccy + R * Math.sin(a) });
        }
      } else {
        pts.push({ x: ex, y: ey });
      }
      curX = ex;
      curY = ey;
      i += 4;
      continue;
    }
    i += 1;
  }
  return pts;
}

// src/router/rubberband-engine.ts
function capsuleToCircles(x1, y1, x2, y2, r) {
  const len = Math.hypot(x2 - x1, y2 - y1);
  if (len < 1e-9)
    return [{ x: x1, y: y1, r }];
  const spacing = Math.max(1.8 * r, 0.5);
  const n = Math.max(1, Math.ceil(len / spacing));
  const out = [];
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    out.push({ x: x1 + (x2 - x1) * t, y: y1 + (y2 - y1) * t, r });
  }
  return out;
}
var __obsSeq = 0;
function nextObsId(prefix) {
  return `${prefix}${__obsSeq++}`;
}
function buildPolygonObstacle(pts, r, net, layer, group, noPrune = false) {
  const out = [];
  for (let i = 0; i < pts.length - 1; i++) {
    out.push({ id: nextObsId("w"), kind: "circle", x: pts[i].x, y: pts[i].y, r, net, layer, group, noPrune });
  }
  for (let i = 0; i < pts.length - 1; i++) {
    for (const c of capsuleToCircles(pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y, r)) {
      out.push({ id: nextObsId("b"), kind: "circle", x: c.x, y: c.y, r, net, layer, group, noPrune, blockerOnly: true });
    }
  }
  if (pts.length >= 3) {
    const xs = pts.map((p) => p.x), ys = pts.map((p) => p.y);
    const mnX = Math.min(...xs), mxX = Math.max(...xs), mnY = Math.min(...ys), mxY = Math.max(...ys);
    const step = r * 1.2;
    const nx = Math.max(1, Math.ceil((mxX - mnX) / step));
    const ny = Math.max(1, Math.ceil((mxY - mnY) / step));
    for (let gx = 0; gx <= nx; gx++) {
      for (let gy = 0; gy <= ny; gy++) {
        const px = mnX + (mxX - mnX) * gx / nx;
        const py = mnY + (mxY - mnY) * gy / ny;
        if (pointInPolygonPx(px, py, pts)) {
          out.push({ id: nextObsId("b"), kind: "circle", x: px, y: py, r, net, layer, group, noPrune, blockerOnly: true });
        }
      }
    }
  }
  return out;
}
function pointInPolygonPx(px, py, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const xi = poly[i].x, yi = poly[i].y;
    const xj = poly[j].x, yj = poly[j].y;
    if (yi > py !== yj > py && px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
      inside = !inside;
    }
  }
  return inside;
}
function buildPolylineObstacle(pts, r, net, layer, group, noPrune = false) {
  const out = [];
  for (const p of pts) {
    out.push({ id: nextObsId("w"), kind: "circle", x: p.x, y: p.y, r, net, layer, group, noPrune });
  }
  for (let i = 0; i < pts.length - 1; i++) {
    for (const c of capsuleToCircles(pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y, r)) {
      out.push({ id: nextObsId("b"), kind: "circle", x: c.x, y: c.y, r, net, layer, group, noPrune, blockerOnly: true });
    }
  }
  return out;
}

// src/router/drc-rules.ts
var MM_TO_MIL = 39.3701;
var FALLBACK_TRACE_WIDTH_MIL = 8;
var FALLBACK_CLEARANCE_MIL = 8;
var FALLBACK_DRC_RULES = {
  traceWidthMil: FALLBACK_TRACE_WIDTH_MIL,
  clearanceMil: FALLBACK_CLEARANCE_MIL,
  trackToTrackMil: FALLBACK_CLEARANCE_MIL,
  trackToLineMil: FALLBACK_CLEARANCE_MIL,
  trackToBoardMil: 12,
  trackToPadMil: FALLBACK_CLEARANCE_MIL,
  trackToFillMil: FALLBACK_CLEARANCE_MIL,
  holeToHoleMil: 12
  // JLC 标准钻孔边距 0.3mm ≈ 11.8mil，取整 12
};
function ruleValueToMil(value, unit) {
  switch (String(unit ?? "mm").toLowerCase()) {
    case "mil":
      return value;
    case "inch":
    case "in":
      return value * 1e3;
    case "cm":
      return value * MM_TO_MIL * 10;
    case "mm":
    default:
      return value * MM_TO_MIL;
  }
}
async function fetchDRCRules(onLog) {
  try {
    const config = await eda.pcb_Drc.getCurrentRuleConfiguration();
    const cfg = config?.config;
    const spacing = cfg?.Spacing?.["Safe Spacing"];
    let trackToTrackMm = 0.1016;
    let trackToPadMm = 0.1524;
    let trackToLineMm = 0.1524;
    let trackToBoardMm = 0.3;
    let trackToFillMm = 0.254;
    let spacingUnit = "mm";
    if (spacing) {
      const tableKey = Object.keys(spacing).find((k) => spacing[k].isSetDefault) ?? Object.keys(spacing)[0];
      const tbl = spacing[tableKey];
      const content = tbl?.tables?.["1"]?.content ?? [];
      const cols = tbl?.column ?? [];
      const rows = tbl?.row ?? [];
      const trackColIdx = cols.indexOf("Track");
      spacingUnit = tbl?.unit;
      const readSpacing = (rowName) => {
        const rowIdx = rows.indexOf(rowName);
        if (rowIdx < 0)
          return null;
        const val = content[rowIdx]?.[Math.min(trackColIdx, content[rowIdx]?.length - 1)];
        return typeof val === "number" && val > 0 ? val : null;
      };
      trackToTrackMm = readSpacing("Track") ?? trackToTrackMm;
      const padVal = Math.max(
        readSpacing("SMD Pad") ?? 0,
        readSpacing("TH Pad") ?? 0,
        readSpacing("Via") ?? 0
      );
      if (padVal > 0)
        trackToPadMm = padVal;
      trackToLineMm = readSpacing("Line") ?? trackToLineMm;
      trackToBoardMm = readSpacing("Board Outline") ?? trackToBoardMm;
      trackToFillMm = readSpacing("Copper/Plane Zone") ?? readSpacing("Fill Region/Teardrop") ?? trackToFillMm;
    }
    const trackRules = cfg?.Physics?.Track;
    let traceWidthMm = 0.254;
    let trackUnit = "mm";
    if (trackRules) {
      const defaultKey = Object.keys(trackRules).find((k) => trackRules[k].isSetDefault) ?? Object.keys(trackRules)[0];
      const tw = trackRules[defaultKey]?.form?.data?.["1"]?.defaultValue;
      if (typeof tw === "number" && tw > 0)
        traceWidthMm = tw;
      trackUnit = trackRules[defaultKey]?.unit;
    }
    let holeToHoleMm = 0.3;
    {
      const other = cfg?.Spacing?.["Other Spacing"];
      const ocKey = other ? Object.keys(other).find((k) => other[k]?.isSetDefault) ?? Object.keys(other)[0] : void 0;
      const oc = ocKey ? other[ocKey] : void 0;
      const v = oc?.form?.hole2Hole;
      if (typeof v === "number" && v > 0)
        holeToHoleMm = v;
    }
    const rules = {
      traceWidthMil: ruleValueToMil(traceWidthMm, trackUnit),
      clearanceMil: Math.max(
        ruleValueToMil(trackToTrackMm, spacingUnit),
        ruleValueToMil(trackToPadMm, spacingUnit)
      ),
      trackToTrackMil: ruleValueToMil(trackToTrackMm, spacingUnit),
      trackToLineMil: ruleValueToMil(trackToLineMm, spacingUnit),
      trackToBoardMil: ruleValueToMil(trackToBoardMm, spacingUnit),
      trackToPadMil: ruleValueToMil(trackToPadMm, spacingUnit),
      trackToFillMil: ruleValueToMil(trackToFillMm, spacingUnit),
      holeToHoleMil: ruleValueToMil(holeToHoleMm, "mm")
    };
    onLog?.(
      `DRC \u89C4\u5219\uFF1A\u7EBF\u5BBD ${rules.traceWidthMil.toFixed(2)}mil\uFF0CTrack \u95F4\u8DDD ${rules.trackToTrackMil.toFixed(2)}mil\uFF0CPad \u95F4\u8DDD ${rules.trackToPadMil.toFixed(2)}mil\uFF0C\u677F\u6846\u95F4\u8DDD ${rules.trackToBoardMil.toFixed(2)}mil`,
      "info"
    );
    return rules;
  } catch (err) {
    console.warn("[DRCRules] \u83B7\u53D6 DRC \u89C4\u5219\u5931\u8D25\uFF0C\u4F7F\u7528\u9ED8\u8BA4\u503C:", err);
    onLog?.("\u8BFB\u53D6 DRC \u89C4\u5219\u5931\u8D25\uFF0C\u4F7F\u7528\u9ED8\u8BA4\u503C", "warn");
    return { ...FALLBACK_DRC_RULES };
  }
}

// src/router/planning-engine.ts
function isGroundNetName(name) {
  const upper = name.trim().toUpperCase();
  if (!upper)
    return false;
  if (upper === "0V" || upper === "\u5730" || upper === "EARTH")
    return true;
  return /(^|[_\-. /])(A|D|P|S|CHASSIS)?(GND|VSS)([_\-. 0-9]|$)/.test(upper);
}
function isPowerNetName(name) {
  if (isGroundNetName(name))
    return false;
  const upper = name.trim().toUpperCase();
  if (!upper)
    return false;
  return /^(VCC|VDD|VBAT|VBUS|VIN|VOUT|VREF|PWR|VSYS)([_\-. 0-9]|$)/.test(upper) || /^[+-]\d/.test(upper) || /^\d+V\d*$/.test(upper);
}

// scripts/eda-dump-driver.ts
function setup(eda2) {
  globalThis.eda = eda2;
}
async function sleep(ms) {
  const t = Date.now();
  while (Date.now() - t < ms)
    await new Promise((r) => setTimeout(r, 0));
}
async function ensurePcbTab(timeoutMs = 6e4) {
  const eda2 = globalThis.eda;
  const t0 = Date.now();
  let opened = false;
  for (; ; ) {
    if (!opened) {
      try {
        const pcbs = await eda2.dmt_Pcb.getAllPcbsInfo();
        const uuid = Array.isArray(pcbs) ? pcbs[0]?.uuid : void 0;
        if (uuid) {
          await eda2.dmt_EditorControl.openDocument(uuid);
          opened = true;
        }
      } catch {
      }
    }
    try {
      const nets = await eda2.pcb_Net.getAllNetsName();
      if (Array.isArray(nets))
        return;
    } catch {
    }
    if (Date.now() - t0 > timeoutMs)
      throw new Error("\u7B49\u5F85 PCB \u6587\u6863\u5C31\u7EEA\u8D85\u65F6");
    await sleep(500);
  }
}
var parsePath = parsePolygonPath;
async function dumpInput(opts = {}) {
  const eda2 = globalThis.eda;
  const drc = await fetchDRCRules();
  const padShapes = {};
  try {
    const padPrims = await eda2.pcb_PrimitivePad.getAll();
    for (const sp of padPrims ?? []) {
      const p = sp;
      if (Array.isArray(p.pad)) {
        const key = `${(p.x ?? 0).toFixed(2)},${(p.y ?? 0).toFixed(2)}`;
        padShapes[key] = {
          pad: p.pad,
          hole: Array.isArray(p.hole) ? p.hole : null,
          rotation: p.rotation ?? 0
        };
      }
    }
  } catch {
  }
  const allNets = await eda2.pcb_Net.getAllNetsName();
  const padMap = /* @__PURE__ */ new Map();
  const allPads = [];
  for (const netName of allNets ?? []) {
    try {
      const primitives = await eda2.pcb_Net.getAllPrimitivesByNet(netName);
      if (!primitives || primitives.length === 0)
        continue;
      const netPads = [];
      for (const prim of primitives) {
        const p = prim;
        if (p.type !== void 0 && p.type !== 1 && p.type !== 2)
          continue;
        const x = p.center?.x ?? p.x ?? p.startX ?? 0;
        const y = p.center?.y ?? p.y ?? p.startY ?? 0;
        if (x === 0 && y === 0 && !p.center)
          continue;
        const id = p.globalIndex || p.uuid || `${netName}_${allPads.length}`;
        const padShape = p.pad ?? p.defaultPadShapeAspect ?? [];
        let wMil = 0, hMil = 0;
        if (Array.isArray(padShape) && padShape.length >= 3) {
          wMil = padShape[1] ?? 0;
          hMil = padShape[2] ?? 0;
        }
        const hole = p.hole ?? p.holeAspect ?? [];
        const holeMil = Array.isArray(hole) && hole.length >= 2 ? hole[1] ?? 0 : 0;
        if (wMil === 0 && hMil === 0) {
          const cached = padShapes[`${x.toFixed(2)},${y.toFixed(2)}`];
          let shape = cached?.pad;
          if (!Array.isArray(shape)) {
            let best = null, bd = 2;
            for (const k of Object.keys(padShapes)) {
              const [kx, ky] = k.split(",").map(Number);
              const dd = Math.hypot(kx - x, ky - y);
              if (dd < bd) {
                bd = dd;
                best = padShapes[k].pad;
              }
            }
            shape = best;
          }
          if (Array.isArray(shape) && shape.length >= 3) {
            wMil = shape[1] ?? 0;
            hMil = shape[2] ?? 0;
          }
        }
        if (wMil === 0 && hMil === 0) {
          wMil = 32;
          hMil = 34;
        }
        const rawLayer = p.layerId ?? p.layer ?? 1;
        let padType = "smd";
        if (holeMil > 0 || rawLayer === 12)
          padType = "th";
        else if (p.isVia || p.type === "via")
          padType = "via";
        let layer = rawLayer;
        if (padType === "smd" && (rawLayer === 12 || rawLayer > 2))
          layer = 1;
        const info = {
          x,
          y,
          net: netName,
          id,
          layer,
          padType,
          holeDiameter: holeMil,
          radius: Math.max(wMil / 2, hMil / 2, holeMil / 2),
          bodyRadius: Math.max(Math.min(wMil, hMil) / 2, holeMil / 2),
          index: allPads.length
        };
        allPads.push(info);
        netPads.push(info);
      }
      if (netPads.length >= 1)
        padMap.set(netName, netPads);
    } catch {
    }
  }
  const obstacles = [];
  const lineClear = Math.max(drc.trackToLineMil, 8);
  const fillClear = Math.max(drc.trackToFillMil, 10);
  const polylines = await eda2.pcb_PrimitivePolyline.getAll();
  for (const pl of polylines ?? []) {
    const p = pl;
    const layer = p.layerId ?? p.layer ?? 1;
    const lw = p.lineWidth ?? 10;
    const net = p.net ?? "";
    if (layer === 11) {
      const r2 = lw / 2 + drc.trackToBoardMil + drc.traceWidthMil / 2;
      const pts2 = parsePath(p.polygon?.polygon ?? p.polygon ?? []);
      if (pts2.length >= 2)
        obstacles.push(...buildPolylineObstacle(pts2, r2, void 0, void 0, "board", true));
      continue;
    }
    if (layer !== 1 && layer !== 2 && layer !== 12)
      continue;
    const r = lw / 2 + lineClear + drc.traceWidthMil / 2 + 2;
    const pts = parsePath(p.polygon?.polygon ?? p.polygon ?? []);
    if (pts.length >= 2) {
      obstacles.push(...buildPolylineObstacle(pts, r, net || void 0, layer, `poly:${pts[0]?.x},${pts[0]?.y}`));
    }
  }
  const lines = await eda2.pcb_PrimitiveLine.getAll();
  for (const line of lines ?? []) {
    const l = line;
    const net = l.net ?? "";
    const layer = l.layerId ?? l.layer ?? 1;
    if (layer === 11) {
      const lw2 = l.lineWidth ?? 10;
      const r2 = lw2 / 2 + drc.trackToBoardMil + drc.traceWidthMil / 2;
      obstacles.push(...buildPolylineObstacle(
        [{ x: l.startX ?? 0, y: l.startY ?? 0 }, { x: l.endX ?? 0, y: l.endY ?? 0 }],
        r2,
        void 0,
        void 0,
        "board",
        true
      ));
      continue;
    }
    if (layer !== 1 && layer !== 2 && layer !== 12)
      continue;
    const lw = l.lineWidth ?? 10;
    const r = lw / 2 + lineClear + drc.traceWidthMil / 2 + 2;
    obstacles.push(...buildPolylineObstacle(
      [{ x: l.startX ?? 0, y: l.startY ?? 0 }, { x: l.endX ?? 0, y: l.endY ?? 0 }],
      r,
      net || void 0,
      layer,
      `line:${l.primitiveId ?? ""}`
    ));
  }
  const arcs = await eda2.pcb_PrimitiveArc.getAll();
  for (const arc of arcs ?? []) {
    const o = arc;
    const net = o.net ?? "";
    const layer = o.layerId ?? o.layer ?? 1;
    const lw = o.lineWidth ?? 10;
    const sx = o.startX ?? 0, sy = o.startY ?? 0;
    const ex = o.endX ?? 0, ey = o.endY ?? 0;
    const sweep = (o.arcAngle ?? 0) * Math.PI / 180;
    const L = Math.hypot(ex - sx, ey - sy);
    if (!sweep || Math.abs(Math.sin(sweep / 2)) < 1e-6 || L < 1e-6)
      continue;
    const R = L / (2 * Math.abs(Math.sin(sweep / 2)));
    const sgn = sweep > 0 ? 1 : -1;
    const h = sgn * R * Math.cos(sweep / 2);
    const nx = -(ey - sy) / L, ny = (ex - sx) / L;
    const cx = (sx + ex) / 2 + h * nx, cy = (sy + ey) / 2 + h * ny;
    const a0 = Math.atan2(sy - cy, sx - cx);
    const pts = [];
    const steps = Math.max(6, Math.ceil(Math.abs(sweep) / (Math.PI / 12)));
    for (let k = 0; k <= steps; k++) {
      const ang = a0 + Math.abs(sweep) * k / steps * sgn;
      pts.push({ x: cx + R * Math.cos(ang), y: cy + R * Math.sin(ang) });
    }
    if (layer === 11) {
      const r = lw / 2 + drc.trackToBoardMil + drc.traceWidthMil / 2;
      obstacles.push(...buildPolylineObstacle(pts, r, void 0, void 0, "board", true));
    } else if (layer === 1 || layer === 2 || layer === 12) {
      const r = lw / 2 + lineClear + drc.traceWidthMil / 2 + 2;
      obstacles.push(...buildPolylineObstacle(pts, r, net || void 0, layer, `arc:${sx.toFixed(0)},${sy.toFixed(0)}`));
    }
  }
  const fills = await eda2.pcb_PrimitiveFill.getAll();
  for (const fill of fills ?? []) {
    const f = fill;
    const layer = f.layerId ?? f.layer ?? 1;
    if (layer !== 1 && layer !== 2 && layer !== 12)
      continue;
    const lw = f.lineWidth ?? 0;
    const net = f.net ?? "";
    const r = lw / 2 + fillClear + drc.traceWidthMil / 2 + 2;
    const poly = f.complexPolygon?.polygon ?? f.complexPolygon ?? f.polygon?.polygon ?? f.polygon ?? [];
    const pts = parsePath(poly);
    if (pts.length >= 3) {
      obstacles.push(...buildPolygonObstacle(pts, r, net || void 0, layer, `fill:${pts[0]?.x},${pts[0]?.y}`, true));
    }
  }
  const viaPrims = await eda2.pcb_PrimitiveVia.getAll();
  const viaClear = Math.max(drc.trackToTrackMil, drc.trackToPadMil);
  for (const v of viaPrims ?? []) {
    const vp = v;
    const vx = vp.x ?? vp.center?.x;
    const vy = vp.y ?? vp.center?.y;
    if (typeof vx !== "number" || typeof vy !== "number")
      continue;
    const dia = Number(vp.diameter) || 24;
    obstacles.push({
      id: "via" + obstacles.length,
      kind: "circle",
      x: vx,
      y: vy,
      r: dia / 2 + viaClear + drc.traceWidthMil / 2 + 0.1,
      net: vp.net || void 0,
      layer: void 0,
      group: "via:" + (vp.net ?? "") + ":" + vx.toFixed(1) + "," + vy.toFixed(1),
      noPrune: true
    });
  }
  let copperLayers = 2;
  try {
    const all = await eda2.pcb_Layer.getAllLayers();
    const copper = (all || []).filter((l) => l?.type === "SIGNAL" && l.layerStatus === 1);
    if (copper.length >= 1)
      copperLayers = copper.length;
  } catch {
  }
  const networks = Array.from(padMap.entries()).map(([name, pads]) => ({ name, pads }));
  return {
    input: {
      drc,
      allPads,
      networks,
      staticObstacles: obstacles,
      padShapes,
      copperLayers,
      options: {
        ripupRounds: opts.ripupRounds ?? 5,
        optimize: opts.optimize ?? true,
        allowViaFallback: opts.allowViaFallback ?? true,
        allowViaInPad: opts.allowViaInPad ?? false,
        netOrder: opts.netOrder ?? "auto"
      }
    },
    totalNetted: allPads.length,
    netCount: networks.length,
    groundNets: networks.filter((n) => isGroundNetName(n.name)).length,
    powerNets: networks.filter((n) => isPowerNetName(n.name) && !isGroundNetName(n.name)).length
  };
}
export {
  dumpInput,
  ensurePcbTab,
  setup
};
