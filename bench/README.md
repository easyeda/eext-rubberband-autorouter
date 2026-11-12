# bench/ — routing benchmarks and profiling

All offline (no EDA client). Units: mil, ms. Scratch output goes to `tmp/` (git-ignored).
Each script has a full header comment (`--help` prints it).

| Script | Purpose |
|---|---|
| `route-bench.mjs` | Determinism + speed suite on seeded synthetic boards (spawns `scripts/bench-offline.mjs`, phase A). Compares result hash to an expected table at the top of the file. |
| `route-profile.mjs` | `node --cpu-prof` run of bench-offline (or route-dump) and a by-function summary: top self time, top inclusive time, callers of a function. |
| `route-dump.mjs` | Runs `PlanningEngine.plan()` on a real-board dump (`tmp/dumps/<name>-dump.json`) with UI-default options and reports nets/pads/vias/length/DRC/hash/phase times. |
| `../test/routing-regress-test.ts` | Small-board regression (about 15 s, part of `npm test`). |

## Typical use

```
node bench/route-bench.mjs                         # quick: nets 20, ~10 s
node bench/route-bench.mjs --suite full --runs 2   # nets 20 + 40, ~50 s per run
node bench/route-bench.mjs --case nets=80,budget=60000   # ad-hoc (budget-bound => hash not compared)
node bench/route-profile.mjs --nets 12                       # profile a synthetic board
node bench/route-profile.mjs --dump tmp/dumps/X-dump.json --budget 120000 --callers gridRoute
node bench/route-dump.mjs tmp/dumps/X-dump.json --runs 2 --out tmp/x.json
```

## Reading the results

- `route-bench` exit code 1: drcEdge > 0, routed nets below baseline, engine/script failure, or a changed
  hash (the last is non-fatal with `--allow-hash-change`). A changed hash means the routing geometry
  changed; if intended, re-measure and update `EXPECTED` in `route-bench.mjs`.
- With a large budget the engine ends by itself and the hash is deterministic. A run that hits the wall-clock
  budget (e.g. `--nets 80 --budget 60000`) is not reproducible; compare quality numbers, not hashes.
- The engine reserves a tail of `max(30 s, 20% of budget)` and exits its main loops early; budgets under
  about 30 s route almost nothing. `route-dump` prints `endedBy=budget` when the engine logged
  "时间预算用尽" or wall time reached 95% of the budget.
- `route-dump` uses UI defaults (ripup 3, connectivityFirst, optimize, via fallback), not the dump's own
  `options` (ripup 5). Its strict edge audit covers routed-vs-routed tracks, tracks vs pad shapes and
  tracks vs routed vias only; static tracks, outline, keepouts, via-pad/via-via and holes are not checked
  (see the header of `route-dump.mjs`).
- The lenient `drc` count (audit-lib) compares centre lines and shrinks pad boxes; the strict `drcEdge`
  count is the one the gates use.
- Hash determinism assumes an idle machine: the engine has wall-clock-based internal limits, so under heavy
  CPU load `nets=20` gave a different hash (`6d7388718ea8`, 127 s) than idle (`1a80913285bd`, ~10 s; `d66b7e731782` before commit 869c38b).
  `route-bench` warns when the load average is high.

## End-to-end tighten run in the real client (`tighten-e2e-*`)

Unlike everything above this runs inside the real JLCEDA Pro desktop client (CLI `invoke`), with the PRODUCTION
`TopologicalRouter.route()` / `tightenExistingRouting()` writing to the real canvas. Needs the client (`scripts/lceda-env.sh`),
the licence on the machine, and the owner board `case/clock-pcb3/` (git-ignored; pass `--data-root` of the main checkout
when running from a worktree). Never touches `case/`: it copies the board to `<data-root>/tmp/testwork-ask003/<run>/board/`.

```
node bench/tighten-e2e-run.mjs --data-root /path/to/main-checkout --run <name>    # route, tighten, tighten2, EDA DRC, metrics, SVGs (to <run>/svg/ only)
node bench/tighten-e2e-run.mjs --data-root ... --run <name> --steps route,tighten,tighten2,drc --depot <dir>   # also copy the SVGs into <dir>
node bench/tighten-e2e-run.mjs --data-root ... --run <name> --force               # re-use an existing run name: stale artefacts are deleted first
node bench/tighten-e2e-run.mjs --data-root ... --steps route,tighten,drc --no-worker --run nw1   # main-thread fallback path
node bench/tighten-e2e-run.mjs --data-root ... --steps route,fault,tighten,drc --fault-at 7   # fault injection between route and tighten
node bench/tighten-e2e-run.mjs --data-root ... --reuse <runDir> [--depot <dir>]   # no client: recompute metrics/SVGs from a run dir
node bench/tighten-e2e-render.mjs <runDir>/snapshot-routed.json --out a.svg [--box x0,y0,x1,y1] [--marks "x,y;x,y"]
```

| File | Role |
|---|---|
| `tighten-e2e-entry.ts` | esbuild entry (ESM, `inline-worker:planning` plugin replicated in the driver); runs in the client, calls the production classes unchanged; background jobs `route`/`tighten`/`drc`/`snapshot`, polled by the driver; tighten accepts `faultAt` (sets `globalThis.__RB_TIGHTEN_FAULT__ = { createFailAt }` for that call only, removed in `finally`) |
| `tighten-e2e-eda.js` | `invoke --code-file` body: imports the bundle once (cached on `globalThis`), forwards commands |
| `tighten-e2e-run.mjs` | driver: build, open session (always closed in `finally`), steps, snapshots, `tighten-metrics.mjs` + `audit-lib.mjs jointAudit` + duplicate-copper check, `metrics.json`, SVGs |
| `tighten-e2e-render.mjs` | SVG renderer of a canvas snapshot (data y up, top red / bottom blue, true track width, real pad shapes) |

Run dir (`tmp/testwork-ask003/<run>/`): `snapshot-{unrouted,routed,fault,tight1,tight2}.json`, `drc-*.json`, `route.json`,
`fault.json`, `tighten{1,2}.json` (result incl. `verified` and the full `issues` array), `probe.json`, `metrics.json`, `bundle.mjs`, `svg/`. Whole-board and zoom SVGs always go to `<run>/svg/`; they are copied to another
directory only when `--depot <dir>` is passed explicitly (there is no default depot path; `--no-render` skips the SVGs).

Run directory safety: a fresh run refuses to start when `<data-root>/tmp/testwork-ask003/<run>/` already exists and is not empty (old
snapshots would be mixed into the metrics). Use a new `--run` name, or `--force` to delete the stale `snapshot-*`, `drc-*`, `route/fault/tighten*.json`,
`probe/doctor/metrics.json` and `svg/` first. `--reuse <runDir>` (no client) is exempt: it only recomputes metrics/SVGs from what is there.

Exit code: non-zero if any route / fault / tighten / snapshot / DRC step failed or errored (job status not `done`, client error, timeout) or on a fatal
error. The session is still closed (unless `--keep-session`) and the metrics of the stages that exist are still written. The entry rejects starting
`route` or `tighten` while the other (or the same) is still running in the client; read-only jobs (`drc`, `snapshot`) are not restricted.

Optional step `fault` (not default; `--fault-at N`, default 7): a tighten run whose N-th canvas create call throws. The driver prints
aborted / adopted / failed / verified / issues, the final message and the warn/error logs, and adds a `fault` column (vs `routed`)
between `routed` and `tight1`; `tight1` is then compared against `fault`. The metrics table also has `tighten verified` / `tighten issues`
rows (issues = write-back canvas-vs-model check: net, layer, missing/extra, x, y).

Caveats: the metrics see only what the snapshot has (copper lines/arcs/vias/pads, layer-11 outline, copper fills/polylines);
keepouts, pours, silk, hole-to-hole and plane layers are not covered. EDA merges/splits same-net lines and reassigns
primitive ids, so geometry is compared by coverage (sampled centre-lines), not by primitive identity. EDA DRC "NEW" items are matched
by type + position (3 mil). The route step takes minutes; `--route-timeout-min` / `--tighten-timeout-min` bound the wait
(the job keeps running in the client until the session is closed).
