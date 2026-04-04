# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

This repository is an EasyEDA extension that performs PCB autorouting locally in JavaScript (no external service).

- Extension metadata: `extension.json`
- Runtime entry loaded by EasyEDA: `./dist/index` (configured in `extension.json`)
- Active source entry for build: `src/index.ts` (configured in `config/esbuild.common.ts`)

## Common development commands

- Install dependencies:
  - `npm install`
- Build extension bundle:
  - `npm run build`
- Watch mode (rebuild on changes):
  - `npm run watch`
- Package `.eext` artifact:
  - `npm run package`

Important build behavior:
- `npm run build` runs `scripts/bump-version.js` first, which **increments patch version** in both `package.json` and `extension.json` and updates the About menu title.
- Build output goes to `dist/index.js`.

Packaging behavior:
- `npm run package` runs `build/packaged.ts` and outputs `.eext` files under `build/dist/`.
- Packaging includes files based on `.edaignore`; source/config/build scripts are excluded from final package.

Lint/tests status in current repo:
- No `npm run lint` script is defined.
- Run unit tests: `npm test` (runs `test/core.test.ts` via tsx, no external test framework needed).
- Test file: `test/core.test.ts` — covers `RoutingOrchestrator`, `PathOptimizer`, `ObstacleBypasser`.

## High-level architecture

## 1) Active runtime path (what actually runs)

`src/index.ts` is a thin entry point (~28 lines) that delegates to `AutoRouterFacade`.

Flow in `autoRoute()`:
1. `AutoRouterFacade.route()` — orchestrates the full pipeline with progress callbacks.
2. `TopologicalRouter.route()` — core routing engine:
   - Read DRC rules (`eda.pcb_Drc.getCurrentRuleConfiguration`)
   - Collect pads per net (`eda.pcb_Net.getAllNetsName`, `getAllPrimitivesByNet`)
   - Build Delaunay graph (`delaunator`)
   - Sort networks via `RoutingOrchestrator` (power nets first, high-fanout first)
   - For each net: Prim MST → A* path → rubberband bypass → `ObstacleBypasser` → `PathOptimizer`
   - Write traces (`eda.pcb_PrimitiveLine.create`) and vias (`eda.pcb_PrimitiveVia.create`)
   - Dynamic obstacle update after each trace

Core files on this path:
- `src/index.ts` — entry, exports `autoRoute()` and `about()`
- `src/AutoRouterFacade.ts` — facade: progress management, cancel support, error handling
- `src/router/TopologicalRouter.ts` — main routing engine
- `src/router/RoutingOrchestrator.ts` — network sorting and retry scheduling
- `src/router/ObstacleBypasser.ts` — bypass strategies (left/right/multi-point/U-shape)
- `src/router/PathOptimizer.ts` — rubberband + via minimization
- `src/router/DRCChecker.ts` — DRC clearance validation
- `src/pcb/PCBWriter.ts` — EDA API write abstraction with stats tracking
- `src/progress/ProgressManager.ts` — phase-based progress callbacks
- `src/utils/logger.ts` — leveled logging
- `src/utils/toast.ts` — EDA toast notifications

## 2) Inactive / legacy stacks (do not use)

These files exist but are NOT wired into the active entry:
- `src/index-simple.ts` — superseded by current architecture
- `src/router/RouterCore.ts`, `src/router/AutoRouter.worker.ts` — worker-based stack, not integrated
- `src/router/PCBParser.ts`, `src/router/PathFinder.ts`, `src/router/ObstacleMap.ts` — worker stack support files
- `src/router/DynamicObstacleMap.ts` — used by old `src/index.ts` implementation, now replaced
- `src/router-core/**` — legacy JS library, superseded by TS implementation

## EasyEDA integration points

- Extension menu registration is in `extension.json` under `headerMenus.pcb`.
- Menu action `AutoRoute` calls exported function `autoRoute` from the runtime bundle.
- About dialog is implemented via exported `about()`.

## Unit and geometry conventions

Unit handling is critical:
- EDA API centers/coordinates are treated as mil in active code.
- Many DRC/size values come from API as mm and are converted using `MM_TO_MIL = 39.3701`.
- Routing math, obstacle checks, and geometry are performed in mil.

## Build/config notes

- TypeScript: strict mode enabled (`tsconfig.json`), `allowJs: true`.
- Bundler: esbuild IIFE bundle targeting browser (`config/esbuild.common.ts`).
- Formatting/lint baseline:
  - tabs, width 4 (`.editorconfig`)
  - single quotes, import sorting via prettier plugin (`.prettierrc.js`)
  - ESLint config exists (`.eslintrc.js`) but no lint script is defined.
