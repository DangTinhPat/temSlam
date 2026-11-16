# Navigation from scanned map

**Authorization:** User requests advancing directly to navigation, building on milestones2/3 in navigation/README.md. User selected click-to-goal on a 2D map. Initial pose is configurable by CLI; goals are chosen in the UI.
**Scope:** Scan-derived conservative grid, A* and bounded path follower with fresh pose/depth guards, interactive map demo and native MuJoCo short-run odometry baseline. Do not claim failed scan matcher validated. No hardware motion.
**Tools:** Existing NumPy/SciPy, Tk, native simulator and inference worker. Use implementation agents for independent grid/controller tasks, then independent review. Existing workspace changes preserved.

## Shared interfaces

navigation/grid.py:
- FREE=0, OCCUPIED=1, UNKNOWN=-1.
- NavigationGrid(cells:int8 HxW, origin_xy:(2,), resolution:float, clearance_m:float=.25). Grid stored row y, col x, origin lower-left edge.
- .cells, .origin, .resolution, .clearance_m, .blocked (bool HxW, occupied/unknown inflated including boundary).
- .world_to_cell(xy)->(row,col), .cell_to_world(cell)->np.array(2)
- .segment_clear(a_xy,b_xy)->bool (conservative supercover; never cut corners).
- .plan(start_xy,goal_xy)->list of [x,y], ValueError if invalid/blocked/no path. Path includes exact start/goal.
- .save(path), .load(path); build_grid(session,resolution=.05,clearance_m=.25)->NavigationGrid.
- build_grid reads fusion depth+saved estimated poses only, no CAD/scene/truth. Floor evidence + ray-free evidence across robot height bands; small/overhanging obstacle endpoints block. Unknown and outside map blocked. Document approximations; never force holes free.

navigation/controller.py:
- Navigator(grid); .set_goal(start_xy,goal_xy,goal_yaw=None); .cancel(); .update(pose:dict|None, now:float, obstacles:dict|None, dt:float)->dict.
- pose {xy:[x,y],yaw:rad,stamp:monotonic_seconds,usable:bool,uncertainty_m:float}; uncertainty must be finite <=.05m (experiment assumption, not scan matcher certificate).
- obstacles {stamp:monotonic_seconds,clear:bool,reason:str} from current RGB–ToF check. Missing/stale stops; only forward driving, circular footprint .25m; rotation allowed only when footprint map-safe/current depth obstacle guard clear.
- result {state:str,reason:str,v:float,w:float,path:list,goal:list|None}; idle/planning/driving/aligning/reached/blocked/lost/cancelled.
- speed<=.1m/s,yaw<=.4rad/s; ramp acceleration/deceleration, terminal states remain latched; obstacle/pose gates stop and require explicit new goal (no unexpected autonomous resume).
- .path and .goal available for UI. Progress timeout, no skipping unsafe path corners or taking lookahead through blocked cells. Controller never accesses truth/backend.
- depth_guard(depth_mm,K,body_from_camera,stamp,now)->obstacles dict. Reject empty/invalid measurements, unknown forward footprint coverage, neck facing away, obstacle endpoints (including low ones) in stopping corridor. Use dynamic mount, no vertical_points matching mask.

## Tasks
- [x] Grid/plan: TDD on unknown cells, low obstacles, ray clearing endpoints, diagonal corner collision, blocked endpoints, save/load, inflated boundaries; real map build with coverage/connected components output.
- [x] Controller/depth guard: TDD goal following in kinematic fixtures, arrival/cancel/missing pose/stale depth, invalid uncertainty, obstacle latch, acceleration, no-progress; no command side effects.
- [x] Runner/UI: clickable map, plan/path/pose/state visible, cancel/close stop. Kinematic demo explicitly labeled. Native test owns NativeBackend only; wheel/IMU baseline+operator initial pose, fresh RGB–ToF guards, records diagnostics.
- [x] Verify actual map routes, controller cases, native startup test; retain failed/blocked outcomes. Review code and update navigation docs/results.

## Verification checkpoint — 2026-11-14

51 tests passed. Kinematic wall detour reached within 2.46 cm, with stop/latch
cases passing. Native startup/observation passed; guard rejected unknown forward
corridor. Review fixes cover capture age, processing latency, watchdog rearm,
terminal reason visibility and startup stability across readiness/odometry gaps.

**Real navigation acceptance remains unmet:** scanned grid has zero traversable
cells after 25 cm inflation, localization validation failed, and native depth
coverage has not cleared the guard. No native goal or hardware trial claimed.
Results and reproduction: `evaluation/navigation_results.md`.
