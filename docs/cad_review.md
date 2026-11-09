# RB v1.3 CAD and mass review

## Source and reference frame

- Source: `cad/robot_v1_3.step`; SHA-256 `e15ec80d90e41f0f44d8f6a12a0ed586a3863434765069a6c981ab51306232e7`.
- STEP has 17 top-level components. Its length unit is millimetres.
- `body_link` is the placement origin of `Body 1 v1.3.2-1`, at STEP
  `[1344.2053, 678.8958, 915.1971] mm`, with identity rotation.
- Coordinates below are metres relative to `body_link`; axes are `+X` forward,
  `+Y` left, `+Z` up. Component bounds are CAD axis-aligned bounding boxes
  (AABBs), not rigid-body extents after motion.

The complete source-assembly AABB in `body_link` is
`[-131.210, -132.602, -77.312]` to `[111.650, 132.602, 117.302] mm`.

## Joint-axis positions verified against CAD geometry

The rotational axes below are taken from the relevant cylindrical CAD geometry
and tire centerlines. The axis sign is normalized to the digital-twin convention;
STEP cylinders themselves do not define encoder-positive direction.

| Axis | Origin in `body_link` (mm) | Direction |
| --- | ---: | --- |
| Left hip | `[0.000, +81.200, 0.000]` | `+Y` |
| Right hip | `[0.000, -81.200, 0.000]` | `+Y` |
| Left knee | `[-99.999, +102.400, -0.514]` | `+Y` |
| Right knee | `[-100.000, -102.400, 0.000]` | `+Y` |
| Left wheel | `[-9.152, +120.100, -42.309]` | `+Y` |
| Right wheel | `[-9.369, -120.100, -42.262]` | `+Y` |
| Head motor axis | `[+25.000, 0.000, +61.600]` | `+Z` |

Hip, knee and wheel center spacings are 162.4 mm, 204.8 mm and approximately
240.2 mm. The CAD is close to mirrored, but not exact: the knee-axis vertical
difference is 0.514 mm, and the left/right wheel-axis X difference is 0.217 mm.
These small measured differences are retained in the frame file rather than
silently mirrored.

The inspector rechecks each proposed axis against cylinders in the named CAD
parts. All seven axes matched: hip/knee lines were supported within 0.00005 mm,
wheel lines within 0.0005 mm, and the head-motor line within 0.00005 mm. These
are geometric axis-line residuals, not claims about encoder zero or joint limits.

## Major CAD block locations

Bounds are `[min X,Y,Z] .. [max X,Y,Z]` in millimetres relative to `body_link`.
These describe component geometry only; they are not mass centers.

| User mass group | Matching CAD product(s) | CAD bounds (mm) | Review |
| --- | --- | --- | --- |
| Upper body | `Body 1 v1.3.2-1` | `[-81.648,-80.400,-42.500] .. [111.648,80.400,50.000]` | Direct product-name match |
| Lower body | `Body 2 v1.3.2` | `[-81.648,-71.648,-50.000] .. [111.648,71.648,-0.500]` | Direct product-name match; AABB overlaps upper-body AABB at the shell seam |
| PCB | `PCBA` | `[46.998,-65.502,-26.602] .. [97.002,65.002,-19.398]` | Direct match; inside body envelope |
| 14.4 V battery | `battery 14.4V-1` | `[-75.000,-47.500,-40.000] .. [-49.000,47.500,30.000]` | Direct match |
| Battery cover | `battery Cover v1.3` | `[-80.000,-49.700,-37.700] .. [-75.300,49.700,44.700]` | Direct match; adjacent to battery |
| Thigh-motor bracket | `Ga dong co dui` | `[41.710,-34.000,-17.700] .. [95.110,34.000,44.000]` | Direct match |
| Thigh motors | two `dong co dui` instances | left `[42.441,15.799,-16.970] .. [94.380,55.201,34.970]`; right `[42.441,-55.201,-16.970] .. [94.380,-15.799,34.970]` | Pair is placed symmetrically around body center |
| Knee motors | two `Dong co chan cho` instances | left `[-35.301,22.299,-35.301] .. [35.301,64.871,35.301]`; right `[-35.301,-64.871,-35.301] .. [35.301,-22.299,35.301]` | CAD instances sit at the hip axis; user confirms both are fixed to the body |
| Head estimate | `Robot Head POC` | `[-38.002,-48.002,47.998] .. [97.902,48.002,117.302]` | Product matches head assembly; 220 g remains an estimate |
| Left wheel assembly | `Wheel` (contains XGO motor, 70 mm wheel, rings, washer) | `[-44.153,102.399,-77.310] .. [25.849,132.601,-7.308]` | User confirms 150 g includes XGO |
| Right wheel assembly | `Wheel-1` (same component types) | `[-44.370,-132.601,-77.263] .. [25.632,-102.399,-7.261]` | User confirms 150 g includes XGO |
| Left leg assembly | `Left LEG v1.3` | `[-131.209,12.099,-77.310] .. [39.001,132.601,39.001]` | Contains multiple moving parts, not one rigid link |
| Right leg assembly | `Right LEG v1.3` | `[-131.149,-132.601,-77.263] .. [39.001,-12.099,39.001]` | Contains multiple moving parts, not one rigid link |

The report file `cad_blocks.yaml` contains the AABB and uniform-volume
centroid for each named product inspected. A uniform-volume centroid is only a
geometric reference; different materials and the supplied component masses mean
it must not be treated as a measured center of mass.

## Mass reconciliation and model status

The supplied rows add to **2,495 g** when each KL value is treated as mass per
item and multiplied by `SL/bộ`. The repeated 210 g battery row in the second
table is counted only once. The 220 g head value is approximate. This arithmetic
sum is recorded as `user_row_total_kg`; it is **not yet verified as the complete
robot mass**.

The user confirmed that the 130 g thigh motors and 410 g knee motors are all
fixed in the body; the thigh motors drive the shoulder through belts. The thigh
link is the motor-free CAD segment between hip and knee axes (about 102.2 mm
axis-to-axis). The supplied 85 g per side is mapped to the CAD Hip product and
35 g per side to the CAD Knee product. Each 150 g wheel includes its XGO motor.

Per-link mass, COM and inertia are recorded in `inertials.yaml` and
written as `<inertial>` elements in `inertial_skeleton.xml`. The latter
is an inertial-only articulated skeleton: it has the 7 revolute joints and a
free root, but no visual or collision geoms. The existing preview remains a
static CAD viewer, because the STEP has no mates to establish how the spring
and linkage components move. Do not treat the skeleton as a complete contact
simulation model yet.

The assigned masses and COM positions (COM relative to each link frame) are:

| Link | Mass (kg) | COM (m) |
| --- | ---: | ---: |
| `body_link` | 1.735 | `[0.007846, -0.000006, 0.001192]` |
| `left_thigh` | 0.085 | `[-0.047820, 0.009059, 0.008213]` |
| `right_thigh` | 0.085 | `[-0.047777, -0.009059, 0.008459]` |
| `left_shin` | 0.035 | `[0.054534, -0.007152, -0.025035]` |
| `right_shin` | 0.035 | `[0.054404, 0.007152, -0.025315]` |
| `left_wheel` | 0.150 | `[0.000084, -0.001472, -0.000039]` |
| `right_wheel` | 0.150 | `[0.000084, 0.001472, -0.000039]` |
| `head` | 0.220 | `[-0.006760, 0.000054, 0.027170]` |

Each CAD tensor is the exact BREP volume tensor scaled to the supplied mass as
if density were uniform within that named mass group. This is a transparent
approximation, not a measured COM or material-density model. The head's 220 g
and miscellaneous 100 g remain estimates; the latter is represented by a
body-centered cuboid. The 2.495 kg sum matches the supplied table, but a scale
measurement of the complete robot is still needed to verify that no component
is missing or counted twice.

## Reproducible inspection

Run `python3 tools/model/inspect_cad_blocks.py` from the workspace root to
regenerate `cad_blocks.yaml`. The audit uses the STEP source hash above,
the Body 1 datum, named component placements and BREP bounds.
