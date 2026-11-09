# AWS RoboMaker Small House source

- Repository: <https://github.com/aws-robotics/aws-robomaker-small-house-world>
- Branch: `ros1`
- Source revision: `dd44be6205b2e6577bf641300dacc1699cb2ca81`
- License: MIT; see `source/LICENSE`.
- Source scene: `source/worlds/small_house.world`.

Only models referenced by `small_house.world` are vendored under `source/models/`; the upstream `.git` directory is not included. The imported scene in `simulation/scenarios/indoor_small_house.xml` is a MuJoCo conversion with mesh and texture assets under `converted/`. Portraits in the upstream world reference external personal photos; these are not copied. The converter substitutes deterministic abstract poster textures for those missing images.

The upstream world is SDF for Gazebo Classic. The converter in `tools/model/import_small_house.py` adapts its visuals and static collision geometry to this MuJoCo RB model; it does not import Gazebo runtime code.
