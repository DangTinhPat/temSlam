# Mô hình và mô phỏng

Tài liệu này mô tả đường đi từ CAD đến mô hình MuJoCo và hằng số hình học của firmware, cùng cách chạy mô phỏng.

## Luồng dữ liệu

```text
cad/robot_v1_3.step
   │  tools/model/inspect_cad_blocks.py
   ▼
model/cad_blocks.yaml  ──┐
model/joint_frames.yaml ─┤  (toạ độ trục khớp, đo từ CAD)
model/actuators.yaml  ───┤  (nhập tay: động cơ, tỉ số truyền, ma sát)
model/imu.yaml  ─────────┤  (nhập tay: vị trí và hướng IMU)
                         │  tools/model/assign_inertials.py
                         ▼
model/inertials.yaml, model/inertial_skeleton.xml
                         │  tools/model/extract_thigh_hull.py   → model/thigh_hull.json
                         │  tools/model/generate_dynamic_model.py
                         ▼
model/dynamic.xml  ──►  simulation/mujoco (rb1_sim)

model/*.yaml ──► tools/codegen/generate_geometry.py ──► firmware/core/include/rb1/core/GeneratedGeometry.hpp
                                                        firmware/core/include/rb1/core/GeneratedBodyEnvelope.hpp
```

Các file sinh ra không sửa tay. Sửa YAML đầu vào rồi chạy lại script tương ứng.

## Các script

Chạy từ thư mục gốc của repo. Yêu cầu: `cadquery-ocp`, `PyYAML`, `numpy`; xem hoặc nạp mô hình cần `mujoco`.

| Script | Việc làm |
| --- | --- |
| `tools/model/build_preview.py [--launch]` | Đọc STEP, căn tâm theo datum `Body 1 v1.3.2-1`, xuất mesh STL và `model/preview.xml` (xem CAD tĩnh) |
| `tools/model/inspect_cad_blocks.py` | Kiểm tra lại vị trí từng khối CAD và trục khớp, ghi `model/cad_blocks.yaml` |
| `tools/model/extract_thigh_hull.py` | Trích khối lồi của đùi, ghi `model/thigh_hull.json` |
| `tools/model/assign_inertials.py` | Gán khối lượng, tâm khối, quán tính cho từng link, ghi `model/inertials.yaml` và `model/inertial_skeleton.xml` |
| `tools/model/generate_dynamic_model.py` | Sinh `model/dynamic.xml` cho mô phỏng điều khiển |
| `tools/codegen/generate_geometry.py` | Sinh các header hình học cho firmware |
| `tools/standup_search.py` | Tìm tham số đứng dậy bằng mô phỏng (cần đã build `rb1_sim`) |

`model/preview.xml` chỉ là bản nhập CAD để xem vị trí: file STEP không có ràng buộc khớp, góc zero của encoder
hay giới hạn chuyển động, nên bản này không mô phỏng động lực. Tổng khối lượng trong bảng quán tính là 2.495 kg lấy từ
bảng cung cấp, chưa cân thực tế. Giả định ánh xạ và giới hạn xem `docs/cad_review.md`.

## Chạy mô phỏng

```bash
cmake -S firmware -B firmware/build -DCMAKE_BUILD_TYPE=Release
cmake --build firmware/build -j8 --target rb1_sim
python3 gui/sim_gui.py          # cần python3-tk và mujoco
```

Chạy thẳng không GUI (robot nằm, lệnh đứng dậy ở giây 0.3):

```bash
firmware/build/bin/rb1_sim --model model/dynamic.xml --start-key home --run-for 26 --state-hz 10 --at 0.3:STANDUP
```

Tuỳ chọn thường dùng: `--start-key home|spawn_extended` (nằm hoặc đứng), `--sensor-noise K --noise-seed N`,
`--wheel-delay S`, `--wheel-lag S`, `--wheel-torque NM`, `--standup-set khóa=giá_trị`. Danh sách đầy đủ: `rb1_sim --help`. Đầu ra là các dòng `READY`, `STATE`, `VIEW` phân tách bằng `|`.

Biến môi trường của GUI: `RB1_SIM_RUNNER` (đường dẫn runner), `RB1_SIM_MODEL` (file XML), `RB1_SIM_NOISE` (hệ số nhiễu
cảm biến), `RB1_SIM_ARGS` (tham số thêm cho runner).
