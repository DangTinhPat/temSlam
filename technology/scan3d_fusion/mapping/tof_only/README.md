# Ví dụ ToF-only

Đây là mã nguồn lõi của lần thử map chỉ dùng ToF: lấy mẫu 8×8 trong adapter mô phỏng, tích phân encoder/gyro thành odometry, rồi cập nhật occupancy voxel 3D và map xám 2D. Mapper không phụ thuộc ROS, MuJoCo hay firmware khi chạy/replay. **Chưa có scan matching hoặc loop closure.**

## Nội dung

- `mapper.cpp`: parser `TOFMAP`, odometry, chiếu tia và cập nhật occupancy.
- `tof_sensor.hpp`: adapter MuJoCo lấy 64 khoảng cách bằng ray cast và thêm nhiễu mô phỏng.
- `generate_tof_calibration.py`: sinh header C++ từ calibration dùng chung tại `technology/scan3d_fusion/config/tof.yaml`.
- `test_mapper.py`: kiểm tra hình học map, floor rejection, dữ liệu lỗi, reset và replay xác định.

Dashboard UI vẫn thuộc `gui/rb1_gui/`; route, RViz bridge và viewer scripts nằm trong `technology/scan3d_fusion/mapping/` và `technology/scan3d_fusion/viewer/`. Chúng gọi mapper này khi build SIM.

## Build và replay

Từ thư mục gốc repo:

```bash
cmake -S technology/scan3d_fusion/mapping/tof_only -B build/tof_only -DCMAKE_BUILD_TYPE=Release
cmake --build build/tof_only -j2
python technology/scan3d_fusion/mapping/tof_only/test_mapper.py build/tof_only/rb_tof_mapper
build/tof_only/rb_tof_mapper /tmp/tof-map < datasets/raw/tof-SESSION/sensors.log
```

Header calibration được sinh từ `technology/scan3d_fusion/config/tof.yaml`; cần Python và PyYAML lúc configure. Bản đồ/replay mapper chỉ cần C++17.

SIM cũng build cùng target qua `technology/scan3d_fusion/mapping/native/CMakeLists.txt`; `simulation/mujoco/sim_runner.cpp` dùng `tof_sensor.hpp` trong ví dụ này. Vì vậy đây là nguồn đang được SIM sử dụng, không phải bản sao.

## Luồng ghi

```text
TOFMAP t supported left_m right_m gyro_yaw_rad pitch_rad roll_rad axle_body_x axle_body_z wheel_radius track head_yaw range0 ... range63
MAPRESET
```

Zero range là không có return, không phải free space đến tầm tối đa. Mapper chỉ dùng encoder, IMU, calibration và range; không nhận pose ground truth. Occupancy update dùng voxel XY/Z 2,5 cm. Map xám là phép chiếu occupancy theo các lớp cao độ, không biểu thị clearance cho toàn robot.

## Fusion odometry

Encoder bánh và gyro hiện đi qua [module odometry](../odometry/README.md): fusion theo phương sai, cờ nghi trượt, covariance và xử lý mất support. Mapper xuất thêm `odometry.jsonl` để nối pose thân/cuVSLAM.

## Giới hạn

Map hiện là odometry mapping. Không scan matching/loop closure nên sai số có thể tích lũy; ToF mô phỏng là một tia tâm cho mỗi vùng 8×8 với nhiễu range, chưa mô hình histogram/SPAD hay vùng tia có diện tích. Voxel 2,5 cm là độ phân giải lưu trữ, không phải độ chính xác đo.
