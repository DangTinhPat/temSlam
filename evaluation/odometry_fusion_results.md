# Fusion encoder bánh + gyro — 2026-11-11

Đã thay công thức 80% encoder / 20% gyro trong mapper bằng module C++ native
[WheelImu](../technology/scan3d_fusion/mapping/odometry/wheel_imu.hpp). Build tích hợp
`firmware/build/bin/rb_tof_mapper` đã cập nhật; source dùng chung cho mapper
standalone và simulator. RViz vẫn nhận x/y/yaw qua state của mapper như trước.
Các tiến trình mapper đang chạy cần khởi động lại để dùng binary mới.

Module dùng yaw gyro đã tích phân và bù bias ở nguồn, encoder đã bù góc thân và
các khớp chân, động học hai bánh, covariance x/y/yaw và kiểm tra bất đồng hai nguồn.
Không tích phân lúc chưa cân bằng, mất mẫu dài hoặc bộ đếm nhảy. Có output
`odometry.jsonl` mang timestamp, trạng thái fusion, hình học thân và góc cổ.

Đã nối `RgbdSlam.track_body()` với `BodyLocalization`: dùng mount camera theo
thời gian để chuyển pose camera về thân, giữ odom local liên tục và tính riêng
map_from_odom. Khi visual không có pose, giữ correction cuối để tiếp tục local
odometry; khi chưa có visual hợp lệ, gắn nhãn frame odom. Pose visual nhảy lớn
bị chặn. Đây là correction ngoài cuVSLAM; chưa thêm encoder/IMU factor vào solver.

Kiểm tra đã qua:

- C++: đi thẳng 1 m, quay tại chỗ, bất đồng encoder/gyro, unsupported/re-support,
  mất mẫu, timestamp lùi, số đo NaN và covariance tăng.
- Python localization: đầu quay với offset camera không làm thân quay theo,
  visual bị mất, visual nhảy 100 m, timestamp sai và packet unsupported.
- 3 test adapter cuVSLAM hiện có vẫn qua.
- Hồi quy mapper bốn tường, floor, dữ liệu lỗi, replay xác định và reset đều qua.
- Replay log `datasets/maps/tof-draft-20261110-121041/sensors.log`; số liệu thực
  được lưu trong [odometry_fusion_replay.json](odometry_fusion_replay.json).

Giới hạn: noise chưa hiệu chuẩn phần cứng; chuyển động tiến khi cả hai bánh cùng
trượt chưa quan sát được chỉ từ encoder/gyro. Filter hiện xử lý planar axle
odometry theo packet ToF ~15 Hz, sử dụng tích phân gyro tốc độ cao ở nguồn;
không phải estimator IMU 6DoF đầy đủ. Correction visual có gate cố định và chưa
xử lý tự động loop closure/relocalization lớn.

Chưa chạy RGB-D + wheel/IMU đồng bộ end-to-end, chưa nối vào điều khiển robot,
chưa có số đo giảm ATE. Capture sofa handheld không có encoder/IMU tương ứng;
không tạo encoder giả từ pose ground truth. Hướng dẫn và hợp đồng dữ liệu ở
[README odometry](../technology/scan3d_fusion/mapping/odometry/README.md).
