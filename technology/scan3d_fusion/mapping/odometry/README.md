# Odometry encoder bánh + IMU, không ROS

RB là robot cân bằng có bánh xe. Module này dùng encoder hai bánh và gyro,
không dùng mô hình bước chân. Đã thay công thức trộn cố định 80/20 trong mapper
ToF-only bằng `wheel_imu.hpp`, được build vào `firmware/build/bin/rb_tof_mapper`.

## Luồng đang hoạt động

`sim_runner.cpp::emitTofMapping()` lấy encoder bánh, bù góc thân/hip/knee để ra
quãng đường bánh; gyro được tích phân ở tốc độ nguồn và bù bias như trước.
Packet `TOFMAP` mang số đo này xuống mapper theo tần số ToF (~15 Hz).

`WheelImu` kết hợp delta yaw gyro và encoder theo phương sai. Khi độ lệch lớn
hơn ngưỡng 5 sigma (sàn 0,04 rad), nó gắn cờ nghi trượt, dùng gyro cho góc quay
và tăng covariance. Chuyển động tiến vẫn lấy từ bánh; trượt đều cả hai bánh
không thể phát hiện chỉ bằng hai nguồn này. Covariance 3×3 cho x/y/yaw được
lan truyền qua động học vi sai; đây không phải EKF IMU 15 trạng thái.

Robot chưa cân bằng/nghiêng quá ngưỡng, mất mẫu >0,5 giây hoặc encoder nhảy quá
0,5 m thì không tích phân đoạn đó. Bộ đếm được seed lại để không cộng chuyển
động bánh treo vào vị trí khi trở lại cân bằng. Timestamp cũ/nonfinite bị từ chối.
`MAPRESET` xóa cả trạng thái fusion.

`TOFMAP` dùng pitch đã trừ `controller.imuPitchOffset()`. Roll lấy mốc khi
robot nằm yên ở home và chốt lúc nhận STANDUP, với cùng giới hạn offset 0,08 rad
của controller; không học lại offset khi đang chạy. RESET xóa mốc roll. Điều này
đòi hỏi home nằm trên sàn ngang đã biết, tương tự quy trình lấy mốc pitch hiện có;
không thay thế hiệu chuẩn IMU nhiều tư thế trên phần cứng. Không dùng pose thật
của MuJoCo để tính offset. Khởi động đứng sẵn không có mẫu home thì không tự bù roll.

Mapper ghi thêm `odometry.jsonl` cùng `trajectory.csv` và map. Dữ liệu gồm timestamp,
x/y/yaw tại trục bánh, covariance row-major, pitch/roll, hình học trục bánh so với
thân, góc cổ và cờ valid/slip. Bản ghi reset chia các đoạn tọa độ độc lập.
Chỉ dùng các packet `valid=1` cho định vị. Covariance này chỉ thuộc local axle
odometry; không coi nó là covariance đầy đủ của pose camera hoặc global map.

Các hệ số noise hiện là giá trị khởi đầu cho mô phỏng; cần đo noise/bias/slip của
phần cứng để hiệu chuẩn. Covariance là xấp xỉ: bộ bù bias ở nguồn có dùng wheel_rate,
nên tương quan encoder/gyro chưa được mô hình hóa đầy đủ. Gyro đầu vào là yaw tích phân **không wrap**, đã bù bias
ở nguồn. Module không tự hiểu raw gyro trong hệ trục IMU bất kỳ.

## Nối cuVSLAM

`RgbdSlam.track_body(timestamp_ns, rgb, depth_mm, odometry_packet, body_from_camera)`
đã nối `BodyLocalization` trong adapter cuVSLAM. Dùng hàm này thay cho `track()`
cho cùng frame. Odometry phải cùng session và lệch timestamp không quá 20 ms;
`body_from_camera(t)` phải gồm góc cổ, chiều cao/kinematics và offset camera.
Không suy ra mount từ chỉ một góc yaw khi thiếu calibration.

Luồng tính:

```
encoder + gyro → odom_from_axle → odom_from_body (pitch/roll, kinematics)
RGB-D → cuVSLAM → map_from_camera
map_from_body = map_from_camera × inverse(body_from_camera(t))
map_from_odom = map_from_body × inverse(odom_from_body)
```

Pose local odometry luôn liên tục. Pose map có thể được hiệu chỉnh khi nhận visual
pose. Khi visual bị mất, kết quả tiếp tục dùng local odometry và correction cuối;
`visual_accepted=false` biểu thị không có hiệu chỉnh mới. Khi chưa có visual đầu
hợp lệ, `frame='odom'`, không giả vờ đã có tọa độ map. Packet odometry không hợp lệ
trả `None`.

Correction vượt 0,5 m hoặc 0,35 rad bị chặn. Đây là gate khởi đầu, chưa có calibration
covariance visual; chưa phải fusion chặt bên trong bundle adjustment cuVSLAM.
Loop closure/relocalization lớn cần quản lý reset/rebase tường minh, hiện có thể
bị gate từ chối. Tạo `BodyLocalization` mới khi đổi session/MAPRESET; không nối
hai session khác gốc. Không đưa IMU thân vào rig camera cổ quay như một mount cố định.

**Đã bổ sung lượt chạy đồng bộ trong mô phỏng** tại
`datasets/processed/odometry/full-pipeline-20261111`; xem
`evaluation/full_pipeline_results.md` để biết kết quả và giới hạn.
Capture sofa handheld cũ không có odometry bánh phù hợp; không lấy pose MuJoCo
để giả encoder. Nguồn RGB-D trực tiếp và đồng bộ encoder/cổ vẫn cần nối trước khi
chạy toàn pipeline trên robot. Replay CLI `track_rgbd.py --odometry PATH` nhận log odometry đồng bộ và camera mount,
áp dụng correction từ pose SLAM cuối trước khi xuất trajectory cho TSDF.
`track_body()` là giao diện cho nguồn dữ liệu trực tiếp.

## Kiểm tra đã chạy

```bash
python technology/scan3d_fusion/mapping/odometry/test_mapping_attitude.py -v
g++ -std=c++17 -Wall -Wextra -Werror \
  technology/scan3d_fusion/mapping/odometry/test_wheel_imu.cpp -o /tmp/rb_test_wheel_imu
/tmp/rb_test_wheel_imu
technology/scan3d_fusion/mapping/cuvslam/run_python.sh -m unittest discover \
  -s technology/scan3d_fusion/mapping/odometry -v
python3 technology/scan3d_fusion/mapping/tof_only/test_mapper.py firmware/build/bin/rb_tof_mapper
```

Kiểm tra: đi thẳng/quay tại chỗ, bất đồng encoder/gyro, treo bánh, mất mẫu,
nonfinite/thứ tự timestamp, đầu quay không kéo thân quay theo, visual bị mất hoặc
nhảy pose. Bộ hồi quy mapper bốn tường/floor/reset cũng đã qua.
Replay log thực đã ghi có thống kê ở `evaluation/odometry_fusion_replay.json`;
log không có ground truth đồng bộ nên không suy ra ATE hay mức cải thiện drift.
