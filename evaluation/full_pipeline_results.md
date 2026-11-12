# Toàn luồng đồng bộ RGB–ToF–encoder–IMU, 2026-11-11

Đã chạy thành công mô phỏng vật lý/controller, capture đồng bộ, DELTAR GPU,
odometry bánh/gyro, cuVSLAM GPU + correction odometry, TSDF GPU và hiển thị RViz.
Các bước xử lý chạy **replay tuần tự**, chưa phải pipeline trực tiếp chạy realtime.
ROS chỉ phục vụ cửa sổ RViz cuối cùng.

## Dữ liệu và chuyển động

Thư mục: `datasets/processed/odometry/full-pipeline-20261111`.
Scene `simulation/scenarios/indoor_tiny_house.xml`, physics 3000 Hz,
MPU-6500 model bật với unit seed 648. Robot bắt đầu home, STANDUP lúc 0,3 s,
HEIGHT 0.14 lúc 16 s, DRIVE 0.12 0 lúc 22 s và STOP lúc 26 s, kết thúc lúc 30 s.
Controller vẫn Balancing, fault_reason=none ở cuối. Không đặt lại pose robot
trong quá trình di chuyển. Encoder trong lượt này dùng cấu hình simulator hiện tại.

Chế độ mới `rb1_sim --capture-sync` ghi CAPTURE ngay tại lần lấy mẫu TOFMAP.
`capture_synchronized.py` dựng RGB 640×480 và ToF area rays từ chính trạng thái
vật lý đó. Pose mô phỏng chỉ dùng render, lấy mount camera tương đối thân và
đánh giá; không truyền pose chuẩn vào tracker hay tạo encoder giả từ quỹ đạo.
ToF dùng cùng mô hình histogram xấp xỉ và noise hiện có của bộ capture DELTAR.

Đoạn 20–30 s có 150 frame, trung bình ~15 Hz; timestamp odometry và ảnh khớp
chính xác trong log. Khoảng cách mẫu luân phiên 0,065/0,070 s do control 200 Hz.
Chiều dài đường đi camera tham chiếu 0,5464 m, gồm dao động khi cân bằng;
quỹ đạo ước lượng dài 0,5393 m. Cổ ở vị trí trung tính trong lượt này.

## Kết quả

| Chỉ số | Giá trị |
|---|---:|
| Frame được tracking | 150/150 |
| Visual correction bị từ chối | 0 |
| Frame có loop closure | 0 |
| ATE SE3, không fit scale | 0,00833 m |
| ATE căn theo pose đầu | **0,01544 m** |
| RPE tịnh tiến liên tiếp | 0,000467 m |
| RPE xoay liên tiếp | 0,0303° |
| Tracker median/p95 | 2,16 / 4,41 ms |
| Depth DELTAR RMSE trước lọc, đối chiếu mô phỏng | 0,5756 m |
| Pixel depth giữ lại sau lọc ToF | 29,57% |
| Mesh | 2.738 đỉnh / 4.917 mặt |
| TSDF voxel / thời gian dựng và xuất | 0,025 m / 12,28 s |

Timing tracker không gồm inference DELTAR, lọc ToF hoặc TSDF. Map LMDB, pose
JSON và mesh PLY/STL/OBJ đều đã lưu. Mesh được kiểm tra bằng ảnh preview, rồi
mở RViz riêng với ROS_DOMAIN_ID=43 để tránh trộn marker với các viewer trước.

## Vai trò của fusion trong phép thử

So sánh trên cùng dữ liệu trong `comparison.json`:

| Pose | ATE căn theo pose đầu |
|---|---:|
| Local odometry encoder + gyro | 0,02080 m |
| Visual cuVSLAM | 0,01544 m |
| Odometry với correction visual | 0,01544 m |

Mọi frame đều có visual pose hợp lệ. Với cách correction map_from_odom hiện tại,
pose global tại các timestamp visual trùng pose visual; vì vậy **phép thử này
không chứng minh cải thiện độ chính xác so với cuVSLAM đơn lẻ**. Local odometry
cung cấp đường liên tục để dùng giữa các frame/khi visual mất. Các trường hợp
mất visual và bù cổ quay đã có kiểm tra riêng, chưa được thử trong lượt vật lý này.
Không so trực tiếp 0,00833 m ở đây với lỗi 0,835 m của chuỗi sofa handheld cũ:
hai quỹ đạo và điều kiện quan sát khác nhau.

## Tệp kết quả

Trong thư mục dataset:

- `simulator.log`: lệnh, trạng thái vật lý, encoder/IMU và CAPTURE đồng bộ.
- `input/`: H5 RGB/ToF, calibration, mount camera từng frame, GT dành cho đánh giá.
- `prediction/`: depth DELTAR uint16 mm.
- `odometry/odometry.jsonl`: encoder/gyro fusion native.
- `slam/sequence.json`: visual pose, pose local/global và camera pose để dựng mesh.
- `slam/tracking.json`, `slam/evaluation.json`, `comparison.json`: số đo gốc.
- `slam/map/data.mdb`: map đặc trưng cuVSLAM.
- `slam/mesh/clay.stl`, `slam/clay_preview.png`: mesh và ảnh kiểm tra.
- `slam-failed-json/`: map từ lần đầu phát hiện lỗi serialize NumPy bool; đã sửa
  và chạy lại thành công trong `slam/`.

![Quỹ đạo](../datasets/processed/odometry/full-pipeline-20261111/slam/trajectory.png)

## Chạy lại

Từ root repo, dùng thư mục output mới:

```bash
RUN=datasets/processed/odometry/full-pipeline-new
mkdir -p "$RUN"
firmware/build/bin/rb1_sim --model simulation/scenarios/indoor_tiny_house.xml \
  --rate 3000 --start-key home --imu-model 1 --imu-unit 648 --capture-sync \
  --run-for 30 --at '0.3:STANDUP' --at '16:HEIGHT 0.14' \
  --at '22:DRIVE 0.12 0' --at '26:STOP' > "$RUN/simulator.log"
firmware/build/bin/rb_tof_mapper "$RUN/odometry" < "$RUN/simulator.log"
MUJOCO_GL=egl env -u PYTHONPATH /home/dvt/.venvs/deltar/bin/python \
  technology/scan3d_fusion/capture/capture_synchronized.py "$RUN/simulator.log" "$RUN/input"
env -u PYTHONPATH /home/dvt/.venvs/deltar/bin/python \
  technology/scan3d_fusion/inference/run_demo.py --data-root "$RUN/input" \
  --output "$RUN/prediction" --max-frames 0
SLAM=technology/scan3d_fusion/mapping/cuvslam
$SLAM/run_python.sh $SLAM/track_rgbd.py "$RUN/input" "$RUN/prediction" "$RUN/slam" \
  --odometry "$RUN/odometry/odometry.jsonl"
$SLAM/run_python.sh $SLAM/evaluate.py "$RUN/input" "$RUN/slam/sequence.json"
env -u PYTHONPATH /home/dvt/.venvs/deltar/bin/python \
  technology/scan3d_fusion/reconstruction/reconstruct.py \
  "$RUN/input" "$RUN/prediction" "$RUN/slam/mesh" \
  --trajectory "$RUN/slam/sequence.json" --voxel .025
env ROS_DOMAIN_ID=43 technology/scan3d_fusion/viewer/run_clay_rviz.sh "$RUN/slam/mesh/clay.stl"
```

## Giới hạn

Đã kiểm tra end-to-end trên đoạn thẳng ngắn, chưa quét cả nhà, chưa loop closure,
chưa quay cổ trong lần thu này và chưa chạy trực tiếp trên robot thật. Mesh thưa,
chủ yếu là mảng sàn và vài mảnh hình học: chỉ quan sát một hướng, depth có nhiều
pixel bị loại. Đây là xác nhận pipeline hoạt động, chưa phải bản đồ nhà hoàn chỉnh
hay xác nhận đủ chất lượng navigation. Noise/covariance odometry còn cần hiệu chuẩn.
