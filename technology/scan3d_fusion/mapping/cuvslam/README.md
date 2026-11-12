# RGB-D SLAM cho RB, không dùng ROS

Adapter này dùng cuVSLAM **17.0.1+cu12** (release commit
`51631ba358ea51262342f9ae649820d639363eb9`). Mã tham chiếu ở
`prj_reference/cuVSLAM`. API release khác nhánh main; không cập nhật wheel độc lập.

Luồng hiện chạy được bằng dữ liệu đã ghi:

```
RGB uint8 + DELTAR depth + vùng ToF/calibration
  → lọc/hiệu chỉnh depth dùng chung với TSDF
  → cuVSLAM RGBD odometry + SLAM/loop closure trên GPU
  → pose cuối sau tối ưu + map LMDB
  → TSDF dựng lại bằng pose ước lượng
```

`track_rgbd.py` không đọc depth ground truth trong H5 và không truyền pose ground
truth cho tracker. `evaluate.py` là bước riêng đọc pose MuJoCo để chấm sai số.
Các map trước dùng pose biết sẵn vẫn có thể chạy như cũ.

**Kết quả chạy thực tế:** 432/432 frame, ATE toàn chuỗi **0,835 m**, mesh còn méo.
Theo dõi đủ frame không có nghĩa vị trí chính xác. Xem [báo cáo kiểm thử](../../../../evaluation/cuvslam_rgbd_results.md)
trước khi dùng kết quả; baseline này chưa đạt chất lượng navigation hay quét sofa hoàn chỉnh.

## Cài trên PC hiện tại

Ubuntu 24.04 x86_64, Python 3.12, NVIDIA GPU. Tạo môi trường riêng với DELTAR:

```bash
env -u PYTHONPATH /usr/bin/python3 -m venv /home/dvt/.venvs/cuvslam
env -u PYTHONPATH /home/dvt/.venvs/cuvslam/bin/python -m pip install \
  -r technology/scan3d_fusion/environment/requirements-cuvslam.txt
```

`run_python.sh` chọn thư viện CUDA trong môi trường này và bỏ `PYTHONPATH` ROS
đang được shell kế thừa. Có thể đặt `CUVSLAM_PYTHON` nếu dùng đường dẫn khác.
Runtime SLAM không import ROS. Repo tham chiếu giữ nguyên giấy phép NVIDIA;
release này yêu cầu phần cứng NVIDIA theo LICENSE của upstream.

## Chạy lại dữ liệu đã ghi

Từ root workspace (output phải là thư mục mới):

```bash
SCAN=datasets/processed/sofa_clay/current-sofa-432views-20261111-124227
SLAM=technology/scan3d_fusion/mapping/cuvslam
$SLAM/run_python.sh $SLAM/track_rgbd.py "$SCAN/input" "$SCAN/prediction" "$SCAN/cuvslam"
$SLAM/run_python.sh $SLAM/evaluate.py "$SCAN/input" "$SCAN/cuvslam/sequence.json"

env -u PYTHONPATH /home/dvt/.venvs/deltar/bin/python \
  technology/scan3d_fusion/reconstruction/reconstruct.py \
  "$SCAN/input" "$SCAN/prediction" "$SCAN/cuvslam/mesh" \
  --trajectory "$SCAN/cuvslam/sequence.json" --voxel .025

MUJOCO_GL=egl env -u PYTHONPATH /home/dvt/.venvs/deltar/bin/python \
  technology/scan3d_fusion/reconstruction/render_mesh.py \
  "$SCAN/cuvslam/mesh/clay.obj" --output "$SCAN/cuvslam/clay_preview.png"
technology/scan3d_fusion/viewer/run_clay_rviz.sh "$SCAN/cuvslam/mesh/clay.stl"
```

Depth uint16 được chia **1000** để ra mét. `K` và kích thước ảnh lấy từ capture;
RGB/depth phải căn chỉnh pixel và cùng timestamp. Giữ timestamp gốc, không giả
lập FPS cao hơn bằng cách sửa thời gian. Chuỗi sofa cũ cách khung ~0,533 giây,
không đại diện cho luồng 15 Hz trực tiếp. Quỹ đạo ở map riêng: gốc camera đầu tiên,
X hướng trước/Y trái/Z lên khi camera đầu tiên nằm ngang. Không dùng
`--compare-sofa` với mesh này vì sofa scene và map SLAM có gốc tọa độ khác nhau.

## Đầu ra và mất tracking

- `sequence.json`: pose cuối đã tính lại sau tối ưu pose graph. TSDF chỉ đọc các
  frame này khi chỉ định `--trajectory`; không thay pose bị mất bằng GT/odometry.
- `tracking.json`: số khung tracked, frame thất bại, thống kê loop closure và
  thời gian gọi tracker; timing không gồm inference DELTAR/lọc ToF.
- `map/`: database SLAM cuVSLAM, dùng cho API save/localize của release.
- `evaluation.json`, `trajectory.png`: ATE theo SE3 và theo pose đầu; RPE giữa
  các khung liên tiếp. Không fit scale. Luôn xem coverage cùng sai số.
- `mesh/`: mesh TSDF từ depth fusion và pose SLAM, có thể chứa các bề mặt nền
  trong ảnh. Bounds lấy từ depth quan sát, không lấy kích thước sofa ground truth.

Nếu tracking mất, adapter dừng và trả exit code 2, vẫn ghi đoạn hợp lệ và báo
frame lỗi. Không nối tiếp mù qua một lần tracker đổi gốc tọa độ.

## Cổ robot và tích hợp luồng ảnh

`RgbdSlam.track(timestamp_ns, rgb, depth_mm)` là giao diện có thể gọi từ nguồn
RGB-D khác, không cần MuJoCo. Cấu hình hiện là replay đồng bộ để kết quả đầy đủ
khi lấy pose cuối. Chưa nối vào vòng điều khiển RB hay nguồn ảnh trực tiếp.

Camera là rig SLAM. Khi có phép biến đổi `body_from_camera(t)` từ encoder cổ và
calibration, `body_pose()` tính:

```
world_from_body = world_from_camera × inverse(body_from_camera(t))
```

Capture mới ghi kinematics này từng khung; capture cũ thiếu nó thì chỉ xuất pose
camera. Không dùng IMU thân làm IMU cố định của đầu quay. Chưa bật IMU,
planar constraints hay VPR của nhánh main trong baseline release này.

## Kiểm tra

```bash
technology/scan3d_fusion/mapping/cuvslam/run_python.sh -m unittest discover \
  -s technology/scan3d_fusion/mapping/cuvslam -p test_adapter.py -v
env -u PYTHONPATH /home/dvt/.venvs/deltar/bin/python \
  technology/scan3d_fusion/reconstruction/test_geometry.py
```

Các bài kiểm tra bảo vệ đổi trục camera, offset camera trên cổ quay và đánh giá
trajectory không che sai số scale. Replay thực tế là kiểm tra tích hợp GPU/API/map.
Chất lượng depth DELTAR, texture RGB và góc nhìn vẫn quyết định chất lượng map;
cuVSLAM không sửa trực tiếp bề mặt depth sai.

## Odometry bánh và IMU

Đã có `RgbdSlam.track_body()` để nhận packet odometry đồng bộ và mount camera theo thời gian, hiệu chỉnh local odometry bằng pose visual. Xem [module odometry](../odometry/README.md). Replay sofa cũ vẫn chỉ RGB-D vì thiếu encoder/IMU đồng bộ; chưa đo cải thiện ATE bằng fusion này.

## Replay đầy đủ với encoder/IMU

Thêm `--odometry PATH/odometry.jsonl` khi chạy `track_rgbd.py`. Capture phải có
`body_from_camera` và timestamps cùng nguồn với odometry (sai lệch tối đa 20 ms).
Leading MAPRESET được phép; reset giữa log phải chia thành session riêng.
Adapter giữ `visual_camera_to_world` để đối chiếu, xuất camera pose sau correction
trong `camera_to_world` và pose local/global trong `localization`. Chạy TSDF với
`--trajectory` như bình thường. Xem [lượt chạy đầy đủ](../../../../evaluation/full_pipeline_results.md).
