# cuVSLAM RGB-D native — kết quả tích hợp 2026-11-11

Đã tích hợp và chạy hết luồng **RGB + ToF → DELTAR → cuVSLAM → TSDF** trên
RTX 3060, không dùng ROS trong xử lý. Đây là baseline replay có kiểm chứng;
**chưa đạt chất lượng map cho navigation hoặc mesh sofa hoàn chỉnh**.

## Thành phần đã nối

- [Adapter cuVSLAM](../technology/scan3d_fusion/mapping/cuvslam/track_rgbd.py)
  dùng release `17.0.1+cu12`, RGB-D odometry, SLAM, loop closure và tối ưu pose graph.
- RGB 640×480, depth DELTAR căn chỉnh theo RGB; dùng chung bộ hiệu chỉnh/lọc ToF
  với TSDF. Depth cấp cho cuVSLAM là uint16 mm, hệ số chia 1000.
- SLAM chạy GPU trong môi trường Python 3.12 riêng tại `/home/dvt/.venvs/cuvslam`.
  DELTAR/TSDF dùng môi trường cũ. `pip check` không phát hiện phụ thuộc hỏng.
- Xuất lại **toàn bộ pose cuối sau tối ưu** rồi TSDF tích phân lại từ đầu;
  không giữ mesh ghép theo pose cũ trước loop closure.
- Lưu map LMDB thành công (~57 MB), pose JSON, thống kê, PLY/STL/OBJ và ảnh preview.
- Có chuyển pose camera sang thân theo `body_from_camera(t)`. Capture mới ghi
  kinematics này; capture dùng trong bài thử cũ chưa có nên chỉ xuất pose camera.
  Không gán IMU thân cố định vào rig đầu quay.

Nguồn tham chiếu: [cuVSLAM v17.0.1](https://github.com/nvidia-isaac/cuVSLAM/tree/v17.0.1),
commit `51631ba358ea51262342f9ae649820d639363eb9`, clone tại `prj_reference/cuVSLAM`.
API của release khác nhánh main; wheel và adapter phải cập nhật cùng nhau.

## Bài thử chính

Dataset: `datasets/processed/sofa_clay/current-sofa-432views-20261111-124227`.
Camera kinematic đi ba vòng quanh sofa, cao 0,5 / 1,1 / 1,8 m. Timestamp gốc cách
nhau ~0,533 s (~1,875 Hz), **không phải bài thử luồng trực tiếp 15 Hz hoặc robot
đang cân bằng**. Không sửa timestamp để tạo FPS giả.

| Chỉ số | Kết quả |
|---|---:|
| Frame có pose / tổng số | 432 / 432 |
| Frame có cờ loop closure thành công | 2 |
| ATE RMSE, căn chỉnh rigid SE3, không fit scale | **0,835 m** |
| ATE RMSE, chỉ căn theo pose đầu | 2,909 m |
| RPE tịnh tiến giữa hai frame liền kề | 0,0447 m |
| RPE xoay giữa hai frame liền kề | 1,944° |
| Tổng chiều dài quỹ đạo tham chiếu / ước lượng | 28,422 / 25,771 m |
| Gọi tracker: median / p95 | 3,54 / 19,80 ms |
| Pixel depth được giữ sau lọc ToF | 59,88% |
| TSDF: voxel / số mặt mesh | 0,025 m / 95.997 |
| Thời gian tích phân và xuất mesh | 47,70 s |

Thời gian tracker **không gồm DELTAR, lọc ToF, I/O hay dựng mesh**. Hai frame có cờ
loop closure không được dùng để suy ra hai vòng lặp độc lập. Số frame thành công
chỉ cho biết tracker trả pose, không xác nhận pose đủ chính xác.

Tracker chính không đọc depth ground truth và không nhận pose ground truth.
MuJoCo pose chỉ được dùng sau đó trong evaluator. Căn chỉnh SE3 chỉ phục vụ
đánh giá/đồ thị; không dùng phép căn chỉnh này để thay pose trong mesh.

## Kiểm tra nguyên nhân sai số

Đã chạy thêm một **đối chứng riêng dùng depth chuẩn mô phỏng**, vẫn để cuVSLAM tự
ước lượng pose: 432/432 frame, ATE **0,654 m**, RPE tịnh tiến 0,0247 m và RPE xoay
0,615°. Đầu ra nằm trong `cuvslam-depth-control/`, không thay đầu ra fusion chính.
Điều này cho thấy thay depth bằng depth lý tưởng vẫn chưa giải quyết hết lỗi.

Phân tích từng đoạn giữ nguyên pose từ lượt chạy đầy đủ, căn chỉnh SE3 riêng
từng đoạn, cho ATE sau (chỉ dùng chẩn đoán, không thay số đo toàn chuỗi):

| Vùng quét, chỉ số frame 0-based | Depth fusion | Depth chuẩn đối chứng |
|---|---:|---:|
| Vòng thấp 0–129 | 0,951 m | 0,568 m |
| Vòng giữa 159–273 | 0,067 m | 0,106 m |
| Vòng trên 303–431 | 0,031 m | 0,031 m |

Ảnh vòng thấp nhìn sát phần gầm sofa, nhiều mảng tối, ít chi tiết và hình dạng
lặp lại. Kết quả gợi ý đoạn này là điểm yếu chính của tracking; chưa đủ bằng
chứng để quy toàn bộ lỗi cho một nguyên nhân. Cần thu luồng liên tục đúng tần số,
giữ đủ chi tiết RGB và đánh giá lại trước khi nối với điều khiển/navigation.

## Tệp để xem và chạy lại

Trong thư mục dataset trên:

- `cuvslam/tracking.json`, `cuvslam/evaluation.json`: số đo gốc.
- `cuvslam/sequence.json`: 432 pose SLAM cuối, gốc camera đầu tiên.
- `cuvslam/map/data.mdb`: map đặc trưng cuVSLAM; không phải mesh TSDF.
- `cuvslam/trajectory.png`: quỹ đạo tham chiếu và ước lượng.
- `cuvslam/mesh/clay.{ply,stl,obj,json}`: mesh từ depth fusion + pose SLAM.
- `cuvslam/clay_preview.png`: preview đã kiểm tra; bề mặt còn méo và hở.
- `cuvslam-depth-control/`: pose/evaluation của phép thử depth chuẩn, tách biệt.

![Quỹ đạo toàn chuỗi](../datasets/processed/sofa_clay/current-sofa-432views-20261111-124227/cuvslam/trajectory.png)

Lệnh cài và chạy đầy đủ ở [README adapter](../technology/scan3d_fusion/mapping/cuvslam/README.md).
Output tracking phải là thư mục mới để không ghi đè map. Không mở chế độ
`--compare-sofa` với mesh SLAM vì nó không cùng gốc tọa độ với scene tham chiếu.

## Kiểm tra và giới hạn

- 3 test adapter đạt: đổi trục quang học, bù cổ quay/offset camera, đánh giá SE3
  không che lỗi scale.
- 4 test hình học hiện có đạt: vùng ToF, pose rig, depth support và TSDF.
- Replay GPU toàn chuỗi, lưu map, đọc pose cuối và dựng mesh đều đã chạy thành công.
- Chưa nối nguồn ảnh trực tiếp, chưa chạy robot cân bằng/đi lại với pose này,
  chưa kiểm tra relocalization bằng map đã lưu hoặc trên phần cứng robot thực.
- Không bật planar constraint/IMU; không thêm tính năng VPR của nhánh main.
- Không nên dùng map hiện tại cho navigation: drift đầu chuỗi và depth không
  ổn định làm bề mặt bị méo dù tracker không báo mất bám.
