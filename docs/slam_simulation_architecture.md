# Kiến trúc mô phỏng SLAM

## Mục tiêu

Tạo vòng lặp kín để RB giữ thăng bằng, di chuyển trong môi trường mô phỏng và phát dữ liệu cảm biến cho SLAM. Tầng SLAM phải xử lý cùng loại đầu vào mà robot sẽ có ngoài đời; không được đọc pose hoặc bản đồ nội bộ của MuJoCo.

## Các lớp và quyền truy cập dữ liệu

```text
model/ ──► simulation/world + robot plant ──► simulated sensors ──► technology/scan3d_fusion/mapping/ ──► pose/map estimate
                        │                                           │
                        └──► evaluation/ground truth                └──► evaluation/metrics
```

| Khu vực | Trách nhiệm | Được đọc ground truth? |
| --- | --- | --- |
| `model/` | Mô tả hình học, khối lượng và cảm biến gắn trên RB | Không áp dụng |
| `simulation/` | Tích hợp plant, thế giới, kịch bản, clock và mô hình cảm biến | Chỉ bộ ghi ground truth riêng được đọc trạng thái thật |
| `technology/scan3d_fusion/mapping/` | Sensor adapter, ToF odometry mapper, route và viewer bridge | Không |
| `datasets/` | Lưu dữ liệu cảm biến và metadata của lần chạy | Tách ground truth khỏi tập đầu vào |
| `evaluation/` | Đồng bộ kết quả ước lượng với quỹ đạo/bản đồ thật và tính metric | Có |
| `firmware/` | Bộ điều khiển hiện tại và mã phần cứng | Không phải module SLAM |

## Hợp đồng dữ liệu

1. Mô phỏng xuất một luồng đầu vào cảm biến có timestamp và frame rõ ràng. Mô hình cảm biến phải cho phép cấu hình rate, độ trễ, nhiễu, bias, dropout và seed.
2. SLAM nhận luồng cảm biến qua interface độc lập với MuJoCo. Adapter có thể đổi transport mà không sửa thuật toán.
3. Output của SLAM (pose, covariance nếu có, map, trạng thái tracking) được ghi riêng với sensor stream.
4. Ground truth được ghi vào namespace hoặc file riêng trong `evaluation/`. Không đưa topic/file ground truth vào launch/runtime SLAM.
5. Báo cáo đánh giá ghi rõ alignment/scale handling và metric; phép căn chỉnh ngoại tuyến chỉ dùng sau khi SLAM đã xuất estimate.

## Ranh giới hiện tại

Hiện có cảnh indoor Small House, lõi ToF-only trong `technology/scan3d_fusion/mapping/tof_only/`, luồng record `TOFMAP` và cầu nối GUI/RViz trong `technology/scan3d_fusion/mapping/native/`. GUI tự mở map viewer, ghi sensor log và cho replay không cần MuJoCo. Lõi mapping không liên kết firmware/MuJoCo/ROS. Đây là mapping theo odometry; scan matching và loop closure chưa triển khai. Xem `evaluation/tof_mapping_results.md` để biết phạm vi kiểm tra và các giới hạn.

## Trình tự phát triển đề xuất

1. Chốt sensor suite và SLAM backend; xác định frame, rate, calibration và loại nhiễu.
2. Thêm thế giới/kịch bản có texture và hình học đủ đặc trưng, có thể tái lập bằng seed.
3. Xuất sensor stream và ghi bag/log cùng metadata; kiểm tra SLAM runtime không subscribe ground truth.
4. Nối SLAM qua adapter, lưu estimate và map.
5. Thêm evaluator ngoại tuyến đọc ground truth tách biệt để tính ATE/RPE hoặc metric phù hợp.

## Quy ước thư mục

- `simulation/scenarios/`: cấu hình từng thí nghiệm, không chứa thuật toán SLAM.
- `simulation/sensors/`: cấu hình và implementation sensor model.
- `technology/scan3d_fusion/`: source, cấu hình, model vision và mapper ToF; ngoại trừ adapter mô phỏng, lõi mapper không import API MuJoCo.
- `datasets/raw/`: log cảm biến bất biến sau khi ghi; không commit dữ liệu lớn nếu chưa cần.
- `datasets/processed/`: dữ liệu dẫn xuất/cache, có thể tái tạo từ raw.
- `evaluation/`: script đọc estimate và ground truth ngoại tuyến.
