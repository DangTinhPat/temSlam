# Kiến trúc mô phỏng SLAM

## Mục tiêu

Tạo vòng lặp kín để RB giữ thăng bằng, di chuyển trong môi trường mô phỏng và phát dữ liệu cảm biến cho SLAM. Tầng SLAM phải xử lý cùng loại đầu vào mà robot sẽ có ngoài đời; không được đọc pose hoặc bản đồ nội bộ của MuJoCo.

## Các lớp và quyền truy cập dữ liệu

```text
model/ ──► simulation/world + robot plant ──► simulated sensors ──► slam/ ──► pose/map estimate
                        │                                           │
                        └──► evaluation/ground truth                └──► evaluation/metrics
```

| Khu vực | Trách nhiệm | Được đọc ground truth? |
| --- | --- | --- |
| `model/` | Mô tả hình học, khối lượng và cảm biến gắn trên RB | Không áp dụng |
| `simulation/` | Tích hợp plant, thế giới, kịch bản, clock và mô hình cảm biến | Chỉ bộ ghi ground truth riêng được đọc trạng thái thật |
| `slam/` | Tiền xử lý cảm biến, odometry/visual odometry, backend và bản đồ | Không |
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

Hiện repo đã có plant MuJoCo, XML robot, IMU mô phỏng và runner điều khiển trong `simulation/mujoco/`. Runner vẫn liên kết tới thư viện `firmware/control`; mã controller không bị thay đổi. Chưa có world/scenario SLAM, camera/LiDAR giả lập, recorder chuẩn, transport cảm biến hoặc SLAM backend. Các mục đó cần được thêm trong lớp mô phỏng/SLAM, không bằng cách sửa controller.

## Trình tự phát triển đề xuất

1. Chốt sensor suite và SLAM backend; xác định frame, rate, calibration và loại nhiễu.
2. Thêm thế giới/kịch bản có texture và hình học đủ đặc trưng, có thể tái lập bằng seed.
3. Xuất sensor stream và ghi bag/log cùng metadata; kiểm tra SLAM runtime không subscribe ground truth.
4. Nối SLAM qua adapter, lưu estimate và map.
5. Thêm evaluator ngoại tuyến đọc ground truth tách biệt để tính ATE/RPE hoặc metric phù hợp.

## Quy ước thư mục

- `simulation/scenarios/`: cấu hình từng thí nghiệm, không chứa thuật toán SLAM.
- `simulation/sensors/`: cấu hình và implementation sensor model.
- `slam/`: code SLAM và adapter input/output, không import API MuJoCo.
- `datasets/raw/`: log cảm biến bất biến sau khi ghi; không commit dữ liệu lớn nếu chưa cần.
- `datasets/processed/`: dữ liệu dẫn xuất/cache, có thể tái tạo từ raw.
- `evaluation/`: script đọc estimate và ground truth ngoại tuyến.
