# Bản đồ ToF nháp đã lưu

- `map-3d.png`: xem trước 3D kiểu ArduBot: voxel màu theo độ cao, lớp chiếu xám bên dưới, quỹ đạo xanh.
- `map.ply`: 5.997 voxel vật cản có tọa độ X/Y/Z; mở và xoay trong CloudCompare,
  MeshLab hoặc công cụ đọc PLY.
- `preview.png`: bản đồ chiếu 2D kèm quỹ đạo màu xanh và vị trí cuối màu đỏ.
- `map.png`, `map.pgm`: lưới xám gốc 320 × 320, mỗi ô 5 cm (16 × 16 m).
- `trajectory.csv`: quỹ đạo; `sensors.log`: dữ liệu ToF/odometry để phát lại.
- `metadata.json`, `tof.yaml`, `snapshot.json`: cấu hình, nguồn phiên và kiểm tra toàn vẹn.

Màu tối là vật cản, trắng là vùng đã quan sát trống, xám là chưa biết.
Đây là bản đồ nháp theo odometry, chưa hiệu chỉnh vòng lặp; vùng trắng không
bảo đảm đủ khoảng hở cho toàn bộ thân robot. `state.json` có `valid=0` vì lượt
phát lại đã kết thúc, không phải chỉ báo robot bị lỗi.

Lưới nội bộ có chiều cao 16 lớp; PLY giữ riêng từng voxel X/Y/Z với bước 5 cm.
Ảnh map 2D chỉ là hình chiếu bổ sung.

Bản chụp được tái dựng từ phần log hoàn chỉnh cố định, nên map và quỹ đạo
đồng nhất. Phiên mô phỏng trực tiếp tiếp tục chạy và không ghi đè thư mục này.
