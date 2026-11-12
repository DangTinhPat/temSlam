# Tuyến bao quanh phòng khách — 2026-11-12

Tuyến cũ dùng cột lưới ngoài cùng X=-5,25 m. Ở khoảng Y=-3,1…-1,9 m, cột này
vướng footprint sofa sau khi cộng biên robot, nên các mục tiêu bị bỏ qua. Kiểm
tra vật lý cho thấy dải X=-5,4 m vẫn đi được. Lưới lập tuyến đổi từ 0,25 xuống
0,15 m để biểu diễn lối này; không di chuyển đồ vật hoặc thay collider.

Tuyến mới: điểm xuất phát → cạnh đông → cạnh nam → cạnh tây qua sau sofa → cạnh
bắc → khép vòng ngoài → các dải ziczac bên trong → điểm xuất phát. Tổng 96,3 m,
184 waypoint, 66 điểm dừng lia đầu. Đoạn sát tường giới hạn 0,12 m/s; bên trong
0,20 m/s. Các đoạn nối kiểm tra mỗi 5 cm bằng probe bán kính 0,25 m.

- Lượt thử controller độc lập hoàn tất 184/184 điểm, giữ Balancing. Quỹ đạo đã
  đi và kế hoạch: [living_room_executed_route.png](living_room_executed_route.png).
- Luồng đầy đủ hoàn tất 184/184 điểm, ×5, 483 keyframe, 562,1 giây thực (chưa tính
  xuất mesh). Phiên: `datasets/raw/tof-20261112-094256-483079`.
- Có 8 keyframe depth hợp lệ khi thân robot ở lối sau sofa (X<-5,25 m,
  -3,1<Y<-1,9 m); phiên ziczac trước có 0. Chi tiết:
  [living_room_behind_sofa_views.json](living_room_behind_sofa_views.json).
- Mesh cuối đã xuất và mở trong RViz. Sàn quan sát được chấp nhận 21,598 m²,
  sàn suy ra 1,143 m². Cả hai nhỏ hơn bản ziczac đã tinh chỉnh trước (25,008 và
  2,945 m²). Vì vậy chỉ khẳng định tuyến đã lấy thêm góc nhìn, không khẳng định
  độ kín mesh tăng. Diện tích theo lưới pose ước lượng không phải phần trăm phủ
  ground truth; số đo depth, lịch keyframe và ghép pose vẫn ảnh hưởng kết quả.
- 10 kiểm tra route controller đã qua. Bộ sinh tuyến tự kiểm tra không dời
  mục tiêu, không có đoạn bị chặn, có điểm dừng sau sofa.

Tuyến ziczac cũ giữ trong `mapping/native/living_room_interior_route.json` dưới
`technology/scan3d_fusion`. Phiên mới lưu `scan_route.json` để tuyến không bị
mất dấu khi sửa cấu hình về sau.

Một lượt thử SLAM trước đó dừng do giới hạn bước quay cổ quá nhỏ khi GPU đang
xử lý; dữ liệu dở giữ riêng tại `datasets/raw/tof-20261112-093932-477254`.
Đã đổi giới hạn bù thời gian thành bước tối đa 8° mỗi lệnh, giữ tốc độ lia danh
định 20°/s và chặn gửi tiếp khi cổ chưa theo kịp. Phiên hoàn tất nêu trên dùng
bản này; không tính lượt dở là kết quả hoàn chỉnh.

Đây là tuyến cố định trong phòng khách đã ngăn riêng, dùng scene/pose mô phỏng
để lập và bám tuyến; không phải navigation khám phá không gian chưa biết.
SLAM dựng mesh từ dữ liệu cảm biến và pose ước lượng, không lấy hình học scene
để vá map. Thêm đường đi không bảo đảm quan sát mặt trên đồ vật hoặc mọi vùng
bị che khuất; không suy tỷ lệ phủ 3D từ số waypoint đã hoàn thành.
