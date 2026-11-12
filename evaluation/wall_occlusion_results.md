# Kiểm tra điểm xuyên tường — 2026-11-12

Phiên nguồn: `datasets/raw/tof-20261112-094256-483079`.
110 tia từ hai phía của 11 wall/partition tại cao độ cảm biến đều gặp mặt tường
trong 30 cm hoặc một vật gần hơn. Chưa tìm thấy xuyên tường vật lý trong phép
thử này. Chạy lại bằng `evaluation/check_wall_rays.py` trong môi trường DELTAR,
với `PYTHONPATH` unset và `MUJOCO_GL=egl`.

Kiểm tra 32 keyframe rải đều, dùng depth/segmentation chuẩn chỉ làm oracle đánh
giá: trong 1.919.226 pixel hợp lệ thực sự nhìn tường, có 36.490 pixel dự đoán
sâu hơn mặt tường >8 cm, 4.778 pixel >20 cm; đuôi sai số tới 1,42 m.

Bộ lọc cũ OR các rectangle ToF chồng nhau và dùng 3 sigma không chặn biên trên,
nên vùng xa hoặc phân bố rộng có thể chấp nhận điểm sau bề mặt gần. Bộ lọc mới
chiếu điểm 3D vào đúng ô ToF, chặn dung sai ở 15 cm và áp dụng occlusion bảo thủ
giữa các rectangle. Thử trên cùng depth: còn 30.131 pixel >8 cm và 3.098 pixel
>20 cm (giảm 35,2% nhóm >20 cm), giữ 1.803.614 pixel tường hợp lệ. Chi tiết:
`wall_measurement_audit.json`, `wall_filter_audit.json`.

Đã đưa bộ lọc vào `tof_support` dùng chung và kiểm tra lần nữa sau floor support
trong live. 7 kiểm tra depth support/geometry qua. Keyframe mới giữ các thống kê
ToF để kiểm tra lại mà không tái render. Phép kiểm tra ray dùng cùng chính sách
geom group như sensor; không khẳng định đã kiểm tra mọi tia của toàn bộ phiên.

Bản đối chiếu toàn bộ 483 keyframe ở
`datasets/processed/scan3d_fusion/wall-filter-20261112`.
Giữ nguyên pose ước lượng đã ghép bề mặt; tái tạo histogram ToF từ snapshot
simulator và **chỉ loại** depth không phù hợp. Không chèn depth chuẩn hay clip
map theo biên CAD. Pixel depth giữ lại: 75.106.524 / 82.405.974 (~91,1%).
Lệnh tái tạo nằm trong `evaluation/refilter_sim_depth.py`; nguồn giữ nguyên.

Lỗi chưa hết: parallax/occlusion giữa hai cảm biến, depth trong vùng đo ToF thưa,
và drift pose vẫn có thể tạo điểm sau tường. Loại điểm làm tăng vùng hở. Chưa có
vị trí người dùng chỉ cụ thể để tái hiện đúng quan sát của họ; chưa chạy lại toàn
tuyến sau thay đổi, chỉ reprocess dữ liệu đã lưu.
