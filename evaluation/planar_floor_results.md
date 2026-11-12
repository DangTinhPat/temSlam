# Sàn phẳng liên tục — 2026-11-12

**Thí nghiệm đã loại khỏi luồng SLAM chính:** sàn bổ sung là suy diễn, không
đáp ứng yêu cầu hình học bản đồ chỉ đến từ RGB + ToF fusion. Các số liệu dưới
đây mô tả thí nghiệm cũ, không phải mức độ hoàn chỉnh của fusion. Xem
[bản sửa nguồn dữ liệu](fusion_only_results.md).

## Tham khảo mã nguồn

- [HDL Graph SLAM: floor_detection_nodelet.cpp](https://github.com/koide3/hdl_graph_slam/blob/master/apps/floor_detection_nodelet.cpp): lọc theo chiều cao/pháp tuyến, RANSAC, kiểm tra mặt phẳng sàn và hướng pháp tuyến. Kế thừa cách mô hình hóa sàn thành một mặt phẳng có bằng chứng; không tích hợp ROS vào core, không nhập thuật toán tối ưu graph của dự án.
- [PCL: Construct a concave or convex hull polygon for a plane model](https://pointclouds.org/documentation/tutorials/hull_2d.html): chiếu điểm lên mặt phẳng rồi dựng biên 2D. Ở đây dùng đường bao lưới và phép khép hình thái từ SciPy đang cài sẵn; không phải gọi trực tiếp PCL/alpha-shape.
- [Open3D: Surface reconstruction](https://www.open3d.org/docs/release/tutorial/geometry/surface_reconstruction.html#poisson-surface-reconstruction): Poisson làm mượt và có thể suy diễn ở nơi thưa điểm. Không dùng Poisson cho toàn cảnh trong thay đổi này: riêng sàn phẳng có thể biểu diễn trực tiếp, tránh làm phồng vật thể hoặc tự đóng cửa/hốc.

## Nguyên nhân và thay đổi

Trước đây `complete_floor.py` dùng cả dấu vết vật cản trên cao để khoét sàn.
Vì vậy bàn/ghế và outlier tạo lỗ ở mặt sàn. Giới hạn vá 60 cm khiến những lỗ
lớn bị giữ lại dù đã nằm bên trong đường bao sàn quan sát.

Nay chiếu các ô có ít nhất hai khung đo sàn lên một mặt phẳng chung; khép vết
nứt biên bằng đĩa bán kính 25 cm, bỏ cụm dưới 0,04 m² rồi lấp khoảng bên trong.
Không dùng convex hull toàn phòng. Vật cản vẫn giữ mesh riêng, sàn bên dưới
được đánh dấu suy ra thay vì khoét lỗ. Chỉ khoét chỗ thấp hơn mặt phẳng 6 cm
khi có ít nhất ba khung và số khung này vượt số khung báo sàn, kèm biên 5 cm.

Các mặt sàn cũ gần mặt phẳng, trong miền được thay thế, được bỏ khỏi lớp vật
thể để tránh chồng bề mặt. RViz mặc định hiện lớp sàn tổng cùng màu đất sét;
có thể tắt lớp tổng rồi bật hai lớp đo/suy ra để kiểm tra bằng chứng. Ảnh
MuJoCo của bản sàn phẳng tắt shadow-map vì các tam giác đồng phẳng gây bóng
tự che loang lổ; thao tác này không đổi mesh. RViz không dùng bóng đổ này.

## Dữ liệu và kết quả

Nguồn giữ nguyên: `datasets/processed/scan3d_fusion/wall-filter-20261112`.
Kết quả: `datasets/processed/scan3d_fusion/planar-floor-20261112`.
483 keyframe, depth và pose surface-matched giữ nguyên. Chỉ đọc RGB-D đã lưu,
floor-plane của keyframe và pose ước lượng; không đọc CAD, depth chuẩn hoặc
pose thật. Keyframes liên kết tới nguồn nên cần giữ nguồn khi lưu trữ.

| Chỉ số | Trước | Sau |
|---|---:|---:|
| Tổng diện tích mặt nạ sàn | 18,1375 m² | 45,0175 m² |
| Cụm lỗ kín bên trong đường bao | 46 | 0 |
| Diện tích lớp sàn quan sát | 16,0700 m² | 16,0700 m² |
| Diện tích lớp sàn suy ra | 2,0675 m² | 28,9475 m² |

Sàn mới có một thành phần, 36.014 tam giác, Euler characteristic = 1 (miếng
mặt phẳng không có vòng thủng bên trong). Sai lệch đỉnh khỏi mặt phẳng sau
đọc lại STL <1e-6 m. Không có ô thỏa điều kiện hố xác nhận trong phiên này;
kiểm tra tổng hợp xác nhận một hố có bằng chứng vẫn được giữ. Hai lớp sàn
không trùng nhau. Các diện tích trên theo pose ước lượng, không phải tỷ lệ
phủ hoặc độ chính xác so với toàn bộ phòng thật.

9 kiểm tra floor support/completion đã qua, gồm lỗ lớn được bao quanh, biên
lõm mở, hố lặp nhiều khung, outlier dưới sàn, mặt phẳng nghiêng và topology mesh.
Số liệu: [planar_floor_results.json](planar_floor_results.json).

```bash
env -u PYTHONPATH /home/dvt/.venvs/deltar/bin/python -m unittest discover -s technology/scan3d_fusion/reconstruction -p 'test_*floor*.py' -v
env -u PYTHONPATH /home/dvt/.venvs/deltar/bin/python technology/scan3d_fusion/reconstruction/complete_floor.py datasets/processed/scan3d_fusion/planar-floor-20261112
technology/scan3d_fusion/viewer/run_clay_rviz.sh datasets/processed/scan3d_fusion/planar-floor-20261112/final_mesh.stl --floor-completion
```

Ảnh: `datasets/processed/scan3d_fusion/planar-floor-20261112/floor_completion/result.png`.

## Giới hạn

Đây là mô hình hình học sàn liên tục cùng cao độ. Khoảng 64,3% diện tích hiển
thị là suy diễn, bao gồm dưới đồ vật; không coi đó là phép đo mới hoặc vùng
free-space. Hố chưa nhìn thấy/ít khung có thể bị phủ; biên hẹp dưới khả năng
khép có thể bị nối nhầm. Chưa hỗ trợ nhiều tầng hoặc sàn biến thiên cao độ.

Không sửa pose drift, mesh tường/vật thể còn lỗi DELTAR và sai số tracking;
khe hở ở rìa tiếp giáp tường có thể còn. Áp dụng sau finalize, chưa thay mesh
realtime, chưa chạy thêm lượt robot và chưa kiểm chứng trên phần cứng thật.
