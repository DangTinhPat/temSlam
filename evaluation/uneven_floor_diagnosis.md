# Lồi lõm sát tường phải — 2026-11-12

Nguồn đang hiển thị: `room-floor-preserved-20261112`. Không đổi cấu hình nhiễu,
pose, depth hoặc luồng map chính trong đợt phân tích này.

## Tách nguyên nhân

Dùng 32 frame gốc rải đều và oracle depth/camera pose **chỉ ở evaluator**.
Các frame này có trong map; không phải tập test độc lập. Pixel phải có nhãn
sàn theo oracle, depth fusion hợp lệ và reference depth <4 m. Vùng bên phải
được chọn theo reference depth chiếu qua pose map ước lượng: X trong (1,1;2,1),
Y trong (-4,8;1,5). Tổng 198,701 pixel.

| Thành phần | RMS cao độ | P95 sai số tuyệt đối |
|---|---:|---:|
| Depth fusion với pose chuẩn để cô lập lỗi depth | 2.598 cm | 6.354 cm |
| Depth chuẩn với pose ước lượng để cô lập lỗi pose | 0.167 cm | 0.338 cm |
| Depth fusion với pose ước lượng | 2.521 cm | 6.139 cm |

Các thành phần không cộng tuyến tính vì có tương quan. Trong mẫu kiểm tra này,
lỗi depth lớn hơn lỗi pose khoảng 15 lần; không đủ bằng chứng để đổ lỗi chính
cho cổ/encoder/IMU. ToF không thêm nhiễu không làm mất sai số nội suy, pretrained
DELTAR, trường hợp vùng 8×8 trộn sàn/tường và bước căn bias cục bộ.
Bước phục hồi crossing trước chỉ khôi phục bề mặt có số đo; nó cũng giữ lại
độ cong sai vốn có trong số đo, không phải một bộ sửa độ sâu.

Số liệu chi tiết: [uneven_floor_audit.json](uneven_floor_audit.json).

## Thử lọc nhiều góc — đã loại khỏi core

Thử dịch vertex gần nằm ngang theo median point-to-plane residual của các depth
fusion đã đồng bộ/pose ước lượng. Đòi 6 view phù hợp, normal agreement >0,8,
residual <6 cm, giới hạn dịch 3 cm, không thêm/xóa tam giác. Histogram residual
61 bin cách 2 mm. Có self-check plane lệch Z=0, pixel không đo, ít view và bóng
đường kính 7 cm được giữ lại trong fixture.

Lượt thử trên 3.198 frame mất 18,19 giây; 26.857 vertex thay đổi. Một số vùng
sát tường tốt lên, nhưng RMSE mesh gần sàn toàn vùng **tăng 1,843 → 2,302 cm**.
RMSE cao độ render so với oracle tăng **2,333 → 2,764 cm**; pixel sàn có optical
depth trong 15 cm reference giảm 2.544.328 → 2.436.891. Vì vậy không tích hợp
bộ lọc thử vào worker/live; đã bỏ file prototype/test khỏi core, không đổi
bản đang hiển thị. Hai artifact giữ để đối chiếu thí nghiệm:
[multiview_surface_results.json](multiview_surface_results.json),
[multiview_surface_render_check.json](multiview_surface_render_check.json).

## Bản đối chiếu có giả thiết mặt phẳng — chưa áp dụng chính thức

Học theo hướng khai thác mặt phẳng từ dữ liệu của
[PlanarSLAM](https://github.com/yanyan-li/PlanarSLAM), nhưng đây chỉ là demo
regularization mesh, **không phải đã tích hợp PlanarSLAM hoặc plane-factor BA**.

Chọn connected component lớn nhất của tam giác có |normal.Z|>0,85. Fit RANSAC
120 lần từ tối đa 6.000 tâm tam giác, ngưỡng inlier 1,5 cm, ít nhất 50% điểm,
rồi SVD inlier. Không lấy chiều cao robot/CAD/Z=0 để đặt mặt phẳng.
Chỉ dịch vertex thuộc riêng component đó, không dùng chung với mặt vật thể
khác, và cách mặt phẳng dưới 6 cm. Giữ nguyên toàn bộ tam giác và lỗ hở.

Kết quả fit: n=[0.002350171531326639, 0.0025502081554199998, 0.9999939865479878], d=-0.00140003 m;
Z tại XY gốc=1.400 mm, inlier 66.77%.
Đã dịch 25,524 vertex, tối đa 5.98 cm;
**0 tam giác mới**, không đổi depth hoặc pose.

[Bản thử riêng](../datasets/processed/scan3d_fusion/room-estimated-plane-preview-20261112/result.png) ·
[Thông số fit](uneven_floor_plane_preview.json).

Đây là **giả thiết hình học**, không phải nguyên trạng số đo. Gờ/vật thấp bị
fusion trộn vào cùng component sàn có thể bị làm phẳng. Không dùng bản này làm
map chính khi chưa rõ lựa chọn của người dùng về ràng buộc mặt phẳng. Nó không
khắc phục mọi sai số depth, mesh thủng hoặc hình dạng sofa/cầu thang.

Chưa benchmark lại live ×5, tracking, robot hoặc độ an toàn navigation. Oracle
chỉ được đọc ở phân tích/evaluation, không cấp vào mesh của hai lượt thử.
