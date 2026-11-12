# Lỗ còn lại sát tường phải — 2026-11-12

Ảnh người dùng: `Screenshot from 2026-11-12 12-31-48.png`.
Nguồn: `room-dense-20261112`; kết quả chính thức: `room-floor-preserved-20261112`
trong `datasets/processed/scan3d_fusion/`.

## Nguyên nhân đã kiểm chứng

Các điểm gần tâm lỗ X/Y [1,53;0,94], [1,70;-0,05], [1,35;-2,95],
[1,45;-1,50] có lần lượt 13, 25, 7, 9 quan sát depth fusion cách optical depth
của điểm thử Z=0 dưới 10 cm. Không phải tất cả đều thiếu dữ liệu. Phép Gaussian
trên tổng TSDF và tổng weight có thể làm mất lớp SDF âm mỏng ở góc nhìn xiên,
vì voxel phía không gian trống có nhiều lượt quan sát hơn voxel phía sau bề mặt.
Khi toàn bộ ô mất đổi dấu, Marching Cubes không còn dựng được bề mặt.

Đã tái hiện bằng unit test trường TSDF phẳng với 6 quan sát ở lớp âm mỏng và
300 quan sát ở phía dương: mã trước báo không có bề mặt, mã mới giữ mặt tại
cao độ đúng dưới 5 mm. Đây là fixture kiểm tra; không phải dữ liệu đưa vào map.

Khung đo toàn phòng trước chỉ tới X=1,4 m; nhiều lỗ trong ảnh nằm tới X=1,9 m.
Mốc 82,9% trước không đại diện đủ cho dải lỗ này, cũng không phải độ phủ sàn CAD.

## Sửa trong core dùng chung

Giữ nguyên TSDF tích phân, Gaussian hiện có và Taubin mesh. Khi một ô có đổi
dấu trong TSDF thô nhưng mất mặt sau Gaussian, chỉ lấy lại giá trị TSDF thô ở
8 đỉnh của ô khi:

- Mọi đỉnh có ít nhất hai lần min_views (live: 6 quan sát; ngưỡng mesh chung vẫn 3).
- Bề mặt gần nằm ngang trong hệ map Z-up, xét pháp tuyến từ gradient SDF thô: |normal.Z| >=0,9.
- Không có mặt smoothed ở ô đó hoặc ô chung mặt kề nó, để tránh dựng hai lớp sát nhau.

Một trường TSDF liên tục duy nhất đi vào Marching Cubes; không ghép chồng một
tấm sàn khác. Không dùng Z=0 để quyết định chỗ khôi phục, không tạo depth, không
đổi pose, không dùng CAD hoặc depth chuẩn của mô phỏng trong map. Toàn bộ
3.198 frame và depth gốc được liên kết nguyên vẹn từ nguồn. Những vùng chưa
quan sát vẫn phải vượt điều kiện đủ cả 8 đỉnh; không vá qua dải depth bằng zero.

## Kết quả ở đúng các vùng lỗi

Raster XY 2 cm, tam giác gần sàn có |Z tâm|<8 cm và |normal.Z|>0,7.
Các ô đo có thể gồm phần ngoài biên phòng; đây là hỗ trợ mesh trong ROI,
không phải tỷ lệ phủ sàn thật. Bounds từng vùng được ghi trong JSON.

| Vùng từ trên xuống dọc tường phải | Trước | Sau | RMSE cao độ |
|---|---:|---:|---:|
| 1 | 71.6% | 93.5% | 3.33 → 2.78 cm |
| 2 | 67.7% | 81.9% | 4.34 → 3.99 cm |
| 3 | 57.9% | 72.4% | 2.76 → 2.92 cm |
| 4 | 79.7% | 88.4% | 1.63 → 1.38 cm |

RMSE toàn khung đo cũ giữ gần nguyên: 1,8438 → 1,8426 cm. Vùng số 3 tăng nhẹ
RMSE 2,76 → 2,92 cm khi giữ lại phần bề mặt trước đó bị mất. Không tuyên bố
không còn lỗ hoặc toàn bộ hình học đã chính xác.

[Ảnh so sánh](remaining_holes_results.png) · [Số liệu](remaining_holes_results.json) ·
[Ảnh 3D](../datasets/processed/scan3d_fusion/room-floor-preserved-20261112/result.png).

## Kiểm tra phụ bằng oracle — chỉ đánh giá

Render mesh từ đúng camera pose ước lượng của 32 frame gốc rải đều, so với
camera depth chuẩn đã lưu riêng tại `floor-audit-20261112/reference.npz`.
Đây là các frame đã tham gia dựng map, **không phải test set độc lập**.
Không có oracle nào cấp vào bước khôi phục mesh.

Pixel nhìn sàn có mesh trong 4 m tăng 98,926% → 99,534%; pixel sàn có optical
depth mesh cách reference dưới 15 cm tăng 2.527.351 → 2.544.328. RMSE cao độ
render trên phần sàn có mesh 2,337 → 2,333 cm. Proxy pixel trên vật thể/tường
bị render xa hơn reference >10 cm giảm 1.326.339 → 1.325.947. Số pixel mesh
trong 4 m nhưng reference ngoài 4 m tăng 417.155 → 422.180; chỉ số cắt tại
4 m này không tự phân biệt sai số sát ngưỡng với bề mặt sai/che khuất.
Sai số mesh/pose cũ vẫn còn lớn; không thể suy ra map sạch vật cản ảo từ phép này.

Xem [remaining_holes_final_render_check.json](remaining_holes_final_render_check.json).

Đã thử bỏ Gaussian hoàn toàn: độ gồ ghề mesh tăng 1,84 → 2,73 cm, không giữ.
Cũng không giữ bản phục hồi mọi hướng vì làm lộ thêm speckle ở tường.

24 test reconstruction đã qua, gồm dải thiếu dữ liệu vẫn mở, đo quá ít không
được phục hồi và nhiễu dốc ở tường không bị phục hồi bởi nhánh dành cho sàn.
Thay đổi ở `reconstruction/reconstruct.py` áp dụng cho live mesh, finalize và
offline. Chưa chạy lại toàn tuyến live hoặc tracking; kết quả này là dựng lại
mesh từ fusion/pose có sẵn. Mặt trên/đằng sau vật thể chưa được nhìn thấy vẫn
có thể hở. Đầu ra symlink nên cần giữ thư mục nguồn.
