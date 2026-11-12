# Cải thiện sàn từ fusion — 2026-11-12

Nguồn đối chiếu: `datasets/processed/scan3d_fusion/fusion-only-20261112`.
Kết quả: `datasets/processed/scan3d_fusion/floor-improved-20261112`.
Giữ nguyên 483 prediction DELTAR, đầu vào RGB/ToF, pose ước lượng và bounds TSDF.
Không dùng floor correction, floor completion hoặc hình học CAD trong map.

## Thay đổi được giữ

1. `depth_support.sloping_zone_support`: kiểm tra phân bố của vùng liên tục
   thay vì bắt mọi pixel trong vùng nhìn xiên phải gần cùng một mean. Khớp
   inverse-depth affine từ chính depth fusion ở đúng ô ToF, cần >=24 điểm lấy
   mẫu có độ phủ hai chiều >=70%, residual P90 <=2,5 cm. Phân bố lấy mẫu đều
   trên vùng phải có mean cách ToF <=8 cm và std phù hợp trong
   `max(2,5 cm, 35% sigma)`. Chỉ thêm hỗ trợ cho pixel gốc có residual <3 cm và
   còn nằm trong 3 sigma ToF. Không dùng mô hình đó để tạo/thay depth, không
   dùng nhãn sàn hoặc IMU. Với mặt đứt/không phù hợp, giữ bộ lọc cứng cũ.
2. Mask Marching Cubes: trước đây co vùng đủ quan sát bằng khối 3×3×3, làm
   mất bề mặt ở vùng quan sát mỏng. Nay kiểm tra đúng 8 đỉnh của từng ô, mọi
   đỉnh đều phải đạt min_views; không bỏ điều kiện số đo để đóng vùng chưa biết.
   Vị trí mask ở góc chỉ số cao được đối chiếu với
   [mã nguồn scikit-image 0.21](https://github.com/scikit-image/scikit-image/blob/v0.21.0/skimage/measure/_marching_cubes_lewiner_cy.pyx#L901).

Hai thay đổi ở helper dùng chung nên áp dụng vào live, finalize, offline và
replay. Không thêm dependency, không đổi nhiễu ToF, không giảm min_views.

## Kết quả

Đánh giá depth ở 32 khung rải đều, trên pixel thực sự nhìn sàn theo depth chuẩn
và pose chuẩn **chỉ ở evaluator**. Oracle cục bộ tại
`datasets/processed/scan3d_fusion/floor-audit-20261112/reference.npz`; mặt nạ sàn
là điểm chuẩn có |Z| <3 mm. Map không đọc oracle này.

| Chỉ số | Trước | Sau |
|---|---:|---:|
| Pixel sàn hợp lệ giữ lại | 1.873.603 | 1.917.773 |
| Tỷ lệ pixel sàn giữ lại trên 32 khung | 63,43% | 64,92% |
| MAE cao độ depth trên phần sàn giữ lại | 1,888 cm | 1,872 cm |
| RMSE cao độ depth | 2,727 cm | 2,714 cm |
| Pixel bề mặt cao >15 cm bị dự đoán sau >10 cm | 33.341 | 33.341 |
| Tổng diện tích chiếu XY tam giác gần sàn | 29,525 m² | 33,587 m² |
| RMSE cao độ tam giác gần sàn | 2,211 cm | 2,227 cm |

Diện tích tam giác gần sàn tăng **13,76%**, nhưng sai số mesh gần như giữ
nguyên, chưa có cải thiện đáng kể về độ nhẵn. Diện tích này chọn tâm tam giác
|Z|<8 cm, |normal.Z|>0,7 trong hệ map ước lượng, có thể chồng lấn; **không phải
tỷ lệ phủ phòng thật hoặc chứng minh map không có lỗ**. Điểm sau tường chưa
được loại hết, chỉ không tăng trên bộ kiểm tra này. Điều kiện độ cao >15 cm là
nhóm bề mặt phía trên sàn, có thể gồm vật thể, không chỉ riêng tường.

Toàn bộ 483 khung giữ 77.929.603 pixel depth so với 76.867.901 trước.
Thử bộ lọc trên 6 khung: khoảng 98,7 → 124,7 ms/khung trên PC này; chưa gồm
inference/TSDF và không phải benchmark FPS realtime đầy đủ.

Đã thử riêng tích phân point-to-plane từ pháp tuyến depth. Mặc dù diện tích
tăng, MAE mesh gần sàn tăng 1,72 → 3,80 cm; phương án này **đã loại khỏi mã
chạy chính**. Không đánh đổi sàn đầy hơn bằng sai lệch cao độ lớn hơn.

19 kiểm tra đã qua: ToF support (5), geometry/unknown stripe/oblique view (6),
IPC/finalize nguồn fusion (5), surface matching (2), replay cache (1). Kiểm
tra oblique view giữ bề mặt tới >3,3 m, sai số dưới nửa voxel 4 cm; stripe
không có depth vẫn để mở. Replay cache xác nhận không render/chạy model lại
hoặc đổi pose/bounds.

## Tái tạo

```bash
env -u PYTHONPATH MUJOCO_GL=egl OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 /home/dvt/.venvs/deltar/bin/python evaluation/replay_fusion_only.py datasets/processed/scan3d_fusion/fusion-only-20261112 datasets/processed/scan3d_fusion/floor-improved-MOI
env -u PYTHONPATH /home/dvt/.venvs/deltar/bin/python evaluation/evaluate_fusion_floor.py datasets/processed/scan3d_fusion/fusion-only-20261112 datasets/processed/scan3d_fusion/floor-improved-20261112 --oracle datasets/processed/scan3d_fusion/floor-audit-20261112/reference.npz --output evaluation/floor_fusion_improvement_results.json
technology/scan3d_fusion/viewer/run_clay_rviz.sh datasets/processed/scan3d_fusion/floor-improved-20261112/final_mesh.stl
```

Số liệu đầy đủ: [floor_fusion_improvement_results.json](floor_fusion_improvement_results.json).
Ảnh: [floor_fusion_comparison.png](floor_fusion_comparison.png).
Đầu ra liên kết inputs/prediction từ nguồn, cần giữ nguồn khi lưu trữ.

## Còn thiếu

Sàn vẫn có vùng thủng lớn và lượn sóng. Khoảng 22,6% pixel sàn của oracle nằm
ngoài FoV ToF trong phép thử; không thể cứu các vùng đó bằng nới bộ lọc trong
FoV. Phần còn lại chịu sai số pretrained depth, che khuất và pose. Bước kế tiếp
là kiểm tra/hợp nhất depth nhiều góc nhìn với độ tin cậy và bảo vệ occlusion.
Chưa triển khai temporal depth fusion trong thay đổi này; chưa chạy lại robot,
tracking, benchmark live ×5 hoặc kiểm chứng phần cứng.
