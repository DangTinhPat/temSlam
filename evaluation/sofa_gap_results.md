# Khe sofa–bàn: bổ sung góc fusion đã ghi

So sánh `floor-improved-20261112` (483 khung) với
`sofa-gap-dense-20261112` (912 khung). Thêm 429 RGB/ToF đầu vào ở các lượt đi
sát khe đã ghi, chạy DELTAR trên GPU. Depth và pose của 483 khung gốc giữ nguyên.
Pose bổ sung = pose anchor đã ước lượng × delta wheel/IMU đồng bộ × mount đầu.
Không lấy pose world/depth RGB-D chuẩn từ mô phỏng vào map, không tạo sàn hoàn thiện.

Trong vùng XY [-3, -3.3] tới [-1.3, -1.3] m, raster 2 cm:

| Chỉ số | Trước | Sau |
|---|---:|---:|
| Diện tích XY có mesh gần sàn | 2,168 m² | 3,271 m² |
| Tỷ lệ vùng XY có mesh gần sàn | 63,75% | 96,20% |
| RMSE cao độ tam giác gần sàn | 2,68 cm | 1,75 cm |

Đây là hỗ trợ mesh đo được trong vùng chọn, không phải tỷ lệ phủ CAD. Tam giác
được chọn bởi |Z tâm| <8 cm, |normal.Z| >0,7; không chứng minh sàn phẳng tuyệt đối.
Các vùng khác trong phòng chưa được bổ sung đủ ảnh trong lượt này.

Ảnh: [sofa_gap_results.png](sofa_gap_results.png).
Số liệu: [sofa_gap_results.json](sofa_gap_results.json).

```bash
env -u PYTHONPATH MUJOCO_GL=egl OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 /home/dvt/.venvs/deltar/bin/python evaluation/replay_sofa_gap.py datasets/raw/tof-20261112-094256-483079 datasets/processed/scan3d_fusion/floor-improved-20261112 datasets/processed/scan3d_fusion/sofa-gap-MOI
```

Giới hạn: replay dùng pose anchor có sẵn và odometry cục bộ, không chạy lại
tracking/global optimization; dữ liệu được symlink nên phải giữ các thư mục nguồn.
Cấu hình live đã mở rộng cách chờ fusion từ 6 điểm quanh khe sang toàn phòng;
kết quả replay không thay thế kiểm tra lại toàn luồng live.
