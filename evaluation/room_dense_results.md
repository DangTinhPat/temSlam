# Toàn phòng: tăng góc RGB–ToF fusion — 2026-11-12

Nguồn: `datasets/processed/scan3d_fusion/sofa-gap-dense-20261112`.
Kết quả: `datasets/processed/scan3d_fusion/room-dense-20261112`.

Giữ nguyên 912 frame/pose/depth/prediction đã có. Bổ sung 2.286 khung từ các
lượt lia đầu **đã ghi**, khi wheel odometry báo vận tốc <3,5 cm/s và đầu đang
lia chiều dương 0,02–0,5 rad/s. Lấy góc mới cách ít nhất 8° hoặc vị trí cách
12 cm; bỏ mẫu cách frame đã có dưới 0,3 giây và cách anchor quá 12 giây.
Tổng 3.198 khung, chạy DELTAR GPU rồi ToF support và TSDF; voxel 4 cm,
min_views=3 giữ nguyên. Thời gian lượt replay 659 giây trên PC này.

## Kết quả mesh đo được

| Chỉ số toàn vùng đo | Trước | Sau |
|---|---:|---:|
| Diện tích XY có mesh gần sàn | 32.139 m² | 35.548 m² |
| Tỷ lệ ô XY có mesh gần sàn | 74.96% | 82.91% |
| RMSE cao độ tam giác gần sàn | 2.30 cm | 1.84 cm |

Bốn vùng xung quanh **loại phần khe sofa–bàn đã cải thiện** khỏi mẫu đo:

| Vùng theo tọa độ map | Trước | Sau | Diện tích hỗ trợ tăng |
|---|---:|---:|---:|
| tây-nam | 57.6% | 67.4% | 0.750 m² |
| đông-nam | 86.8% | 91.4% | 0.374 m² |
| tây-bắc | 60.0% | 71.9% | 1.373 m² |
| đông-bắc | 86.4% | 93.9% | 0.920 m² |

Chia tại X=-2,15 m và Y=-2,3 m; bỏ vùng khe [-3,-3,3] tới [-1,3,-1,3] m.
Toàn vùng đo XY [-5,55,-4,8] tới [1,4,1,35] m, raster 2 cm. Mesh gần sàn được
chọn theo tâm |Z|<8 cm và |normal.Z|>0,7. **Tỷ lệ này không phải độ phủ sàn CAD**:
vẫn tính vùng đồ vật che khuất và có thể gồm phần ngoài biên phòng do khung
map ước lượng lệch hướng. RMSE chỉ trên tam giác vượt bộ chọn này, không phải
sai số toàn cảnh. Lỗ dưới vật thể và dải thiếu quan sát sát biên vẫn còn.

Ảnh đo: [room_dense_results.png](room_dense_results.png).
Ảnh 3D: [result.png](../datasets/processed/scan3d_fusion/room-dense-20261112/result.png).
Số liệu: [room_dense_results.json](room_dense_results.json).

## Kiểm tra nguồn dữ liệu

- 912 frame cũ: dictionary pose giữ nguyên; inputs/prediction/keyframe symlink
  về đúng file gốc. Tất cả 2.286 H5 bổ sung có depth placeholder toàn zero.
- 2.286 keyframe bổ sung mang nguồn `deltar_tof_fusion`, có ToF 64 vùng,
  không có `floor_plane`. Tên file duy nhất, timestamp tăng chặt.
- Sai lệch thời gian sensor/odometry tối đa 5 ms, dưới gate 20 ms.
- Pose bổ sung lấy từ anchor map **ước lượng** và delta wheel/IMU + encoder cổ.
  Simulator world pose chỉ để tái tạo RGB/ToF, không vào camera_to_world của map.
- Không dùng depth RGB-D chuẩn, CAD floor, ép phẳng hoặc completion trong dựng map.

## Cấu hình cho lần quét mới

Cả 66 điểm dừng giữ đầu trong 8° so với ảnh fusion mà SLAM đã xử lý;
feedback cũ quá 5 giây không dùng. Quá 180 giây không tiến triển góc ít nhất
1° thì dừng rõ lý do. Không dùng thời hạn tổng lượt lia ở các điểm quét dày.
Giới hạn 4.096 keyframe, giữ nguyên tọa độ/tốc độ tuyến, lưới và kiểm tra va chạm.
Sinh lại tuyến không có đoạn bị chặn; so với HEAD mọi trường khác ngoài
scan_step_deg đều giống hệt.

Point cloud hiển thị giữ đúng điểm và thứ tự của phép voxel 3 cm cũ, đổi
khóa sort sang số nguyên 1D. Phép thử 7,42 triệu điểm lặp từ mẫu tương đương
3.040 keyframe: riêng gom trùng giảm 7,27 → 1,22 giây, không phải FPS toàn luồng.
Depth TSDF đầy đủ giữ nguyên. 48 unit tests đã qua: reconstruction 22, IPC 7,
route 14, replay/evaluation 5. py_compile và git diff --check qua.

**Chưa chạy lại toàn tuyến live/tracking/loop closure** với cấu hình dày này.
Replay dùng anchor pose cố định và odometry truyền tiếp, nên không đánh giá
được tracking có cải thiện hay không. Nhiều keyframe tốn RAM/đĩa và thời gian;
đây chưa phải cam kết chạy ×5 wall-clock. Không thay độ nhiễu ToF trong lượt này.

## Tái tạo

```bash
env -u PYTHONPATH MUJOCO_GL=egl OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 /home/dvt/.venvs/deltar/bin/python evaluation/replay_sofa_gap.py datasets/raw/tof-20261112-094256-483079 datasets/processed/scan3d_fusion/sofa-gap-dense-20261112 datasets/processed/scan3d_fusion/room-dense-MOI --room
env -u PYTHONPATH /home/dvt/.venvs/deltar/bin/python evaluation/evaluate_sofa_gap.py datasets/processed/scan3d_fusion/sofa-gap-dense-20261112 datasets/processed/scan3d_fusion/room-dense-MOI --room --output evaluation/room_dense_results.json
technology/scan3d_fusion/viewer/run_clay_rviz.sh datasets/processed/scan3d_fusion/room-dense-MOI/final_mesh.stl
```

Phải giữ thư mục nguồn vì dữ liệu cũ được liên kết, không chép đôi.
