# Giảm sàn dạng phễu khi lia cổ — 2026-11-12

Đã sửa hai nguồn sai lệch: `TOFMAP` nay dùng pitch đã lấy mốc của controller và
roll lấy mốc khi nằm yên ở home; luồng RGB-D không xoay/dịch cao độ pose camera
theo mặt phẳng fit riêng từng ảnh depth. Pose luôn lấy roll/pitch và chiều cao
từ IMU đã bù mốc cùng động học chân. Depth chỉ được làm phẳng ở vùng sàn đã xác
nhận, theo cùng phương trọng lực/cao độ đó; mất xác nhận không đổi nguồn pose.

Không đưa ground truth vào mapping. Công cụ `../diagnose_head_spin_floor.py`
dùng depth/pose chuẩn MuJoCo chỉ để chọn pixel thực sự thuộc sàn và đo sai số.

## Kiểm chứng

- Build `rb1_sim` thành công.
- 3 kiểm tra floor support, 3 kiểm tra localization đều qua.
- Kiểm tra native với IMU lý tưởng và MPU6500 unit 648 đều qua, mỗi nhánh 300
  mẫu khi robot cân bằng và quay cổ. Với IMU có nhiễu, P95 sai số tuyệt đối
  pitch/roll = 0,238° / 0,149°; ngưỡng kiểm tra là 0,6°.
- Tuyến headless mới hoàn tất 137/137 waypoint, 451 keyframe, ×5, 469,3 giây thực.

| Chỉ số | Trước | Sau |
|---|---:|---:|
| P95 sai số cao độ sàn, lượt quay cổ đầu | 8,04 cm | 4,83 cm |
| Cùng depth nhưng dùng pose chuẩn để chẩn đoán | 5,05 cm | 4,75 cm |
| Sàn quan sát được chấp nhận sau mặt nạ vật cản | 13,86 m² | 17,53 m² |
| Sàn suy ra, lưu riêng | 1,40 m² | 2,43 m² |

Trước: `datasets/raw/tof-20261111-184619-486515`.
Sau: `datasets/raw/tof-20261112-090937-766548`.
Các đường dẫn phiên tính từ gốc repo. Mesh gốc và các lớp floor completion của
cả hai phiên được giữ riêng. Đồ thị: [before_after.png](before_after.png).
Số liệu từng khung: [before.json](before.json), [after.json](after.json).

Lượt quay đầu gồm 14 khung trước và 21 khung sau; lịch lấy keyframe không trùng
nhau. Đây là kiểm chứng hai lượt chạy cùng cấu hình/tuyến, chưa phải thử nghiệm
nhiều lần với từng ảnh đầu vào ghép cặp. Diện tích lưới dùng pose ước lượng,
không phải tỷ lệ phủ ground truth. Không quy toàn bộ chênh lệch diện tích cho IMU.

## Tái kiểm tra

```bash
python technology/scan3d_fusion/mapping/odometry/test_mapping_attitude.py -v
env -u PYTHONPATH MUJOCO_GL=egl /home/dvt/.venvs/deltar/bin/python evaluation/diagnose_head_spin_floor.py datasets/raw/tof-20261112-090937-766548 evaluation/head_spin_floor/after.json
technology/scan3d_fusion/viewer/run_clay_rviz.sh datasets/raw/tof-20261112-090937-766548/final_mesh.stl --floor-completion
```

## Giới hạn

Lấy mốc home và hiệu chỉnh sàn yêu cầu mặt sàn ngang. Không dùng chế độ này
để kết luận về dốc/cầu thang hoặc thay cho hiệu chuẩn IMU phần cứng nhiều tư thế.
Chưa kiểm chứng trên robot thật. Depth DELTAR còn sai số vài centimet; mesh vẫn
có vùng gợn/lỗ. Vùng gò thấp nối trơn với sàn có thể bị làm phẳng như giới hạn
đã ghi của bộ lọc; sàn suy ra không phải bảo đảm khoảng trống cho navigation.
