# Demo navigation trên bản quét 3D — 14/11/2026

![RB chạy trên mesh đã quét](navigation_mesh_3d_run_20261114/navigation_3d.gif)

Đây là mesh **`room-floor-preserved-20261112/final_mesh.stl`** đã có, không phải
phòng minh họa tạo mới. Mô hình CAD RB được hiển thị bằng viewer MuJoCo có sẵn;
pose chạy theo bộ bám đường động học. Không `mj_step`, không chạy controller
cân bằng, không định vị từ cảm biến và không gửi lệnh tới robot/native runner.
GIF phát nhanh khoảng 2×; viewer tương tác phát 1×.

## Kết quả

| Đại lượng | Giá trị |
|---|---:|
| Điểm xuất phát X/Y | 0,7 / −1,6 m |
| Đích X/Y | 0,8 / 0,8 m |
| Chiều dài đường A* | Khoảng 2,46 m |
| Thời gian mô phỏng | 29,35 s |
| Sai số cuối | 2,45 cm |
| Cắt vào vùng cấm đã nới theo grid | Không |
| Vận tốc tiến/quay tối đa | 0,1 m/s / 0,4 rad/s |
| Free / occupied / unknown trước inflation | 10.761 / 5.715 / 9.099 ô |
| Ô đi được sau khoảng đệm 25 cm | 2.655 |

[Quỹ đạo, tham số và SHA256 mesh nguồn](navigation_mesh_3d_run_20261114/result.json)
· [Ảnh tới đích](navigation_mesh_3d_run_20261114/arrived.png)
· [Grid chỉ dùng trình diễn](navigation_mesh_3d_run_20261114/grid.npz).

## Vì sao khác grid depth trước đó?

Grid trước hợp nhất mọi endpoint cao 2,5–40 cm từ 3.198 depth frame: chỉ một
endpoint nhiễu cũng chặn cả ô. Với sai số depth đã đo, các quan sát sàn không
nhất quán có thể trở thành vật cản. Kết quả trước là 20.616 ô occupied, 0 ô đi
được sau inflation; kết quả đó vẫn được giữ nguyên, không đổi ngưỡng để mở đường.

Demo mới chiếu **bề mặt mesh đã tích hợp** thay vì hợp tất cả endpoint thô.
Mesh có khoảng 29,78 m² tam giác sàn đạt ngưỡng cao độ/pháp tuyến, cho phép
chọn một tuyến trên vùng sàn liên thông. Ngưỡng sàn ±2,5 cm và khoảng đệm 25 cm
vẫn giữ nguyên. Đây là thay đổi loại bằng chứng dùng cho trình diễn, chưa phải
sửa xong chất lượng depth/localization.

Không tạo hoặc dời vertex/tam giác của mesh. Lỗ lớn không được lấp. Kiểm tra sàn
lấy mẫu cách 5 mm có thể bỏ sót lỗ nhỏ hơn; mặt mesh có thể thiếu vật mảnh hoặc
vật cản bị mất khi dựng. Vùng trống theo mặt sàn không chứng minh toàn bộ thể
tích robot đã được quan sát. Vì vậy file mang cờ `demo_only`, bị chặn trong
native; không báo lượt chạy này là navigation tự hành bằng cảm biến.

## Kiểm tra và tái hiện

58 test đạt: gồm regression cũ, mặt sàn/giữ lỗ, vách đứng diện tích chiếu bằng 0,
vật thấp 4 cm, overhang 35 cm, mặt dốc, dữ liệu sai, lưu/đọc cờ demo và từ chối
khởi động native. Lượt xuất 3D dùng renderer thật, assert tới đích trong 2,5 cm
và không cắt vùng cấm grid. Chưa đo va chạm vật lý.

```bash
env -u PYTHONPATH MUJOCO_GL=egl /home/dvt/micromamba/envs/wheel-leg-ros/bin/python \
  technology/scan3d_fusion/navigation/mesh_demo.py --headless \
  --output datasets/processed/scan3d_fusion/navigation-3d-check
```

Output phải mới. [Hướng dẫn viewer 3D tương tác](../technology/scan3d_fusion/navigation/README.md#xem-rb-đi-trên-bản-quét-3d).
