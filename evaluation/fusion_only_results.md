# Bản đồ chỉ từ RGB + ToF fusion — 2026-11-12

## Sửa nguồn hình học

Yêu cầu: được lý tưởng hóa phép đo ToF, nhưng hình học map phải đến duy nhất
từ depth của bộ RGB + ToF fusion. Sàn kín của thí nghiệm `planar-floor-20261112`
không đạt yêu cầu này: phần lớn diện tích được bù bằng giả thuyết mặt phẳng.

Đã bỏ hai bước khỏi luồng chính:

1. `FloorSupport.apply`: từng nắn depth về mặt phẳng theo IMU/chiều cao robot.
2. `complete_floor.complete`: từng lấp vùng thiếu bằng sàn suy diễn sau TSDF.

Luồng mới: RGB + histogram ToF → DELTAR → căn bias/kiểm tra ToF → RGB-D →
TSDF/mesh. Pose vẫn lấy từ tracking và IMU/encoder; không dùng pose để ép depth
phải nằm trên sàn. TSDF/lọc mesh vẫn xử lý bề mặt từ các quan sát fusion; không
đưa CAD hoặc mặt phẳng bổ sung vào map. Vùng thiếu dữ liệu giữ là chưa biết.

Tracking và `build_surface` yêu cầu `depth_source=deltar_tof_fusion`, đồng thời
từ chối `floor_plane` hữu hạn của phiên đã ép sàn. Keyframe cũ không được chỉ
đổi nhãn; cần chạy lại fusion từ đầu. `finalize`, mesh realtime, ảnh headless
và `mesh.json` đều dùng mesh fusion, không trỏ tới `floor_completion/completed.stl`.
Hai tiện ích ép/lấp sàn cũ giữ làm hồ sơ thí nghiệm, không tự được gọi nữa.

## Phát lại dữ liệu

Nguồn snapshot: `datasets/raw/tof-20261112-094256-483079`.
Đầu ra: `datasets/processed/scan3d_fusion/fusion-only-20261112`.

`evaluation/replay_fusion_only.py` tái tạo RGB/ToF của 483 keyframe đồng bộ,
chạy checkpoint DELTAR NYU trên CUDA và kiểm tra ToF, sau đó dựng lại TSDF.
Snapshot simulator chỉ phục vụ tái tạo **đầu vào cảm biến**; pose map giữ nguyên
pose **ước lượng** ở `surface_sequence.json`. Không dùng depth RGB-D chuẩn.
H5 `depth` toàn zero để đáp ứng định dạng loader; model chỉ nhận RGB và ToF.

Giữ `inputs/` (RGB/ToF), `prediction/` (depth DELTAR), `keyframes/` (depth sau
kiểm tra ToF), `live_sequence.json`, `source.json` và mesh để đối chiếu nguồn.
ToF giữ cấu hình noise sigma = 0 hiện có; không lý tưởng hóa output fusion.

```bash
env -u PYTHONPATH MUJOCO_GL=egl OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 /home/dvt/.venvs/deltar/bin/python evaluation/replay_fusion_only.py datasets/raw/tof-20261112-094256-483079 datasets/processed/scan3d_fusion/fusion-only-MOI
technology/scan3d_fusion/viewer/run_clay_rviz.sh datasets/processed/scan3d_fusion/fusion-only-20261112/final_mesh.stl
```

## Kiểm tra

12 kiểm tra đã qua: IPC/finalize và ràng buộc nguồn (5), ToF association/occlusion
(3), calibration/ToF alignment/TSDF (4). Kiểm tra riêng xác nhận toàn bộ 483 H5
có depth placeholder bằng zero. Số liệu đối chiếu dữ liệu phát lại được lưu ở
`fusion_only_results.json`; nguồn và thời gian chạy ở `source.json` trong phiên.

## Giới hạn

Đây là phát lại fusion với pose ước lượng cố định, chưa chạy lại toàn bộ tracking
và controller. Không thể dùng kết quả để tuyên bố sàn đã hết lỗ hoặc chính xác
toàn phòng: depth pretrained, vùng không được ToF hỗ trợ, sai số pose và vùng
khuất vẫn giới hạn mesh. Muốn cải thiện phải cải thiện fusion, đồng bộ/pose và
quan sát đa góc; không dùng mặt phẳng giả để che thiếu dữ liệu. Chưa kiểm chứng
trên phần cứng thật.
