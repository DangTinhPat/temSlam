# MuJoCo + bản đồ RGB–ToF cập nhật trực tiếp trong RViz

Từ root repo, chạy:

```bash
./run_vision.sh
```

To fast-forward the simulator by a chosen factor, pass it in the environment,
for example `RB1_SIM_SPEED_FACTOR=15 ./run_vision.sh`. Physics time, controller
steps, and simulated sensor timestamps advance together. GPU workers still run
at hardware speed, so the latest-frame live buffer may skip RGB-D frames at high
factors; omit the variable for normal wall-clock real time.

Gói encoder/IMU/ToF được truyền đủ qua pipe có backpressure. Nếu mapper chậm,
tốc độ thực có thể thấp hơn hệ số yêu cầu; không bỏ gia số odometry để giữ ×5.

Một lệnh mở bảng điều khiển hiện có, MuJoCo với robot/nhà và RViz với bản đồ scan.
Để quét riêng phòng khách có sofa ở tốc độ ×5, dùng `technology/scan3d_fusion/live/run_living_room.sh`; profile này đóng hai lối thông bằng tường va chạm và giới hạn tuyến vào phòng sofa. Nhà đầy đủ vẫn ở scene gốc.
Bấm **Đứng dậy (từ nằm)**, chờ robot cân bằng và trạng thái **Đang vẽ bản đồ**.
Dùng các nút điều hướng hoặc phím mũi tên khi cửa sổ điều khiển đang có focus.
Tốc độ mặc định của profile này là 0,10 m/s. Lệnh di chuyển chỉ được nhận sau khi
tracking có ít nhất 3 keyframe hợp lệ để tránh di chuyển trong lúc GPU đang khởi tạo. Nhả phím để dừng; Esc dừng khẩn cấp.

- MuJoCo hiển thị robot và thế giới vật lý, RViz hiển thị scan ước lượng.
- Point cloud mặc định tắt (có thể bật trong Displays để chẩn đoán); đường đi cập nhật khi có keyframe mới (dịch 5 cm hoặc xoay khoảng 4,6°).
  Tracking vẫn nhận các frame RGB-D được xử lý giữa keyframe.
- Lớp **Bề mặt đất sét (live mesh)** bật sẵn, kiểm tra bản đồ mới mỗi 5 giây.
  Bề mặt đầu tiên yêu cầu ít nhất 3 quan sát hỗ trợ; thời gian xử lý
  và làm nóng GPU có thể khiến lần xuất đầu chậm hơn. Vùng chưa đo vẫn để trống.
- Bản đồ hybrid gom keyframe thành lô 25 khung và bỏ qua cờ PGO để tránh làm
  chậm tracker; chế độ `cuvslam` mới refresh theo pose-graph. Mesh cuối luôn dùng
  toàn bộ lịch sử keyframe đã giữ.
- Dòng trạng thái dưới bảng điều khiển và trong RViz báo chờ cân bằng, nạp GPU,
  tracking, số liệu hoặc lỗi. Lần nạp/làm nóng GPU đầu tiên mất vài giây.
- **Stop SIM** dừng toàn bộ worker/viewer thuộc phiên, giữ dữ liệu đã ghi.
  **Start SIM** mở phiên mới. Đóng bảng điều khiển cũng dừng các tiến trình đó.
- Hybrid kiểm tra delta local cuVSLAM với encoder/IMU rồi chỉ nhận pose hiện tại qua
  `BodyLocalization`; pose-graph không được áp lên lịch sử keyframe. `--pose-policy
  cuvslam` chọn pose đồ thị trực tiếp và dừng khi local tracking bất đồng. Cả hai
  chế độ vẫn cần kiểm tra sai số quay lại vị trí cũ trước khi chấp nhận map. Ảnh
  depth rỗng bị bỏ qua; nếu SDK mất tracking (`None`) hoặc odometry không hợp lệ,
  dừng ghép map. Dùng Stop SIM rồi Start SIM để tạo map mới; không tự nối map khác gốc.

## Dữ liệu và kiến trúc

Mỗi lần Start SIM tạo thư mục mới `datasets/raw/tof-YYYYMMDD-HHMMSS-.../`:

- `sensors.log`, `odometry.jsonl`: log encoder/IMU/ToF và local odometry.
- `live_capture.jsonl`: các snapshot capture theo timestamp để ghép với odometry
  khi mapper bị trễ; `live_capture.json` và `rgbd.npz` vẫn là buffer mới nhất.
  Timestamp odometry và ảnh đã ghép phải lệch không quá 20 ms.
- `keyframes/`, `live_sequence.json`: depth và camera pose đã ước lượng/tối ưu.
- `cloud.npz`: point cloud đã gom voxel 3 cm và quỹ đạo.
- `map_refresh_diagnostics.jsonl`: thời gian dựng từng bản đồ live và lý do cập nhật.
- `mesh-*.stl`, `mesh.json`: mesh mới nhất (giữ 3 revision).
- `cuvslam_map/`: database chỉ được lưu khi tracker thoát sạch và không có lần
  bác bỏ chuyển động visual; không dùng database từ trajectory đã bị chặn.
- `pose_quality.json`, `tracking_diagnostics.jsonl`: thống kê visual bị loại,
  depth rỗng, chính sách pose và trạng thái lưu database. Hybrid chưa chứng minh loop closure.

RViz giữ cloud và đường đã đi cho viewer mở muộn. Quỹ đạo dùng topic riêng
`/scan3d/trajectory` với transient-local để lịch sử trạng thái không đẩy mất nó;
`/scan3d/info` mang trạng thái, `/scan3d/cloud` mang cloud đã lưu.
- `infer.log`, `slam.log`, `mesh.log`, `rviz.log`, `*_status.json`: chẩn đoán.
- `route_state.json`: điểm tuyến, trạng thái controller và pose gần nhất khi quét
  tự động; dùng để đối chiếu khi robot đứng lại. File được thay thế, không phải lịch sử.

Tuyến tự động giữ quyền phát lệnh di chuyển, không bị lệnh tay lưu từ trước ghi đè.
Khi controller tạm chưa sẵn sàng, tuyến giữ nguyên điểm và chờ tối đa 30 giây mô
phỏng; lỗi cân bằng vẫn dừng ngay. Sau khi dừng, nút **Tiếp tục quét** nối lại trong
cùng phiên ứng dụng nếu robot còn cách vị trí dừng không quá 0,3 m và controller
đã sẵn sàng. Khởi động lại ứng dụng không khôi phục tuyến đang dở.

Tuyến toàn nhà giới hạn tốc độ 0,05 m/s, giảm thêm gần điểm dừng và khi lệch
hướng. Các điểm
nối không cần quét không phát STOP. Lượt lia ToF thông thường giữ 20°/s. Tuyến dừng có lý
do khi 12 giây mô phỏng không tiến triển hoặc inference/SLAM báo lỗi. Sau 3
keyframe khởi tạo, tracker chỉ lưu thêm keyframe khi camera dịch ít nhất 5 cm
hoặc xoay ít nhất khoảng 4,6°; đứng yên lâu không tự làm đầy giới hạn keyframe.

Các điểm quét trong nhà và toàn bộ 66 điểm lia trong phòng dùng
`scan_step_deg: 8`: đầu không được đi trước quá 8° so với góc encoder của ảnh
RGB-D mà worker SLAM đã xử lý. Dùng
`sensor_time` và `head_yaw_rad` trong `slam_status.json`, thay vì lấy 15 Hz ToF
làm giả định về tốc độ fusion thực tế. Đầu lia 20°/s khi quét và 45°/s khi vào/ra góc quét. Chờ dữ liệu mới trước khi lia tiếp;
feedback quá 5 giây wall-clock không còn được dùng. Nếu 180 giây wall-clock
không có tiến triển góc fusion (ít nhất 1°), tuyến dừng có thông báo; không dùng
thời hạn tổng lượt lia vì xử lý map lớn sẽ chậm hơn. Không giảm
min_views; giới hạn 8192 keyframe để đủ cho các lượt lia dày toàn nhà.
Không đổi các điểm đi và va chạm.
Headless cộng dự phòng thời gian cho các điểm quét dày; lượt này sẽ lâu hơn.
Xem [kiểm tra khe sofa–bàn](../../../evaluation/sofa_gap_results.md) và
[so sánh bốn vùng toàn phòng](../../../evaluation/room_dense_results.md).

Gom point cloud vẫn giữ cùng điểm đầu tiên trong mỗi voxel 3 cm, nhưng dùng
khóa số nguyên 1D để bỏ trùng thay vì sort từng hàng XYZ. Phép thử 7,42 triệu
điểm (lặp mẫu tương đương 3.040 keyframe) giảm riêng bước này từ 7,27 xuống
1,22 giây trên PC hiện tại; đây không phải benchmark toàn luồng live. Depth
đầy đủ đưa vào TSDF không đổi.

Native controller chạy riêng ở tốc độ cấu hình hiện có. Snapshot CAPTURE đồng bộ
TOFMAP chỉ phục vụ renderer cảm biến; pose thật không cấp vào SLAM. Worker Python
3.8 giữ DELTAR trên GPU và render RGB/area-ToF. Worker Python 3.12 chạy cuVSLAM,
external wheel/IMU correction và dựng cloud; worker thứ ba chạy TSDF GPU.
ROS chỉ nằm ở bridge hiển thị, domain 44. Core không import ROS.
Mapper ghi odometry mỗi frame nhưng chỉ xuất ảnh occupancy/PLY tối đa 4 Hz để
giảm nghẽn khi simulator chạy nhanh; nút quét chờ ít nhất 3 keyframe cuVSLAM.

Quét cả nhà không mở cửa sổ, rồi xuất mesh từ fusion:

```bash
python technology/scan3d_fusion/live/headless.py \
  --route technology/scan3d_fusion/mapping/native/fixed_scan_route.json \
  --speed-factor 1 --pose-policy hybrid
```

Chạy ở tốc độ mô phỏng ×1 để inference theo kịp robot; các lượt ×5 trước đây
để khoảng cách ảnh lớn và làm giảm độ tin cậy visual. Hybrid dùng encoder/IMU
làm quỹ đạo nền. Delta local cuVSLAM phải khớp trong 10 cm/0,15 rad, sau đó
`BodyLocalization` còn chặn hiệu chỉnh tuyệt đối vượt 0,5 m/0,35 rad. Frame bị
loại tiếp tục theo odometry và hiệu chỉnh cuối đã chấp nhận. Pose cuVSLAM hiện
tại chỉ đi qua gate này; không áp pose PGO lên toàn bộ keyframe lịch sử. Cloud
dựng lại mỗi 25 keyframe, không theo cờ PGO có thể tồn tại liên tục. Chưa xác
nhận loop closure toàn nhà; drift odometry vẫn có thể tăng trên tuyến dài. Chế
độ `cuvslam` xuất pose tối ưu toàn cục nên có rủi ro méo khi tracking/PGO bất
thường. Mount camera lấy từ kinematics đầu và thân cùng timestamp; pose mô phỏng
chỉ dùng để đánh giá sau khi ghi map.

## Giới hạn thực tế

Đây là xử lý trực tiếp với buffer mới nhất, không đảm bảo đủ 15 FPS. Tốc độ phụ
thuộc GPU, cảnh, DELTAR và mesh; frame cũ có thể bị bỏ để tránh tăng độ trễ.
Không dùng tăng tốc mô phỏng để rút ngắn lượt quét SLAM. Giới hạn 8192 keyframe/phiên; đến giới hạn hệ thống
ngừng ghép và yêu cầu phiên mới. TSDF là volume dày, voxel tối thiểu 4 cm, tự tăng
khi không gian lớn. Với map lớn, dựng lại toàn mesh sẽ chậm hơn và phải chuyển
sang volume phân mảnh nếu cần mở rộng.

Chất lượng depth/texture và odometry vẫn giới hạn hình học. Hệ thống chưa được
xác nhận cho navigation tự động. Cấu hình này dùng scene ngôi nhà chính thức,
camera 640×480 và các môi trường Python/CUDA đã cài trên PC hiện tại.

### Nguồn hình học: chỉ RGB + ToF fusion

Luồng cập nhật map: RGB + ToF 8×8 → DELTAR → căn bias/kiểm tra phù hợp ToF
(`tof_support`) → RGB-D → TSDF/mesh. ToF có thể không nhiễu theo cấu hình thử;
RGB-D không được lấy từ depth chuẩn MuJoCo. H5 đầu vào DELTAR ghi depth toàn
zero để thỏa định dạng loader, không cung cấp depth chuẩn cho model.

IMU/encoder và động học chân chỉ phục vụ ước lượng pose; không thay đổi depth
để ép sàn về Z=0. Đã bỏ `FloorSupport.apply` khỏi inference và bỏ
`complete_floor.complete` khỏi finalize. Không tự dựng thêm mặt phẳng, phủ dưới
đồ vật hoặc vá khoảng chưa được fusion hỗ trợ. TSDF vẫn hợp nhất/lọc bề mặt từ
quan sát; vùng thiếu depth vẫn là chưa biết.

Mỗi RGB-D/keyframe mới lưu `depth_source=deltar_tof_fusion`, `tof_hist`,
`tof_rect`, `tof_valid`. Tracking và TSDF từ chối dữ liệu không có nguồn này
hoặc có `floor_plane` hữu hạn từ bước ép sàn cũ. Không chỉ đổi nhãn phiên cũ để
vượt kiểm tra: depth đã bị sửa không thể đảo ngược chính xác từ mesh; cần chạy
lại fusion từ RGB/ToF. Không trộn gốc pose giữa các phiên.

Depth fusion được kiểm tra bằng phép chiếu 3D về đúng ô ToF 8×8, giới hạn sai
khác tối đa 15 cm (dung sai tối thiểu 8 cm), và không cho ô xa thắng một vùng đo
gần chồng lên ảnh RGB. Giá trị không phù hợp bị bỏ, không bị kẹp về hình học
CAD. Sai số pose và occlusion do camera/ToF lệch vị trí vẫn có thể đặt điểm sau
tường; bộ lọc này chưa bảo đảm loại hết.

Với vùng depth liên tục nhìn xiên, bộ lọc kiểm tra thêm hình dạng inverse-depth
trong đúng ô ToF. Chỉ giữ thêm pixel sẵn có khi có đủ hỗ trợ 2D, sai số mô hình
cục bộ nhỏ và mean/std của cả vùng khớp ToF. Đây chỉ là kiểm tra tính phù hợp:
không thay depth bằng mặt phẳng, không tạo depth ở pixel zero và không dùng
IMU/chiều cao/sàn CAD. Những vùng có mép đứt hoặc phân bố không phù hợp vẫn theo
giới hạn từng pixel cũ. Nhiễu ToF bằng zero không đồng nghĩa std của vùng bằng
zero: sàn nhìn xiên vẫn có nhiều khoảng cách trong một vùng 8×8.

Khi xuất mesh, mỗi ô Marching Cubes phải có cả 8 đỉnh đạt số quan sát tối thiểu;
không còn co toàn vùng bằng lân cận 27 đỉnh. Vùng thiếu số đo vẫn không được
đóng thành mặt giả. Thay đổi dùng chung cho live, finalize và offline. Xem
[đối chiếu sàn](../../../evaluation/floor_fusion_improvement_results.md).

Ở góc nhìn xiên, Gaussian có thể xóa lớp TSDF âm mỏng dù có số đo hợp lệ.
Core giữ lại TSDF thô ở ô mất bề mặt khi cả 8 đỉnh có ít nhất hai lần số quan
sát tối thiểu (live: 6), mặt gần nằm ngang theo gradient TSDF (|normal.Z| ≥0,9 trong map
Z-up) và không có mặt smoothed tại ô đó/ô chung mặt kề nó. Không đổi depth,
pose hoặc ép cao độ sàn; vùng thiếu quan sát vẫn mở. Xem
[kiểm tra các lỗ sát tường phải](../../../evaluation/remaining_holes_results.md).

### Quét headless và lấy kết quả

Đóng phiên live rồi chạy từ root:

```bash
/home/dvt/micromamba/envs/wheel-leg-ros/bin/python technology/scan3d_fusion/live/headless.py
```

Lệnh chạy phòng khách ở ×5, tự đứng dậy, hạ về chiều cao giữa và đi hết tuyến.
Không mở MuJoCo viewer hoặc RViz. GPU vẫn render cảm biến qua EGL; mesh chỉ dựng
một lần sau khi kết thúc để giảm tải lúc tracking. ToF lấy mức nhiễu trong
`config/tof.yaml`; IMU vẫn dùng MPU-6500 unit 648.

Thư mục phiên được in ra dưới dòng `SESSION`. Sau khi hoàn tất có
`headless_result.json` (trạng thái kết thúc tuyến), `final_mesh.stl`,
`final_mesh.ply`, `final_mesh.json`, ảnh ba góc `result.png` và `mesh.json`.
Ảnh và RViz dùng `final_mesh.stl`, chỉ dựng từ depth fusion; không tự xuất sàn
suy diễn.
Nếu đứng dậy/tracking/tuyến bị lỗi, chương trình trả lỗi và giữ dữ liệu dở dang;
không báo lượt quét đó là hoàn tất.

Finalize dựng lại TSDF theo pose đã lưu cho từng keyframe; không áp surface matching
tuần tự vào pose trước khi xuất mesh. Các phiên hybrid ghi `local_visual_accepted`
và `visual_accepted` cho từng keyframe; `tracking_diagnostics.jsonl` ghi timestamp,
khoảng cách thời gian và lý do loại visual. `pose_quality.json` ghi số lần loại,
khoảng cách thời gian tracker và trạng thái lưu database. Hybrid không lấy pose
graph để cập nhật lại lịch sử; pose cuVSLAM hiện tại vẫn qua gate local và
`BodyLocalization`. Chế độ này chưa chứng minh loop closure; sai số odometry vẫn
có thể tăng theo quãng đường.

### Phát lại fusion để thay dữ liệu đã bị ép sàn

```bash
env -u PYTHONPATH MUJOCO_GL=egl OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 /home/dvt/.venvs/deltar/bin/python evaluation/replay_fusion_only.py datasets/raw/PHIEN_CU datasets/processed/scan3d_fusion/PHIEN_MOI
technology/scan3d_fusion/viewer/run_clay_rviz.sh datasets/processed/scan3d_fusion/PHIEN_MOI/final_mesh.stl
```

Phát lại RGB và ToF từ snapshot cảm biến đồng bộ trong simulator, chạy lại
DELTAR trên GPU rồi dựng TSDF. Giữ nguyên pose **ước lượng** đã lưu để so sánh
nguồn hình học, không đưa pose thật hoặc depth RGB-D chuẩn vào map. Đây chưa
phải một lượt SLAM/tracking mới. Giữ H5 đầu vào, prediction, keyframe và
`source.json` để kiểm tra nguồn dữ liệu. Thư mục đầu ra phải chưa tồn tại.
Nếu nguồn là phiên đã phát lại có `inputs/` và `prediction/`, script dùng lại
đúng RGB/ToF và prediction để so bộ lọc, không render/chạy model lại; giữ nguyên
pose và bounds TSDF. Các thư mục liên kết phải còn tồn tại khi sử dụng kết quả.

Các bản `floor_completion/` cũ chỉ giữ lại để đối chiếu thí nghiệm. Không dùng kết quả
sàn suy diễn làm đầu ra SLAM chính thức. Xem
[fusion_only_results.md](../../../evaluation/fusion_only_results.md).

### Tuyến cố định phòng khách

Tuyến phòng khách hiện đi một vòng chu vi trước, qua hành lang phía sau sofa
tại X=-5,4 m, rồi đi các dải ziczac cách 0,75 m và dải phía bắc cầu thang.
Tuyến dài 96,3 m, 184 waypoint, 66 điểm lia đầu 360°, kết thúc tại gốc. Đoạn
sát tường giới hạn 0,12 m/s, đoạn trong phòng giữ 0,20 m/s. Góc cổ luôn về thẳng
trước khi đi tiếp. Tuyến ziczac trước được giữ ở
`mapping/native/living_room_interior_route.json`; tuyến chu vi đời đầu ở
`mapping/native/living_room_perimeter_route.json`.
Sinh lại tuyến khi đổi đồ vật bằng:

```bash
/home/dvt/micromamba/envs/wheel-leg-ros/bin/python technology/scan3d_fusion/mapping/plan_living_room_scan.py
```

Bộ sinh tuyến dùng cảnh vật lý đã biết cho bài thử đường cố định, kiểm tra bằng
probe bán kính 0,25 m, lưới đường 0,15 m và kiểm tra đoạn nối mỗi 5 cm. Lưới 0,25 m
trước đây bỏ qua hành lang hẹp phía tây sofa. Nó không đưa hình học scene
vào thuật toán dựng bản đồ. Các mục tiêu bị đồ vật chiếm chỗ được bỏ qua từ đầu;
các đoạn nối đi vòng và không quét lại ở vị trí cách điểm đã quét dưới 45 cm.

Kiểm tra controller trên tuyến/cảnh này (không mở cửa sổ):

```bash
python technology/scan3d_fusion/mapping/check_route_physics.py evaluation/living_room_perimeter_physics.json --route technology/scan3d_fusion/mapping/native/living_room_scan_route.json
```

Lượt headless 2026-11-11 hoàn tất 137/137 waypoint ở ×5 trong 470 giây, lưu
450 keyframe (tuyến cũ: 313). Ô có ít nhất hai khung đo sàn tăng từ 32,88 lên
38,83 m², nhưng sau mặt nạ vật cản/hố, sàn được chấp nhận giảm từ 18,49 xuống
13,86 m². Đây là diện tích lưới theo pose ước lượng, không phải tỷ lệ phủ ground
truth. Tăng mật độ tuyến chưa giải quyết được xung đột bằng chứng sàn/vật cản;
xem `evaluation/living_room_route_comparison.json` ở gốc repo để đối chiếu.
