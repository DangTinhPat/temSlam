# Kế hoạch vẽ map bằng ToF, encoder và IMU, không dùng ROS

## 1. Kết quả cần bàn giao

RB di chuyển trong cảnh indoor Small House và tích lũy số đo ToF 8×8 thành bản đồ xám nhìn từ trên xuống. Giao diện hiển thị bản đồ, quỹ đạo ước lượng, vị trí/hướng robot, điểm ToF mới nhất và tình trạng dữ liệu. Tường và vật cản đã quan sát được giữ trong map khi đi khỏi góc nhìn; bằng chứng đo mới có thể sửa chúng.

Mốc đầu tiên là mapping theo odometry, tương ứng cách cấu hình ArduBot đang hoạt động. Sau đó mới bổ sung hiệu chỉnh pose bằng ToF nếu dữ liệu cho phép. Vẽ được map không đồng nghĩa đã có SLAM với loop closure.

Điều kiện bắt buộc:

- Runtime, build và giao tiếp mapping không phụ thuộc ROS, DDS, TF, SLAM Toolbox hoặc Nav2.
- Lõi mapping là executable C++17 độc lập trong `technology/scan3d_fusion/mapping/tof_only/`, không liên kết MuJoCo hay thư viện firmware.
- Chỉ nhận số đo ToF, IMU, encoder bánh/khớp, timestamp và calibration.
- Pose và hình học thật của cảnh chỉ thuộc simulator và evaluator; không truyền `VIEW` vào mapper.
- Luồng RGB/RGB-D hiện có tiếp tục phục vụ xem hình, không trở thành đầu vào mapping ToF.
- Mô phỏng và viewer vẫn chạy riêng với vòng điều khiển cân bằng. Mapper chậm/thoát không được chặn controller.

## 2. Phần tham khảo từ ArduBot

| Nguồn trong `prj_reference/ArduBot-ToF-SLAM/` | Ý tưởng áp dụng | Điều chỉnh cho RB |
| --- | --- | --- |
| `raspberry_ws/src/arducam/src/tof_pointcloud.cpp` | Khoảng cách theo tia → XYZ; kiểm tra tầm đo; tạo biểu diễn để mapping | Dùng 64 vùng và ray calibration của RB thay cho 240×180 pixel; không dùng median 5×5 hay giảm số điểm trên lưới 8×8 |
| `pc_ws/src/ardubot_bno_encoder_odometry/ardubot_bno_encoder_odometry/core.py` | Tích phân quãng đường với hướng ở giữa khoảng đo; wrap góc; hiệu chỉnh gyro | RB dùng MPU6500, không giả định có heading tuyệt đối BNO055; quãng đường có dấu lấy từ encoder đo được, kể cả khi lệnh DRIVE bằng 0 |
| `pc_ws/src/ardubot_projected_map_clearing/ardubot_projected_map_clearing/grid.py` | Hệ tọa độ ô map, mở rộng theo block, quy ước unknown/free/occupied | Lớp này cần map đầu vào từ OctoMap và chỉ lưu lịch sử free; không phải bộ tích lũy vật cản hoàn chỉnh. RB cần bộ cập nhật occupancy từ số đo |
| `pc_ws/src/ardubot_pc_bringup_pointcloud/launch/all_in_one.launch.py` | Depth → điểm 3D → occupancy 3D → chiếu map 2D | Thực hiện bằng lõi C++ nhỏ, không mang launch/ROS stack sang |
| `pc_ws/src/ardubot_pc_bringup_pointcloud/config/slam.yaml` | Mốc mapping bằng odometry | File đang tắt scan matching và loop closure; không coi đây là một SLAM backend đã hoạt động |

Không áp dụng phần `clear_fov()` để tự tô trống toàn góc nhìn: ToF mất số đo hoặc tia không chạm vật không đủ để kết luận vùng đó trống. Khi port mã trực tiếp phải giữ nguồn và thông báo giấy phép tương ứng của mã tham khảo.

## 3. Kiến trúc dữ liệu

```text
MuJoCo + controller đang có
  ├─ adapter ToF mô phỏng: 64 range, status, time ─┐
  ├─ IMU và encoder/khớp có nhiễu, time ──────────┤
  │                                              ▼
  │                                recorder + native mapper
  │                                  │          │
  │                                  │          └─ pose/path + occupancy/map
  │                                  └─ sensor log để replay
  │                                                     │
  └─ ground truth ghi riêng ──► evaluation               ▼
                                                cửa sổ map hiện có GUI
```

ToF phải phát tại nhịp cảm biến 15 Hz từ simulator, không tính theo số frame GUI render được. Dùng `mj_ray` trên CPU tại pose vật lý của thời điểm đo; loại hình học robot khỏi mục tiêu đo. Chỉ bộ tạo cảm biến này được biết cảnh thật. Mapper không nhận mesh, camera matrix thế giới hoặc tọa độ thật của robot.

Giao tiếp dùng dòng record có kết thúc newline, theo cách runner hiện xuất telemetry. Adapter chuyển record cảm biến vào stdin của mapper; stdout mapper trả kết quả, stderr dành cho lỗi. Không thêm mạng/DDS cho trường hợp chạy cùng PC. Hàng đợi có giới hạn; không bỏ âm thầm các mẫu IMU/encoder cần tích phân. Khi quá tải, ưu tiên bỏ frame ToF cũ hoặc đánh dấu mất dữ liệu.

Các record cần có:

| Record | Trường bắt buộc |
| --- | --- |
| Calibration đầu phiên | Phiên bản schema, hệ trục, đơn vị SI, bán kính/track bánh, quy ước góc khớp và zero, pose quang học ToF, 64 ray, tầm đo và rate |
| IMU | Sequence, thời gian lấy mẫu đơn điệu, gyro 3 trục, accel 3 trục, trạng thái hợp lệ; pitch ước lượng hiện có nếu dùng phải ghi rõ nguồn/frame |
| Encoder/khớp | Thời gian lấy mẫu, góc đo có dấu của hai bánh, hip/knee hai bên, trạng thái hợp lệ |
| ToF | Sequence, thời gian lấy mẫu, 64 radial range theo mét, 64 status/validity; uncertainty nếu simulator cung cấp |
| Kết quả | Thời gian, pose ước lượng, chất lượng tracking, path, thay đổi ô map và metadata gốc/kích thước/độ phân giải |

Không gán confidence/amplitude giả khi không có phép đo tương ứng. Record sai kích thước, NaN/Inf, timestamp đảo ngược hoặc mất calibration phải bị từ chối có chẩn đoán. Reset simulator bắt đầu phiên map mới để không ghép hai hệ tọa độ.

## 4. Các mốc triển khai theo thứ tự

### Mốc A — Xuất và ghi được số đo ToF độc lập

1. Tách logic 64 tia và nhiễu hiện ở `technology/scan3d_fusion/sensor/camera_viewer.py` sang `simulation/sensors/` với adapter gọi từ runner.
2. Giữ `technology/scan3d_fusion/config/tof.yaml` làm nguồn calibration/noise. Dùng tâm quang học được suy ra đúng như model generator, không nhầm với tâm module. Từ cấu hình hiện có: lệch trái khoảng 2 cm, yaw −4°.
3. Phát ToF ở 15 Hz, đồng thời xuất IMU và encoder ở nhịp lấy mẫu nguồn. Ghi seed, cấu hình, phiên bản mã và clock vào metadata.
4. Cho cửa sổ ToF đọc cùng frame này để thứ người dùng thấy và thứ mapper nhận khớp nhau. RGB-D không cung cấp range cho nhánh ToF.
5. Ghi sensor log riêng với ground truth để có thể replay khi tắt simulator.

Hoàn thành khi: frame luôn có 64 vùng đúng thứ tự, timestamp đúng nhịp; đóng viewer vẫn ghi được dữ liệu; cùng seed và kịch bản có thể tái lập phép đo; mapper không nhận record `VIEW`.

### Mốc B — Ước lượng chuyển động và tư thế cảm biến

1. Khởi tạo vị trí/hướng cục bộ bằng 0 khi bắt đầu một phiên hỗ trợ trên sàn; không lấy pose thật làm seed.
2. Tính quãng đường hai bánh từ góc có dấu. Bù chuyển động thân và chuỗi chân hip/knee theo cơ học RB; kiểm tra quy ước zero motor/khớp hiện có trước khi dùng công thức.
3. Tính yaw từ gyro có bù bias, đối chiếu với chênh lệch quãng đường hai bánh. MPU6500 không cung cấp heading tuyệt đối kiểu BNO055; phải theo dõi bất định yaw.
4. Tích phân XY bằng hướng giữa hai thời điểm. Lệnh đứng yên không được làm odometry đứng yên giả: bánh tự sửa cân bằng vẫn là chuyển động cần ghi nhận.
5. Bù pitch; cần roll cho trường hợp nghiêng ngang. Estimator hiện chủ yếu trả pitch nên adapter phải bổ sung ước lượng roll từ IMU hoặc đánh dấu khung đo không đủ tin cậy khi lệch khỏi giả định sàn phẳng.
6. Ước lượng chiều cao và vị trí thân so với trục bánh từ encoder khớp + hình học hiệu chuẩn. Không dùng `VIEW.z` hay vị trí thân mong muốn thay cho chiều cao đo được.
7. Lưu lịch sử ngắn để nội suy pose tới timestamp ToF, không ghép với pose GUI nhận gần nhất.

Hoàn thành khi: encoder tiến/lùi và quay cho dấu đúng; đổi chiều cao tại chỗ không tạo quãng đường giả lớn; dao động pitch không làm tường nhấp nhô; input stale/reset không làm pose nhảy âm thầm. Trong lúc đứng dậy, ngã hoặc thiếu tư thế hợp lệ, tạm ngừng tích lũy map và hiển thị lý do.

### Mốc C — Từ ToF thành điểm và bằng chứng vật cản

1. Với mỗi vùng hợp lệ, tính `point_sensor = range × unit_ray`; range ở đây là khoảng cách dọc tia, không phải Z của ảnh depth.
2. Dùng pose quang học cố định và pose thân nội suy để đưa cả gốc tia lẫn điểm chạm sang hệ odometry.
3. Loại số đo lỗi, ngoài tầm, self-return và điểm không thuộc chiều cao môi trường cần vẽ. Xác định sàn bằng hệ trục/động học, không bằng cắt cố định hàng ảnh.
4. Giữ cạnh và vật nhỏ: không lấy median 5×5 của ArduBot áp trực tiếp lên 8×8; không nội suy 64 vùng thành phép đo độ phân giải cao.
5. Lúc đầu giữ mô hình một range mỗi vùng đang có, ghi rõ đây là ray đại diện tâm vùng; độ rộng vùng và uncertainty cần phản ánh khi đánh giá độ dày tường.

Hoàn thành khi: tường, sàn và một vật trước mặt được chiếu đúng phía; quay robot hoặc thay chiều cao không đổi sai hệ trục của các điểm.

### Mốc D — Occupancy và map xám tích lũy

1. Dùng occupancy voxel 3D thưa, giới hạn trong dải chiều cao cần quan sát, rồi chiếu xuống map 2D. Đây là bản tối thiểu của đường point cloud → occupancy 3D → projection trong ArduBot; không cần OctoMap runtime.
2. Khởi đầu với voxel/ô map 5 cm. Đây là kích thước lưu trữ; không phải cam kết cảm biến có độ chính xác hay khả năng phân biệt vật 5 cm.
3. Mỗi tia hợp lệ bổ sung bằng chứng trống cho voxel đã đi qua, bằng chứng có vật cho vùng điểm chạm; cập nhật xác suất/log-odds có chặn biên. Giữ một vùng bất định gần điểm chạm theo nhiễu đo.
4. Cập nhật theo voxel trước khi chiếu 2D để tia nhìn phía trên vật không xóa vật ở lớp thấp. Chỉ dọn vùng thực sự quan sát; frame mới không được thay toàn bộ lịch sử vật cản.
5. Khử trùng cập nhật voxel trong cùng frame, tránh nhiều tia trong một ô làm tăng confidence quá mức. Không xóa cả hình quạt 45° khi có mẫu invalid/no-return.
6. Map 2D: occupied nếu có lớp vật cản đủ bằng chứng; free theo các lớp đã quan sát và chính sách chiếu công bố rõ; phần chưa đủ quan sát giữ unknown. Lớp free ban đầu chỉ là thông tin quan sát, chưa là chứng nhận robot đi qua được.
7. Giới hạn phạm vi/số voxel và chẩn đoán khi đạt trần bộ nhớ. Thông số giới hạn được chốt theo RAM mục tiêu; bản PC đầu tiên không mặc nhiên đạt ngân sách ESP32.

Hoàn thành khi: tường vẫn còn khi rời FoV; xem lại từ góc khác củng cố hoặc sửa map; mất ToF không tự tô trống; sàn không biến thành một khối vật cản; vật không biến mất vì một tia đo ở độ cao khác.

### Mốc E — Cửa sổ map và lưu kết quả

1. Thêm cửa sổ map vào GUI Tk/Pillow đang dùng, nhận kết quả native mapper.
2. Vẽ trắng = vùng trống đã quan sát, đen = vật cản, xám = chưa biết; kèm thước mét, quỹ đạo, hướng robot và 64 điểm mới nhất.
3. Hiển thị tuổi frame, số vùng ToF hợp lệ, số frame bị bỏ, tình trạng odometry/mapping và thời gian xử lý.
4. Cho reset phiên và lưu map: ảnh PGM, metadata JSON chứa resolution/origin/frame, path và snapshot occupancy để tiếp tục kiểm tra offline.
5. Cập nhật bản đồ theo vùng thay đổi, refresh viewer thấp hơn nhịp IMU; GUI chậm không làm chậm sensor/controller.

Hoàn thành khi: một lần chạy trong phòng cho map và path xem được trực tiếp, lưu ra file đúng tỷ lệ, replay cùng sensor log cho cùng kết quả map.

### Mốc F — Đánh giá và bàn giao bản mapping đầu tiên

| Kịch bản | Điều cần xác nhận |
| --- | --- |
| Mặt phẳng và góc tường đơn giản, pose input biết trước trong test hình học | Dấu hệ trục, radial range, extrinsic yaw −4°, chỉ ô/voxel dự kiến được cập nhật |
| RB cân bằng tại chỗ | Không đóng băng chuyển động thật; tách độ trôi odometry khỏi nhiễu ToF, quan sát độ dày tường |
| Thay chiều cao và quay hướng | Bù chuyển động chân/pitch và pose ToF theo timestamp |
| Đi thẳng rồi lùi | Quỹ đạo, map giữ lịch sử và độ lặp lại khi nhìn lại một tường |
| Đi qua góc phòng, quay về vùng cũ | Đo lệch tường/quỹ đạo do odometry; không che sai số bằng pose thật |
| Invalid, mất frame, timestamp reset, viewer đóng | Không tạo free space giả, không làm treo controller, thông báo đúng trạng thái |
| Replay log khi không cài/chạy MuJoCo | Lõi mapper thật sự độc lập simulator và ROS |

Test tập trung vào hình học, tích phân odometry, giao nhau của tia/voxel, log-odds, chiếu map và parser lỗi. Kiểm tra build mapper độc lập, không có liên kết ROS/MuJoCo/firmware; build lại runner và kiểm tra GUI khởi chạy/đóng đúng các tiến trình.

Evaluator đọc ground truth riêng để báo sai số vị trí/yaw, độ dày và độ lệch tường đã quan sát. Bộ cảnh chuẩn khai báo kích thước tường ngoài runtime mapper. Báo riêng độ phủ và vùng chưa biết, không chỉ đưa một ảnh map trông đẹp.

Mục tiêu hiệu năng ban đầu để đo: mapper xử lý một frame ToF nhanh hơn chu kỳ 66,7 ms và không tích backlog tăng dần; ghi thêm p50/p95, CPU và RAM. Không đặt mục tiêu map sắc nét như ToF 240×180 trước khi có số liệu với 8×8.

## 5. Phạm vi file dự kiến

Chỉ tạo file khi triển khai mốc tương ứng, không dựng sẵn các module rỗng.

| Vị trí | Công việc |
| --- | --- |
| `technology/scan3d_fusion/config/tof.yaml` | Giữ calibration/noise làm nguồn dùng chung; bổ sung metadata quang học/trạng thái nếu cần |
| `technology/scan3d_fusion/mapping/tof_only/tof_sensor.hpp` | Đo 64 vùng tại nhịp mô phỏng, nhiễu có seed, trạng thái vùng |
| `simulation/mujoco/sim_runner.cpp` và CMake của simulator | Gọi sensor, xuất record đo, hỗ trợ ghi ground truth tách biệt |
| `technology/scan3d_fusion/mapping/tof_only/` và CMake độc lập | Schema/parser, odometry, chiếu điểm, occupancy, executable đọc stream/replay |
| `gui/rb1_gui/backend.py` | Quản lý mapper, chuyển record, recorder và vòng đời tiến trình |
| `technology/scan3d_fusion/sensor/camera_viewer.py` | Đọc frame ToF từ nguồn chung thay cho tự đo; tiếp tục xem RGB/RGB-D |
| `technology/scan3d_fusion/viewer/tof_map_viewer.py` | Cửa sổ map/path và lưu kết quả |
| `datasets/raw/` | Log đo + metadata; không commit log lớn |
| `evaluation/` | So sánh estimate/map với ground truth offline |
| `docs/slam_simulation_architecture.md`, `slam/README.md`, `vision/README.md` | Cập nhật trạng thái, quy ước dữ liệu và lệnh chạy/replay |

Thứ tự bàn giao: A → B → C → D → E → F. Sau A/B đã có dữ liệu và quỹ đạo; sau D/E có map live; F xác định chất lượng và giới hạn trước khi mở rộng thuật toán.

## 6. Sau khi vẽ được map

Chỉ khi map theo odometry đã đúng hình học, mới thêm ghép scan/điểm ToF với map để hiệu chỉnh XY/yaw. Số đo thưa, tường đơn và hành lang có thể không ràng buộc đủ cả ba bậc tự do; bước này phải có kiểm tra độ tin cậy và từ chối correction không quan sát được. Khi độ tin cậy thấp, tiếp tục odometry và đánh dấu chất lượng, không ép map khớp bằng pose thật.

Loop closure, tối ưu pose graph, dựng lại map sau correction và navigation là các mốc tiếp theo. Chúng chưa thuộc điều kiện bàn giao map theo ArduBot ở kế hoạch này. Không hứa một độ chính xác hoặc loại map 3D chi tiết vượt khả năng 64 vùng đo.

## 7. Tiêu chí chốt mốc vẽ map

- Chạy một lệnh/Start SIM để có dữ liệu ToF và bản đồ xám tích lũy theo quỹ đạo ước lượng.
- Tường/vật lớn xuất hiện đúng hướng và tỷ lệ, không mất khi rời FoV; vùng không có phép đo vẫn unknown.
- Đổi chiều cao và giữ thăng bằng được bù bằng số đo/động học, không truy cập pose thật.
- Lưu và replay được phiên chạy, mapper build/chạy không ROS và không MuJoCo.
- Có báo cáo định lượng độ trôi, độ phủ map và tài nguyên; nêu rõ trạng thái mapping theo odometry và các phần SLAM chưa triển khai.

Tài liệu này là kế hoạch triển khai. Chưa thêm hoặc thay đổi thuật toán mapping trong lần lập kế hoạch này.
