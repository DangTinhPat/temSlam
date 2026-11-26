# Kế hoạch kiểm chứng đóng vòng của cuVSLAM trên bmap1

**Mục tiêu** là giữ cuVSLAM 17.0.1 làm nguồn matching thị giác, đóng vòng và tối ưu quỹ đạo. Khi robot đi hết một vòng, các lần quan sát cùng bức tường phải khớp trên mesh.

**Kiến trúc** dùng RGB + ToF 8×8 qua DELTAR, cuVSLAM RGB-D trên GPU và TSDF. Wheel/IMU hỗ trợ kiểm tra chuyển động. Pose thật từ MuJoCo chỉ dùng khi chấm kết quả. Triển khai tuần tự theo từng phép thử để có thể quay về bản đang dùng sau mỗi thay đổi.

### 1. Ghi được quyết định của SDK và quyết định của ứng dụng

- [x] Lấy phiên `datasets/raw/tof-20261125-171035-804094` làm mốc. Tuyến hoàn thành với 418 keyframe. Kiểm tra chuyển động camera đầu–cuối sai 37,1 cm và 11,14°. Hai cặp tường lệch 10,16° và 1,28°. Phiên này thiếu chuỗi RGB-D đầu vào đầy đủ và lịch sử pose ứng viên cuối đã bị loại.
- [x] Trong `vslam/tracker.py` và `simulation/demo/live/worker_tracking.py`, dùng lại phép tính của `graph_is_consistent`. Tại mỗi sự kiện graph và khi kết thúc, lưu riêng pose đang dùng và pose SDK đề xuất theo timestamp. Ghi sai lệch tại gốc, cặp timestamp vượt ngưỡng, độ dịch, góc quay, timestamp thiếu và thời điểm tracker đổi phiên sau mất bám. Giữ quyết định duyệt hiện tại để lấy số gốc.
- [x] Ghi `get_pose_graph()` và `get_slam_metrics()` khi có sự kiện. So ID hai đầu của cạnh mới với các node đã xuất hiện ở lượt đi và lượt về. Node ID của SDK và số thứ tự keyframe dựng mesh là hai hệ đánh số riêng. Nếu xảy ra nhiều thay đổi giữa hai lần đọc khiến liên kết mơ hồ, đánh dấu chưa xác minh được cạnh đầu–cuối.
- [x] Bổ sung vào `vslam/test_tracker.py` hai đầu vào sẵn có là hiệu chỉnh phân bố dọc đường và một bước nhảy bất thường. Kiểm tra quyết định duyệt khớp bản cũ và log chỉ đúng bước nhảy. Chạy thêm `simulation/demo/live/test_tracking_guard.py` để kiểm tra mesh cuối được giữ khi graph bị bác.
- [x] Chạy tuyến cũ một lần ở ×1 với `simulation/demo/live/headless.py --speed-factor 1 --record-rgbd` qua môi trường demo. Ghi vào thư mục phiên mới. Kiểm tra đối chiếu được timestamp từ tracker, wheel/IMU, SDK tới keyframe mesh. Các phép phát lại dùng chung chuỗi này. Số đo thời gian hiện có thuộc lượt ghi RGB-D. Đo độ trễ vận hành được để ở mục 3.

### 2. Dựng ngoại tuyến và chọn đúng nơi cần sửa

- [x] Trên cùng các ảnh sâu và cùng danh sách keyframe của lượt mới, dựng A từ pose ứng dụng đã duyệt và B từ toàn bộ pose SDK đề xuất. Dùng cùng mốc tọa độ, mount camera, ràng buộc cao độ, voxel và phép đo tường. Nếu SDK thiếu pose hoặc đã khởi động lại, tách tại ranh giới đó và báo phần phủ được. Chỉ so toàn vòng khi cùng phủ đủ tuyến.
- [x] Chấm sai số chuyển động tương đối so với MuJoCo ở đầu–cuối bằng `simulation/demo/live/pose_check.py`. Chấm thêm ở hai thời điểm robot thực sự đi qua lại cùng vùng, rồi đo cả hai cặp tường trên cùng đoạn có quan sát. Robot có thể về lệch điểm xuất phát một chút, nên khoảng cách hai điểm trên map tự nó chưa phải sai số.

| Kết quả đối chứng | Thay đổi nhỏ nhất để thử |
| --- | --- |
| Chưa có bằng chứng SDK nối đúng đoạn về với đoạn đầu | Xem cặp ảnh RGB-D lúc quay về, số landmark được chọn và theo dõi, vùng ảnh chồng lấp, khoảng cách thời gian giữa các khung và lần khởi động lại tracker. Thử một lượt quay lại cùng cửa, cùng hướng quan sát để phân biệt thiếu chồng lấp với lỗi tích hợp. Tuyến gốc vẫn là bài nghiệm thu. |
| B sửa được quỹ đạo và hai tường, A vẫn méo | Kiểm tra tại cặp timestamp đầu tiên gây từ chối, từ hệ tọa độ, căn thời gian đến ranh giới bắt bám lại. Sửa nguyên nhân tái hiện được. Giữ ca kiểm thử từ chối bước nhảy thật. |
| SDK đã nối đúng vùng nhưng pose B vẫn sai | Đối chiếu chuyển động tương đối, yaw wheel/IMU và depth ở chính hai lượt qua cửa. Thử riêng một giả thuyết có số đo ủng hộ, chẳng hạn căn chỉnh RGB–depth khi biên sâu lệch ảnh. |
| Quỹ đạo B đạt nhưng mesh B vẫn méo | Đối chiếu thứ tự timestamp, mount và pose mỗi ảnh thực sự đi vào TSDF. Dựng bản tham chiếu ngoại tuyến từ chính các ảnh sâu với pose MuJoCo để khoanh vùng lỗi depth hoặc phép tích hợp. |

### 3. Điều kiện đưa vào demo

- [ ] Đề xuất hai lượt độc lập trên tuyến gốc và một lượt đi ngược có nhìn lại cùng khu vực. Mỗi lượt phải có bằng chứng khép vòng ở đoạn quay về và đạt phép kiểm đầu–cuối hiện có (≤10 cm, ≤5°). Mục tiêu **đề xuất riêng cho bmap1** là mỗi cặp tường đối diện lệch ≤2° trên đoạn đo có độ phủ tương đương baseline. Chấm trên mesh dựng trực tiếp, trước lọc Greater.
- [x] Chạy thêm một đoạn hở qua các tường giống nhau. Kiểm tra vùng chưa quay lại không bị nhập nhầm, không có bước nhảy giả và khi mất bám robot vẫn giữ được bản đồ cuối đã duyệt. Giữ kết quả thử trong thư mục phiên riêng. Nếu trượt, dùng tiếp cấu hình trước thử nghiệm và ghi lại điều kiện thất bại.
- [ ] Khi đạt hình học, đo trung vị và p95 của thời gian fusion, tracking và độ trễ từ thu ảnh tới pose có thể dùng, cùng tỷ lệ khung bỏ qua. So trên cùng phần cứng ở ×1 với cùng chế độ ghi, dùng đồng hồ monotonic cho độ trễ thực. Nếu fusion chiếm phần lớn thời gian lặp lại qua nhiều khung, thử tối ưu riêng bước đó rồi chạy lại bài nghiệm thu hình học.

### Kết quả mô phỏng ngày 26/11

Các mã phiên dưới đây nằm trong `datasets/raw/`. Tất cả lượt chạy mới dùng tốc độ ×1. Pose tham chiếu MuJoCo chỉ đi vào phép chấm ngoại tuyến của SLAM. Mesh dùng trực tiếp depth RGB + ToF qua DELTAR trước Greater. Các góc dưới đây được đo trên các dải chung của từng cặp tường.

| Tuyến | Keyframe mesh | Sai số đầu–cuối của mesh đang dùng | Bằng chứng trong graph |
| --- | ---: | --- | --- |
| Thuận, `tof-20261126-094451-480884` | 416 | 27,62 cm / 9,84° | 0 LC, 0 PGO. SDK đề xuất hai bước nhảy khi bám lại, lớn nhất 54,9 cm / 15,7°. Dùng nguyên ứng viên làm sai số tăng lên 77,20 cm / 11,80°. |
| Ngược, `tof-20261126-095645-084264` | 402 | 5,03 cm / 10,97° | Cạnh 145 → 149 chỉ nối hai lần quan sát cách nhau 3,2 giây trong cùng khúc cua. Robot về vùng đầu từ giây 558. |
| Đi lại trước sofa, `tof-20261126-101033-082845` | 115 | 2,68 cm / 2,54° | 30 LC và 30 PGO. Có bốn cạnh xác minh ở hai lượt qua vùng gần sofa, còn trong graph cuối đã duyệt. Điểm đầu và toàn bộ bốn tường chưa đạt phép kiểm riêng. |
| Toàn phòng có nhìn lại sofa, `tof-20261126-101732-917270` | 492 | 8,98 cm / 10,19° | Đi đủ 11 điểm, dài 20,44 m. Cả 18 LC ở bản trực tiếp đều thiếu liên kết sang lượt đi trước khi vòng quanh phòng. Hai cặp tường lệch 7,81° và 1,70°. |
| Đối chứng hở, `tof-20261126-101412-911975` | 132 | Tuyến hở 5,50 m | 0 LC, 0 PGO, 0 lần bắt lại, graph cuối được duyệt. Ca mất bám giữ map cuối được kiểm tra riêng bằng test của worker. |

Với cùng ảnh đầu vào, thử thay phần quay trong gia số odometry gửi tới native SLAM bằng góc đo wheel/IMU. Tịnh tiến vẫn lấy từ cuVSLAM. Trên lượt thuận, góc giữa hai cặp tường từ 11,16° và 4,23° xuống 1,81° và 0,17°. Lượt ngược từ 2,60° và 4,81° xuống 1,17° và 0,51°. Sai số tịnh tiến điểm cuối của bản thử là 8,21 cm trên lượt thuận, 13,14 cm khi phát lại độc lập cùng lượt, và 11,61 cm ở lượt ngược. Cả ba lượt phát lại đều thiếu cạnh xác minh tại điểm đầu. Giữ chế độ này ở bước thử ngoại tuyến.

Lượt thuận có tám mẫu định vị vào checkpoint cũ khi quay về. Năm mẫu sai dưới 0,7 cm, ba mẫu sai 26,4–26,9 cm. Luồng trực tiếp tra checkpoint trên tracker phụ, kiểm tra lại bằng chuyển động wheel/IMU trước khi chấp nhận. Các phép thử tăng mật độ landmark theo kích thước ô và bật ràng buộc phẳng SDK cũng chưa tạo được cạnh khép vòng toàn phòng.

Thử tạm dừng tiến trình mô phỏng cho thấy khi chạy bù đồng hồ, camera có thể phát dồn 0,6 giây mô phỏng trong khoảng 0,05 giây thực. Đã sửa điều tốc ở nguồn cho chế độ capture. Test lặp lại ở ×1 và ×2. Trên 2.464 mẫu của lượt thuận sau khởi động, thời gian fusion trung vị 174 ms, p95 198 ms. Đây là thời gian của bước fusion khi đang ghi dữ liệu.

Để tách lỗi pose khỏi lỗi depth, dựng lại đúng 416 và 402 ảnh sâu, TSDF 4 cm, chỉ thay quỹ đạo bằng pose tham chiếu MuJoCo ở đúng timestamp. Trên lượt thuận, hai cặp tường lệch 0,42° và 0,41°. Lượt ngược lệch 0,10° và 0,01°. Các bản này là đối chứng ngoại tuyến. File số đo của từng phiên là `loop_ab/reference_geometry.json`.

Các file để đối chiếu gồm `graph_snapshots/return_assessment.json`, `graph_snapshots/visit_assessment.json` khi chấm vùng sofa, `loop_ab/metrics.json` và `native_replay/`. Hình so sánh bốn mesh nằm trong `tof-20261126-094451-480884/loop_ab/tuong_truoc_sau.png`.

### Thử phối hợp đoạn nhìn lại sofa với góc quay wheel/IMU

Dùng lại toàn bộ 3.155 RGB-D của phiên `tof-20261126-101732-917270`. Phần tìm tương ứng thị giác, nhận diện lại và tối ưu graph dùng cuVSLAM trên GPU. Bản thử giữ nguyên tịnh tiến thị giác và thay gia số quay bằng góc đo trước khi gửi trạng thái odometry vào native SLAM.

| Cùng bản ghi, cấu hình và 492 keyframe dựng mesh | Sai số điểm cuối | Tường bắc–nam / trái–phải | Cạnh nối hai lượt qua khu vực sofa còn trong graph cuối |
| --- | --- | --- | --- |
| Phát lại 1, `native_replay/cell/default.ply` | 4,24 cm / 0,60° | 1,10° / 0,61° | 11 |
| Khởi tạo lại và phát lại 2, `native_replay/repeat/default.ply` | 25,23 cm / 1,97° | 3,34° / 0,13° | 0 |

Các góc trên dùng toàn bộ dải chung của hai tường trong từng mesh, cùng quy tắc với `headless_result.json`. Điểm kiểm sofa là (0,25 m, −0,20 m), bán kính 0,33 m. Điểm xuất phát là (0,80 m, −0,60 m), bán kính 0,40 m. Cả hai bản phát lại đều thiếu cạnh nối chính hai lượt qua vùng xuất phát. Trong bản đầu, cạnh 30 → 452 nối quan sát gần giây 96,00 với giây 741,14, cách nhau 7,1 mm theo vị trí thân tham chiếu. Hai biến thể dùng nguyên bản ghi, kể cả thứ tự ảnh và timestamp. Trước lúc về sofa, khoảng giây 730,8, hai kết quả visual odometry thô đã khác nhau 1,11 m và 11,96° tính tương đối từ pose đầu. Phép đo khoanh vùng sai khác phát sinh trước bước đóng vòng. Chưa xác định được chi tiết tính toán nào của VO tạo ra khác biệt đầu tiên.

Đối chiếu `loop_ab/toan_phong_ba_mesh.png`, `loop_ab/three_mesh_walls.json`, `native_replay/vo_repeatability.json` và `native_replay/{cell,repeat}/verified_visits.json`. Các lượt thuận/ngược trên tuyến gốc vẫn là điều kiện nghiệm thu riêng.

Một đối chứng nữa chạy hai graph native trên cùng trạng thái VO. Cả hai dùng góc wheel/IMU. Ở nhánh thử, lấy trung bình phần tịnh tiến ngang do encoder và thị giác ước lượng trên từng bước, giữ phần thẳng đứng của thị giác. Trên tuyến toàn phòng có sofa, sai số điểm cuối giảm từ 29,86 cm xuống 15,74 cm, hai tường còn 2,02° và 0,85°, vẫn thiếu cạnh toàn vòng. Hai tuyến gốc thuận và ngược lần lượt còn 20,77 cm và 11,21 cm, cả hai có 0 LC. Tuyến hở có 0 LC ở cả hai nhánh. Hệ số 50% là điều kiện đối chứng, chưa đủ bằng chứng để dùng làm mặc định. Kết quả nằm trong `native_replay/translation_control/` của từng phiên.

Các bài kiểm thử ghi nhận 17/17 test luồng live, 16/16 test vslam và test điều tốc capture ở ×1, ×2 chạy đạt. Toàn bộ CTest firmware có 13/20 đạt. Các test còn lỗi gồm lqr, controller, recovery_transfer, physics, tier, coordination và leg_height. Log chi tiết nằm ở `tof-20261126-094451-480884/loop_ab/firmware_ctest.log`.
