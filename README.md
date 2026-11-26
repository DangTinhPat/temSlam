# temSlam · demoSlam

Demo mô phỏng RB đi quanh phòng giữa bmap1 và dựng mesh 3D. Robot đổi góc nhìn bằng cách quay cả thân. Hai cảm biến RGB và ToF 8×8 gắn cố định vào thân.

| Nhóm | Vai trò |
| --- | --- |
| [obser/](obser/) | DELTAR, kiểm tra depth theo ToF, cấu hình và môi trường fusion |
| [vslam/](vslam/) | Adapter cuVSLAM RGB-D, căn hệ map và chẩn đoán pose |
| [simulation/](simulation/) | MuJoCo, scene, tuyến demo, thu cảm biến, tích hợp TSDF và trình xem |
| [firmware/](firmware/) | Bộ điều khiển cân bằng và di chuyển RB |
| `model/`, `cad/`, `tools/` | Mô hình robot và công cụ hỗ trợ |
| `datasets/` | Các phiên quét, log và mesh lưu ở máy chạy |

Luồng chính là **MuJoCo → RGB + ToF → obser → RGB-D → pose graph cuVSLAM → TSDF → mesh**. Xem [sơ đồ và workflow](simulation/demo/architecture.html).

Cụm RGB và ToF của bmap1 trong mô phỏng nghiêng cố định 12° xuống sàn. Hai cảm biến cùng xoay quanh tâm RGB để giữ phép hiệu chuẩn tương đối. Đèn của scene giảm cường độ để tường sáng vẫn còn chi tiết trong ảnh.

## Quỹ đạo dựng map

- cuVSLAM nhận RGB-D theo timestamp và sở hữu pose graph. Trên sàn phẳng, pose dùng dựng map giữ XY và yaw của thân từ cuVSLAM, lấy roll/pitch từ IMU và độ cao thân từ bán kính bánh cùng động học chân. Worker nhân lại phép biến đổi thân–camera theo timestamp trước khi tích hợp depth.
- Bản xem trực tiếp tích lũy keyframe theo visual odometry đã duyệt. Khi SDK báo loop closure hoặc PGO, hoặc khi đóng phiên, chỉ áp dụng graph nếu đủ pose theo timestamp và phần hiệu chỉnh giữa các keyframe không vượt giới hạn. Worker kiểm tra graph trên pose thị giác gốc, rồi áp lại ràng buộc thân của từng keyframe cho cả mesh trực tiếp và mesh cuối. `map_refresh_diagnostics.jsonl` ghi kết quả duyệt graph và mức đổi XY/yaw tại keyframe cuối.
- `tracking_diagnostics.jsonl` ghi sai khác với wheel/IMU và kết quả khớp RGB-D khi bắt lại. `pose_quality.json` ghi số lần bắt lại, từ chối graph và đếm riêng các sự kiện báo từ SDK về loop closure và PGO.
- `headless_result.json` đối chiếu chuyển động tương đối giữa keyframe đầu/cuối với capture MuJoCo đồng bộ, kể cả khi RB quay về với hướng khác. Kiểm tra yêu cầu có cả hai loại sự kiện SDK và sai số tối đa 10 cm, 5°. Capture tham chiếu chỉ được đọc sau khi dựng mesh.
- `live_sequence.json` đánh dấu preview hoặc final. TSDF dựng mesh cuối từ snapshot final khi tracker dừng sạch. Nếu mất tracking, phiên giữ lại dữ liệu đang có để phân tích.
- Chốt pose so chuyển động thị giác với wheel/IMU ở bước mới nhất và trên cửa sổ sáu pose đã duyệt. Sai khác trên 8 cm hoặc 0,12 rad sẽ tạm dừng ghép. Khi bắt lại, worker thử khớp RGB-D với ảnh tốt cuối và các keyframe gần vị trí dự đoán. Các điểm dùng để căn pose phải có cả độ phủ ảnh và depth phù hợp. Worker xác nhận qua ba ảnh liên tiếp trước khi tiếp tục.
- Khi mất bám kéo dài, worker vẫn nhận ảnh và giữ mesh cuối. Nếu đã lưu được checkpoint nhất quán với map, worker có thể gọi cuVSLAM để tái định vị vào checkpoint. Wheel/IMU cung cấp dự đoán cho phép tìm kiếm. Có thể lái tay đưa camera trở lại vùng đã quét.
- Khi bắt đầu giữ map, worker lưu cửa sổ RGB-D, wheel/IMU, pose và thống kê khớp vào `tracking_event_0.npz` đến `tracking_event_3.npz` luân phiên. Giá trị -1 trong thống kê biểu thị phép kiểm tra chưa chạy.
- Khi có keyframe hoặc cập nhật pose graph mới, worker xuất snapshot theo nhịp khoảng 0,5 giây. Worker mesh nhận snapshot mới theo nhịp khoảng 1 giây và chỉ tích lũy thêm keyframe vào TSDF khi pose cũ và vùng lưới vẫn phù hợp. Sau khi graph sửa pose cũ hoặc map vượt vùng lưới, worker dựng lại lưới. `mesh_diagnostics.jsonl` ghi thời điểm xuất, số keyframe được tích hợp và thời gian dựng của từng mesh.

## Chạy demo

Chạy từ gốc repo trên máy Linux có GPU NVIDIA và MuJoCo. Dùng Python 3.8 với [môi trường DELTAR](obser/requirements.txt) ở `~/.venvs/deltar` và Python 3.12 với [môi trường cuVSLAM](vslam/requirements.txt) ở `~/.venvs/cuvslam`. Có thể đặt `DELTAR_PYTHON` và `CUVSLAM_PYTHON` để chọn trình thông dịch. Cần tải checkpoint NYU vào `obser/deltar/weights/nyu.pt` từ [hướng dẫn DELTAR](obser/deltar/UPSTREAM_README.md).

Build runner và mapper. Python dùng cho CMake cần có PyYAML cùng bản MuJoCo có header, hoặc truyền `-DMUJOCO_ROOT=/duong/dan/mujoco`.

```bash
cmake -S firmware -B firmware/build -DBUILD_TESTING=OFF
cmake --build firmware/build --target rb1_sim -j
./run_vision.sh --speed-factor 1
```

Dòng `SESSION` chỉ thư mục đầu ra. Khi chạy xong xem `result.png`, `final_mesh.stl` và `headless_result.json`. Ở chế độ demo, kết quả có thể được xuất khi kiểm tra khép vòng chưa đạt.

### Kiểm chứng cuVSLAM trên vòng bmap1

Lệnh đầu ghi từng RGB-D gửi tới tracker, pose đề xuất của SDK, các cạnh graph và thời điểm đổi tracker. Với bmap1, lượt ghi đạt khi sai số đầu–cuối ≤10 cm và ≤5°, có cạnh nối hai lượt qua vùng đầu trong graph cuối được áp dụng, SDK đã báo PGO và hai cặp tường đạt phép đo trên mesh. Mỗi cặp tường cần lệch ≤2° trên các dải cùng quan sát được. Số dải tối thiểu theo phiên chuẩn là 15 ở cặp bắc–nam và 24 ở cặp trái–phải, mỗi dải rộng 15 cm. Kết quả từng phép kiểm nằm trong `headless_result.json`. Mỗi phiên toàn phòng cần vài GB dung lượng. Đặt `session` bằng đường dẫn in sau chữ `SESSION` để phân tích cả những phiên trượt.

```bash
python3 simulation/demo/live/headless.py --speed-factor 1 --record-rgbd
vslam/run_python.sh simulation/demo/live/graph_capture.py "$session"
env -u PYTHONPATH ~/.venvs/deltar/bin/python simulation/demo/live/compare_loop.py "$session"
```

Tuyến thử sau giữ các góc của vòng quét phòng và thêm hai lượt nhìn về sofa từ cùng một hướng. Lệnh thứ hai chấm riêng khu vực trên đường tiếp cận sofa, bán kính 33 cm, và ghi `graph_snapshots/visit_assessment.json`. Phép kiểm khép vòng tại điểm đầu vẫn được chấm riêng.

```bash
python3 simulation/demo/live/headless.py --route simulation/demo/mapping/native/bmap1_sofa_perimeter_route.json --speed-factor 1 --record-rgbd
vslam/run_python.sh simulation/demo/live/graph_capture.py "$session" --region .25 -.2 .33
```

Xem cạnh nối lượt đi với lượt về trong kết quả của `graph_capture.py`. File `loop_ab/metrics.json` ghi sai số so với MuJoCo, độ phủ và góc hai cặp tường trên cùng các đoạn quan sát. Mesh A dùng pose đã được ứng dụng duyệt, mesh B dùng pose do SDK đề xuất với cùng ràng buộc cao độ. Khi một phiên bị ngắt hoặc thiếu pose, kết quả so sánh chỉ có giá trị trên đoạn được ghi. Khi tiến trình MuJoCo bị tạm dừng lâu, chế độ capture có điều tốc tiếp tục từ thời điểm mô phỏng hiện tại để tránh phát dồn ảnh.

```bash
bash simulation/demo/viewer/run_clay_rviz.sh datasets/raw/tof-PHIEN_QUET/final_mesh.stl
```

Để chuyển sang RB thật, thay nguồn MuJoCo bằng dữ liệu RGB/ToF và wheel/IMU đồng bộ từ driver, cùng bộ hiệu chuẩn RGB–ToF và camera gắn cố định. Bộ điều khiển RB tiếp tục phụ trách di chuyển.
