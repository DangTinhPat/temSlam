# Simulation scenarios

Đặt tại đây cấu hình môi trường, vị trí xuất phát, seed, đường đi tham chiếu cho robot và thời lượng lần chạy. Đường đi tham chiếu chỉ là kịch bản actuator; nếu cần quỹ đạo thật để đánh giá, ghi nó qua recorder riêng trong `evaluation/`, không cung cấp cho SLAM.

## Indoor Small House

`indoor_small_house.xml` là cảnh AWS RoboMaker Small House đã chuyển sang MJCF và ghép với mô hình RB. Scene nhiều phòng này được giữ làm cảnh tham khảo; vị trí bắt đầu mặc định gần tâm nhà. Chọn bằng `RB1_SIM_MODEL` khi cần.

Tạo lại scene sau khi cài các thư viện chuyển đổi:

```bash
python3 -m venv /tmp/temslam-map-env
/tmp/temslam-map-env/bin/pip install -r tools/model/requirements_indoor_map.txt
/tmp/temslam-map-env/bin/python tools/model/import_small_house.py
```

## Indoor Tiny House

`indoor_tiny_house.xml` là **mặt bằng SLAM chính thức** và mặc định khi mở GUI. Kích thước trong nhà 11,52 × 10,08 m; lưới ToF có phạm vi 12,8 × 12,8 m, độ phân giải 2,5 cm. Kích thước thực của robot, đồ đạc và các lối đi được giữ nguyên. Cảnh có cửa vào rộng 1,2 m, hai vách ngăn có cửa rộng 1,4–1,6 m, sofa và bàn ăn, tủ hai cột ba tầng ngăn, cầu thang chi tiết, trụ hình bình eo thắt và hai vật cản thấp cho ToF. Cảnh AWS lớn ở trên vẫn có thể chọn riêng.

Hai scene nhà hiện tách đầu RB thành khớp `head_yaw` với hành trình mô phỏng ±270°. ToF, RGB camera và CAD đầu gắn trên khớp này. Servo vị trí trong MuJoCo dùng đặc tính mô phỏng để kiểm tra lia đầu; cần hiệu chỉnh mô hình mô-tơ trước khi suy ra tốc độ hoặc mô-men trên phần cứng.

Mặc định GUI mở mặt bằng SLAM chính thức. Để mở cảnh AWS tham khảo:

```bash
RB1_SIM_MODEL=simulation/scenarios/indoor_small_house.xml python gui/sim_gui.py
```

Hoặc chọn file XML này trong lệnh chạy `rb1_sim` bằng `--model`.

Ảnh xem trước [nhìn từ trên](../assets/aws_robomaker_small_house/indoor_small_house_topdown.png). Nguồn và license: [AWS RoboMaker Small House](../assets/aws_robomaker_small_house/SOURCE.md).

### Wall collision regression

HouseWallB collision surfaces are converted to individual convex triangular
prisms (2 mm thick). Bounding whole wall faces used to fill doorways and balcony
openings with invisible collision boxes. The source surface positions and real
solid walls are preserved, including the low balcony parapet.

Run `python tools/model/test_house_collision.py` with the importer dependencies
and MuJoCo installed. It checks three openings, three solid surfaces, the full
robot footprint through two clear corridors, and importer/scene agreement.
A 28 s native simulation through the room opening at x = −2.6 m, y = −1.2 m
(unit 648, 0.15 m/s command) previously stopped at x = −2.26 m and fell;
the corrected scene crossed it and finished balancing. Furniture is now split into connected convex pieces; non-convex surfaces use
the same thin-prism method. This preserves gaps below tables and between chair
legs instead of letting MuJoCo fill them with a single convex hull. Two source
ball primitives also have visible sphere geoms, matching their collision shapes
and making them observable by camera/ToF. Unsupported source visual geometry
now fails import rather than silently creating invisible obstacles.

The regression also checks all 87 source instances / 110 visible parts, all
1,602 convex collision assets, four furniture gaps and their solid surfaces,
and ray visibility of both balls. See [the full scene audit](../../evaluation/house_scene_audit.md).
