# Simulation scenarios

Đặt tại đây cấu hình môi trường, vị trí xuất phát, seed, đường đi tham chiếu cho robot và thời lượng lần chạy. Đường đi tham chiếu chỉ là kịch bản actuator; nếu cần quỹ đạo thật để đánh giá, ghi nó qua recorder riêng trong `evaluation/`, không cung cấp cho SLAM.

## Indoor Small House

`indoor_small_house.xml` là cảnh AWS RoboMaker Small House đã chuyển sang MJCF và ghép với mô hình RB. Scene gồm nhiều phòng, hành lang, đồ nội thất, texture và collision proxy tĩnh; vị trí bắt đầu mặc định gần tâm nhà. GUI dùng cảnh này mặc định. Chọn lại model khác bằng `RB1_SIM_MODEL`.

Tạo lại scene sau khi cài các thư viện chuyển đổi:

```bash
python3 -m venv /tmp/temslam-map-env
/tmp/temslam-map-env/bin/pip install -r tools/model/requirements_indoor_map.txt
/tmp/temslam-map-env/bin/python tools/model/import_small_house.py
```

Ảnh xem trước [nhìn từ trên](../assets/aws_robomaker_small_house/indoor_small_house_topdown.png). Nguồn và license: [AWS RoboMaker Small House](../assets/aws_robomaker_small_house/SOURCE.md).
