# temSlam

Workspace phát triển mô phỏng RB và SLAM ToF. Điều khiển robot chạy native qua firmware/controller; mapper ToF chạy C++ độc lập. ROS 2 chỉ là cầu nối tùy chọn để xem kết quả trong RViz.

## Bắt đầu nhanh

Sau khi clone, lấy đúng phiên bản các repo tham chiếu bằng:

```bash
git submodule update --init --recursive
```

Build simulator và mapper, rồi mở GUI:

```bash
cmake -S firmware -B firmware/build -DCMAKE_BUILD_TYPE=Release
cmake --build firmware/build -j8 --target rb1_sim
python3 gui/sim_gui.py
```

GUI dùng `simulation/scenarios/indoor_tiny_house.xml` mặc định. Để chạy mapper/replay độc lập, xem [SLAM ToF-only](technology/scan3d_fusion/mapping/tof_only/README.md). Các lệnh RViz tùy chọn nằm trong [tài liệu cầu nối](technology/scan3d_fusion/mapping/native/README.md).

Chạy runner trực tiếp không mở GUI:

```bash
firmware/build/bin/rb1_sim --model model/dynamic.xml --start-key home --run-for 26 --at 0.3:STANDUP
```

Yêu cầu cơ bản: CMake ≥ 3.20, C++17, Python 3, MuJoCo (header và thư viện); `python3-tk` để mở GUI.

## Bản đồ dự án

| Khu vực | Vai trò và nguồn chính |
| --- | --- |
| [`cad/`](cad/) và [`model/`](model/) | STEP gốc, hình học và mô hình robot dùng chung. YAML trong `model/` là đầu vào cho các mô hình sinh tự động. |
| [`firmware/`](firmware/) | Firmware, bộ điều khiển và thư viện native. Vẫn là nơi cấu hình CMake cho runner MuJoCo. |
| [`simulation/`](simulation/) | Runner MuJoCo, scene, sensor model và asset môi trường. Không đặt thuật toán SLAM ở đây. |
| [`technology/scan3d_fusion/`](technology/scan3d_fusion/README.md) | Bộ công cụ Vision + ToF SLAM: cảm biến, calibration, DELTAR model/weights, RGB-ToF mesh, ToF-only mapper, route và RViz. |
| [`gui/`](gui/) | Dashboard điều khiển mô phỏng và các cửa sổ xem map/camera. Gọi runner và mapper, không chứa lõi SLAM. |
| [`vision/`](vision/README.md) | Lối dẫn tới bộ công cụ vision trong `technology/scan3d_fusion/`. |
| [`datasets/`](datasets/) | Log sensor theo phiên chạy, map và dữ liệu đã xử lý. Giữ raw data riêng theo từng session. |
| [`evaluation/`](evaluation/README.md) | Đánh giá offline. Đây là nơi duy nhất được dùng ground truth của mô phỏng. |
| [`tools/`](tools/) | Sinh mô hình, tiện ích phát triển và lệnh mở RViz. |
| [`docs/`](docs/README.md) | Bản đồ tài liệu, kiến trúc và quy trình làm việc. |
| [`prj_reference/`](prj_reference/) | Mã tham khảo bên ngoài; không phải runtime dependency của dự án. |

### Ranh giới dữ liệu và trách nhiệm

- SLAM nhận sensor stream, odometry đầu vào nếu có, và calibration; không nhận pose thật hoặc bản đồ thật.
- Ground truth chỉ được đọc bởi công cụ trong `evaluation/` để chấm kết quả offline.
- `technology/scan3d_fusion/` là nguồn chuẩn của vision/ToF mapping. `gui/`, `simulation/` và `tools/model/` gọi cấu hình hoặc công cụ từ đó.
- Mã ROS tùy chọn chỉ xuất dữ liệu cho RViz; điều khiển, thu thập sensor và mapping vẫn chạy native.
- Scene SLAM mặc định là `indoor_tiny_house.xml`; scene AWS lớn hơn được giữ làm cảnh tham khảo.

## Tài liệu theo việc cần làm

- Chạy simulator, sinh model từ CAD: [docs/simulation.md](docs/simulation.md)
- Hiểu luồng simulator–SLAM và ranh giới ground truth: [docs/slam_simulation_architecture.md](docs/slam_simulation_architecture.md)
- Build/replay mapper ToF: [technology/scan3d_fusion/mapping/tof_only/README.md](technology/scan3d_fusion/mapping/tof_only/README.md)
- Mở map trong RViz: [technology/scan3d_fusion/mapping/native/README.md](technology/scan3d_fusion/mapping/native/README.md)
- Cấu hình, calibration, chạy vision/SLAM hoặc dựng mesh: [technology/scan3d_fusion/README.md](technology/scan3d_fusion/README.md)
- Kiểm tra kết quả mapping: [evaluation/tof_mapping_results.md](evaluation/tof_mapping_results.md)

Chi tiết tài liệu được lập chỉ mục tại [docs/README.md](docs/README.md).
