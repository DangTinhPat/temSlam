# Tài liệu

Điểm bắt đầu theo công việc:

| Chủ đề | Tài liệu | Phạm vi |
| --- | --- | --- |
| Mô hình và simulator | [simulation.md](simulation.md) | Luồng CAD → model → runner MuJoCo; lệnh build/run và tool liên quan. |
| Firmware/controller | [firmware.md](firmware.md) | Cấu trúc firmware và cách build/test controller. |
| Kiến trúc mapping | [slam_simulation_architecture.md](slam_simulation_architecture.md) | Ranh giới giữa simulator, sensor stream, mapper và evaluation. |
| Lập kế hoạch ToF native | [tof_mapping_native_plan.md](tof_mapping_native_plan.md) | Kế hoạch và các quyết định thiết kế cho mapper không ROS. |
| Rà soát CAD | [cad_review.md](cad_review.md) | Giả định và giới hạn khi chuyển CAD sang model động lực. |
| Thông số cảm biến | [datasheets/](datasheets/) | Datasheet gốc dùng để đối chiếu phần cứng. |
| Kết quả chạy | [evaluation/](../evaluation/) | Log, kiểm tra scene và đánh giá offline; không phải tài liệu runtime. |
| Vision + ToF SLAM Toolkit | [scan3d_fusion](../technology/scan3d_fusion/README.md) | Sensor config/preview, calibration, DELTAR model + weights, RGB-ToF mesh, native ToF mapper, route và RViz. |

Các lệnh chuyên sâu nằm trong cùng toolkit: `technology/scan3d_fusion/mapping/tof_only/README.md` và `technology/scan3d_fusion/mapping/native/README.md`. Simulator/CAD và GUI được hướng dẫn riêng vì chúng là thành phần dùng chung toàn workspace.
