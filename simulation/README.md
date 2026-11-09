# Simulation layer

Lớp mô phỏng quản lý plant RB, thế giới, kịch bản, clock, sensor model và ghi dữ liệu. Mã này là nguồn dữ liệu đầu vào cho SLAM. Nó không chứa thuật toán SLAM.

Plant/runner MuJoCo nằm ở `mujoco/`; XML robot cùng mesh ở `model/`. Runner host liên kết controller hiện có trong `firmware/`, nhưng mã mô phỏng không còn nằm trong cây firmware. Các thư mục con bên dưới là điểm đặt cho phần mô phỏng mới.

- `mujoco/`: runner C++ và tích hợp MuJoCo.
- `scenarios/`: world và thông số lần chạy.
- `sensors/`: định nghĩa sensor suite, hiệu chuẩn và mô hình sai số.

Ground truth cần được ghi riêng cho evaluator, không đưa vào sensor stream.
