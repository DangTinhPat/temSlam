# SLAM layer

Đặt frontend, backend, odometry, localization, mapping và adapter dữ liệu trong khu vực này. Module SLAM chỉ được phụ thuộc vào message/log cảm biến và cấu hình sensor; không phụ thuộc `mujoco`, `firmware/` hay API lấy trạng thái thật của plant.

Sensor suite và backend chưa được chốt. Khi chốt, tạo package/source tree cụ thể theo thư viện được chọn thay vì dựng sẵn các node rỗng.
