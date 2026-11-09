# MuJoCo host runner

`sim_runner.cpp` chạy mô hình RB trong MuJoCo và nối plant với controller hiện có. Đây là ứng dụng host trong lớp mô phỏng; firmware ESP32 và thuật toán điều khiển không được sửa ở bước tái tổ chức này.

Build tương thích với lệnh cũ:

```bash
cmake -S firmware -B firmware/build -DCMAKE_BUILD_TYPE=Release
cmake --build firmware/build -j8 --target rb1_sim
```

Tham số mô hình mặc định và công cụ hiện hành vẫn dùng `model/dynamic.xml`.

GUI lấy `simulation/scenarios/indoor_small_house.xml` làm model mặc định. Viewer tự chọn GTX 1650 bằng Mesa `DRI_PRIME=1` trên laptop hybrid này; đặt `DRI_PRIME=0` trước khi mở GUI nếu muốn dùng GPU tích hợp. Có thể chọn scene khác bằng `RB1_SIM_MODEL=/path/to/model.xml python3 gui/sim_gui.py`.
