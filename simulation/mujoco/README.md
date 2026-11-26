# MuJoCo host runner

`sim_runner.cpp` chạy mô hình RB trong MuJoCo và nối plant với controller hiện có. Đây là ứng dụng host trong lớp mô phỏng; firmware ESP32 và thuật toán điều khiển không được sửa ở bước tái tổ chức này.

Build tương thích với lệnh cũ:

```bash
cmake -S firmware -B firmware/build -DCMAKE_BUILD_TYPE=Release
cmake --build firmware/build -j8 --target rb1_sim
```

Tham số mô hình mặc định và công cụ hiện hành vẫn dùng `model/dynamic.xml`.

Demo sử dụng `simulation/scenarios/bmap1_demo_fixed_camera.xml`. Xem [cách chạy từ gốc repo](../../README.md).

Viewer chạy riêng với runner vật lý. Trên máy hiện tại, renderer dùng RTX 3060; `DRI_PRIME` từ môi trường vẫn được giữ nếu người dùng đã đặt. Tùy chọn `--primitives` của `gui/rb1_gui/cad_viewer.py` vẫn cho phép xem robot bằng các khối đơn giản khi cần.

`--capture-sync` emits a `CAPTURE|...` pose/joint snapshot at each `TOFMAP` timestamp for synchronized offline RGB/ToF rendering. It is disabled by default and never feeds ground truth to the controller. See `evaluation/full_pipeline_results.md` for the full pipeline run.
