# Sửa độ méo và khoảng trống — 2026-11-12

Nguồn: `datasets/raw/tof-20261112-090937-766548`, giữ nguyên.
Kết quả: `datasets/processed/scan3d_fusion/living-room-refined-20261112`.
Đường dẫn tính từ gốc repo. Phiên kết quả liên kết tới keyframe nguồn, nên cần
giữ cả phiên nguồn khi di chuyển hoặc lưu trữ.

Quỹ đạo bản gốc gần trùng odometry. Đối chiếu simulator trên 451 khung sau căn
chỉnh cứng XY, không scale: RMSE 0,277 m, P95 0,571 m, lớn nhất 0,743 m.
Trong phép thử cuVSLAM phát lại **chỉ keyframe**, gate chấp nhận 4/451 pose;
thử này không tái tạo đầy đủ dòng ảnh live nên không dùng tỷ lệ đó làm thống kê
tracking của phiên gốc. Trước đây không lưu cờ gate; phiên live mới lưu
`visual_accepted` để quan sát đúng hành vi này.

Đã thêm ghép bề mặt point-to-plane XY/yaw khi finalize, khởi tạo từ pose hiện
có. Dùng pháp tuyến bề mặt gần đứng, correspondence gần và cùng phương, trọng số
robust, prior odometry, giới hạn hiệu chỉnh và kiểm tra thiếu ràng buộc. Các
khung không ghép được tiếp tục theo odometry cùng correction cuối. Bước này
không đọc CAD, ảnh RGB tái render, pose thật hoặc tuyến đường thiết kế.

Trên cùng dữ liệu, 331/451 khung có hiệu chỉnh được chấp nhận. RMSE giảm xuống
0,099 m, P95 0,193 m, lớn nhất 0,312 m. Ground truth chỉ dùng trong evaluator;
số liệu tại [surface_matching_results.json](surface_matching_results.json).
Đây là lỗi quỹ đạo, không phải sai số mọi bề mặt của mesh.

Luật chặn sàn trước đây giữ vật cản sau 2 khung, không xét bằng chứng sàn ngược
lại. Nay chỉ gỡ nhãn vật cản khi có >=5 khung đo sàn và >=3 lần số khung vật cản;
điểm thấp hơn sàn vẫn chặn. Với cả ghép bề mặt và luật mới, diện tích sàn quan
sát được chấp nhận tăng 17,525 → 25,008 m²; sàn suy ra 2,430 → 2,945 m².
Diện tích tính theo lưới pose ước lượng, không phải tỷ lệ phủ thực tế.

## Kiểm tra

8 kiểm tra floor support, fill và surface matching đã qua. Kiểm tra dữ liệu
thật của phiên xác nhận: 451 pose giữ nguyên cao độ/phương trọng lực; hai lớp
sàn không trùng nhau, không phủ ô blocked hoặc ô đo được điểm thấp hơn sàn.
Đã dựng, xem ảnh kết quả và mở RViz với ba lớp objects/observed/inferred.

```bash
env -u PYTHONPATH /home/dvt/.venvs/deltar/bin/python -m unittest discover -s technology/scan3d_fusion/reconstruction -p 'test_surface_matching.py' -v
env -u PYTHONPATH /home/dvt/.venvs/deltar/bin/python -m unittest discover -s technology/scan3d_fusion/reconstruction -p 'test_*floor*.py' -v
env -u PYTHONPATH MUJOCO_GL=egl /home/dvt/.venvs/deltar/bin/python technology/scan3d_fusion/live/workers.py finalize datasets/processed/scan3d_fusion/living-room-refined-20261112
technology/scan3d_fusion/viewer/run_clay_rviz.sh datasets/processed/scan3d_fusion/living-room-refined-20261112/final_mesh.stl --floor-completion
```

## Giới hạn

Chưa thêm pose graph toàn cục, chưa sửa việc lấy mẫu ảnh realtime để cuVSLAM
theo kịp ×5. Tinh chỉnh hiện chạy khi finalize; mesh trong lúc đang quét vẫn
dùng pose live. Sàn khuất/lỗ lớn không được tự phủ; vật thể và tường vẫn chịu
sai số depth fusion. Luật bằng chứng đếm khung, nên nhiều ảnh cùng sai lệch
không tương đương các phép đo độc lập. Chưa kiểm chứng phần cứng hoặc nhiều cảnh.
