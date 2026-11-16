# RB Lab — cổng thông tin dự án

Trang chủ là danh sách báo cáo dùng chung, có tìm kiếm, lọc chủ đề, sắp xếp và thẻ Thêm báo cáo mới. Báo cáo đã xuất bản tự xuất hiện từ database. Thư viện giữ toàn bộ báo cáo/tài liệu; quản trị hỗ trợ đăng và cập nhật metadata. Demo 3D, công nghệ và đối chiếu SLAM nằm trong bài triển khai SLAM tương ứng. Flask + SQLite; giao diện JavaScript/CSS thuần; Three.js phục vụ từ máy chủ, không dùng CDN khi chạy.

## Chạy trên máy này

Từ thư mục gốc repo:

```bash
'BÁO CÁO TRIỂN KHAI SLAM 3D (SIM version)/run.sh'
```

Mở **http://127.0.0.1:8080**, quản trị tại **http://127.0.0.1:8080/admin**.

Tài khoản quản trị cục bộ đã được tạo cho bản bàn giao. Mật khẩu ngẫu nhiên nằm trong `instance/admin-password.txt` (quyền 0600, bị loại khỏi Git, không được web server phục vụ). Mở file này trên máy để lấy mật khẩu. Website chỉ có một tài khoản quản trị; không yêu cầu tên đăng nhập.

Đổi/thiết lập mật khẩu qua terminal, nhập ẩn:

```bash
cd 'BÁO CÁO TRIỂN KHAI SLAM 3D (SIM version)'
env -u PYTHONPATH .venv/bin/python app.py --set-password
```

Lệnh cập nhật mật khẩu được băm trong SQLite và vô hiệu hóa các phiên đăng nhập cũ. File `admin-password.txt` là thông tin bàn giao ban đầu, không tự đổi theo mật khẩu mới; có thể xóa file đó sau khi đã đổi mật khẩu.

## Cài trên máy mới

Yêu cầu Python 3.10+ và Node/npm để cài Three.js. Chạy từ thư mục website:

```bash
python3 -m venv .venv
env -u PYTHONPATH .venv/bin/python -m pip install -r requirements.txt
npm ci --omit=dev
env -u PYTHONPATH .venv/bin/python prepare_content.py
env -u PYTHONPATH .venv/bin/python app.py --set-password
./run.sh
```

`prepare_content.py` đóng gói 7 tài liệu được chọn và ảnh liên quan từ repo. Nếu có dataset `datasets/processed/scan3d_fusion/room-floor-preserved-20261112`, script chép mesh và xuất pose camera thành `static/demo/`. Dataset lớn và `static/demo/` không nằm trong Git: khi triển khai, phải mang theo thư mục demo đã đóng gói hoặc chạy script trên máy có dataset. Khi thiếu dữ liệu/WebGL, trình xem báo rõ trạng thái và dẫn sang báo cáo hình ảnh.

Không cần MuJoCo, ROS, CUDA hay môi trường DELTAR để chạy website sau khi đã đóng gói nội dung. Các đường dẫn runtime chỉ tới tài nguyên trong website; server không mở toàn bộ repository.

## Đăng thêm tài liệu

1. Từ trang chủ, bấm thẻ **Thêm báo cáo mới**, đăng nhập nếu chưa có phiên.
2. Chọn file, nhập tiêu đề, mô tả và nhãn. Giữ danh mục **Báo cáo** để bài xuất hiện trên trang chủ; các danh mục khác nằm trong **Thư viện tài liệu**.
3. Chọn **Lưu bản nháp** để xem trước hoặc **Xuất bản công khai**.
4. Tải lại trang chủ để thấy thẻ báo cáo mới; chọn thẻ để đọc bài.
5. Dùng **Chỉnh thông tin** để cập nhật metadata. Dùng **Thu hồi xuất bản** để ẩn tài liệu mà vẫn giữ file.

Hỗ trợ Markdown UTF-8 (tối đa 2 MB), PDF, PNG/JPG/WebP và MP4/WebM (mỗi request tối đa 64 MB, gồm cả metadata). Markdown đọc ngay trên website; ảnh có phóng lớn; video dùng trình phát của trình duyệt; PDF tải về để đọc. Codec video phải được trình duyệt hỗ trợ.

File đã tải được giữ nguyên. Nếu cần phiên bản nội dung mới, đăng tài liệu mới rồi thu hồi bản cũ. Markdown tải lên nên dùng đường dẫn ảnh/tài liệu đã công bố trong portal; bản này chưa có trình soạn thảo ảnh đính kèm theo bài, tài khoản nhiều người, tìm kiếm toàn văn, hay quản lý phiên bản nội dung.

## Dữ liệu và sao lưu

- `instance/portal.sqlite3`: metadata, trạng thái xuất bản, mật khẩu băm.
- `instance/uploads/`: file tải lên; bản nháp không có URL công khai truy cập được.
- `instance/session.key`: khóa phiên đăng nhập; giữ bí mật.
- `content/`: bản đóng gói tài liệu gốc.
- `static/demo/`: mesh thực và quỹ đạo đã xuất.
- `report-legacy.html`: giao diện báo cáo ban đầu để đối chiếu.

Dừng server và sao lưu toàn bộ `instance/` cùng `content/`, `static/` khi chuyển máy. Không commit hoặc phục vụ công khai `instance/`. Chạy lại `prepare_content.py` chỉ làm mới bản đóng gói tài liệu gốc và demo; không đụng file upload hoặc metadata đã biên tập trong database.

## Kiểm tra

```bash
env -u PYTHONPATH .venv/bin/python -m pip install -r requirements-dev.txt
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_app.py -q
npm ci
npx playwright install chromium
npm run test:ui
```

Kiểm tra trình duyệt dùng Chromium headless với server thử riêng tại `127.0.0.1:8081`, database tạm và mật khẩu chỉ dành cho test. Không sửa thư viện thật. Ảnh desktop/mobile lưu trong `test-results/`. Xem `QA.md` để biết phạm vi đã kiểm tra.

## Đưa lên Internet

Bản bàn giao chạy loopback bằng Waitress. Trước khi mở công khai, đặt sau reverse proxy HTTPS và chạy với `RB_PORTAL_HTTPS=1` để cookie chỉ truyền qua HTTPS. Dùng storage bền vững cho `instance/`; giới hạn upload ở proxy phải phù hợp 64 MB. Đặt mật khẩu quản trị riêng cho máy triển khai. Không dùng test server hoặc bật debug.

Có thể chọn cổng khác: `./run.sh --port 8090`. Biến `RB_PORTAL_DATA` đổi thư mục lưu dữ liệu. Website này dành cho một tiến trình server và một quản trị viên; chưa triển khai public hosting hay kiểm thử tải nhiều người.

## Nguồn kỹ thuật

- [Flask: xử lý tải file](https://flask.palletsprojects.com/en/stable/patterns/fileuploads/).
- [Three.js: tài liệu thư viện 3D](https://threejs.org/docs/).
