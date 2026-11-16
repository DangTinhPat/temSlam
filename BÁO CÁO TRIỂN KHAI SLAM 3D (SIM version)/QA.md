# Kiểm tra và đối chiếu — 14/11/2026

## So với yêu cầu đã duyệt

| Mục tiêu | Bản trước | Bản bàn giao |
|---|---|---|
| Cổng thông tin có cấu trúc | Một bài dài với mục lục bên trái | 5 khu vực công khai, URL riêng và trang đọc từng tài liệu |
| Demo có dữ liệu thật | Ảnh chụp và nút đổi mô tả | Mesh STL thật + 3.198 pose; orbit, zoom, top view, reset, lớp hiển thị, play/pause/seek |
| Công nghệ tương tác | Đổi đoạn văn trong pipeline | Chọn khối, xem input/output, ảnh phóng lớn và giới hạn từng khối |
| Kết quả có đối chiếu | Các chỉ số trải dọc bài | Chọn thí nghiệm, biểu đồ đo, điều kiện, báo cáo nguồn |
| Đăng tài liệu về sau | Sửa HTML thủ công | Đăng nhập, tải file, lưu nháp, xuất bản, sửa metadata, thu hồi xuất bản |
| Tìm và chia sẻ | Liên kết tới Markdown trong repo | Thư viện lọc/tìm không dấu; bộ lọc nằm trong URL; tài liệu có trang riêng |
| Màn hình nhỏ | Bố cục bài báo cáo | Menu thu gọn, viewer thích ứng, cards và công cụ xếp lại theo màn hình |

## Bằng chứng kiểm tra

- **10 backend tests passed:** đăng nhập/CSRF, bản nháp và file riêng tư, xuất bản/thu hồi, bền vững sau tạo lại ứng dụng, loại file và giới hạn dung lượng, Markdown XSS, logout, đường dẫn riêng tư, CSP download, token bất thường và anchor tiếng Việt.
- **6 browser tests passed:** dữ liệu 3D thực; phát/dừng/tua/layers/camera; công nghệ, ảnh và thí nghiệm; tìm/lọc/empty state/đọc tài liệu; đăng nhập–upload–publish–edit–reload–unpublish–logout; mobile và trạng thái mất dữ liệu 3D.
- Chromium desktop **1440×1000**, mobile viewport **390×844**. Kiểm tra không tràn ngang trên 7 route; ảnh full-page đã được chụp.
- Luồng upload dùng server thử `127.0.0.1:8081`, database tạm; không để tài liệu QA trong thư viện thật.
- Kiểm tra **24 liên kết/tài nguyên nội bộ** trong tài liệu đóng gói: không có 404.
- Trang chủ của server bàn giao `127.0.0.1:8080` đã được mở bằng Chromium headless và xác nhận mesh tải xong. `/` và `/admin` trả 200; đường dẫn thử tới file mật khẩu trả 404.
- Kiểm tra cú pháp Python/JavaScript và shell launcher; bộ test không gọi simulator hoặc GPU inference.

## Vòng review và cải thiện

Một reviewer độc lập đọc backend, frontend, test và nguồn dữ liệu, không sửa instance thật. Đã tái hiện và sửa cả bốn phát hiện:

1. Mục lục tài liệu không có đích: sinh ID Unicode, giữ qua sanitizer; 11 anchor kiến trúc trỏ đúng heading.
2. Phóng ảnh chỉ dùng chuột: ảnh được focus bằng bàn phím, Enter/Space mở dialog, Escape đóng. Đã kiểm tra Enter trong browser test.
3. CSP riêng của file download bị policy chung ghi đè: giữ policy sandbox của route.
4. CSRF chứa ký tự ngoài ASCII gây lỗi 500: từ chối bằng 403 trước khi so sánh.

Reviewer chạy lại backend: **10 passed**, không thấy hồi quy tức thời trong các sửa đổi; chấp nhận cho phạm vi triển khai cục bộ đã mô tả.

## Ảnh đối chiếu

Các ảnh được sinh tại `test-results/` (không commit):

- `overview-comparison.jpg`: phần đầu trang cũ và portal mới đặt cạnh nhau.
- `home-desktop.png`, `production-home.png`: trang chủ với mesh thật.
- `explore-desktop.png`, `explore-wireframe.png`: viewer và các chế độ.
- `library-desktop.png`, `admin-desktop.png`: đọc và xuất bản nội dung.
- `mobile--.png`, `mobile--explore.png` cùng các ảnh mobile khác.

## Phạm vi chưa kiểm tra

Chưa đưa lên Internet/HTTPS, chưa kiểm tra Safari/Firefox, thiết bị điện thoại thật hoặc tải đồng thời nhiều người. Mobile QA là viewport giả lập trong Chromium. Video phụ thuộc codec trình duyệt; QA xuất bản end-to-end dùng Markdown, không xác nhận mọi codec video. PDF được tải về để đọc, chưa có trình đọc PDF nhúng.

Demo là phát lại pose camera trên mesh cuối; không chạy SLAM mới hay hiển thị quá trình tích phân TSDF. Website không thay đổi controller, mapper hoặc dữ liệu tham chiếu gốc.

## Cập nhật khả năng đọc — 14/11/2026

- Hạ nền và các panel sang xám xanh tối; chữ sáng dịu, không trắng tuyệt đối. Đồng bộ màu của khung 3D, lưới và marker; giảm ánh sáng trên mesh. Ảnh/biểu đồ nguồn giữ nguyên màu.
- Dùng Noto Sans phục vụ tại máy chủ (có subset tiếng Việt và giấy phép OFL). Tăng cỡ chữ mô tả; nội dung tài liệu 16 px, line-height 1.85 trên desktop, 15 px trên mobile.
- Đo 15 mẫu chữ trên trang chủ, thư viện, tài liệu và thí nghiệm: tỷ lệ tương phản nhỏ nhất 5.98:1. Đây là các mẫu được chọn, không phải chứng nhận accessibility toàn website.
- Chạy lại 6 luồng Chromium: đều qua, không tràn ngang trong viewport mobile 390 px. Kiểm tra Noto Sans đã nạp và hiển thị tiếng Việt trên server LAN.
- Chưa kiểm tra cảm nhận trên màn hình/độ sáng thực tế của người dùng.

## Điều chỉnh theo phản hồi: nền trắng dịu — bản hiện tại

Người dùng muốn nền trắng giảm chói, không muốn nền tối/ám xanh. Đã thay bảng màu toàn website bằng trắng ngà/xám ấm (nền #e9e8e4, panel #f3f2ee), chữ xám đậm và điểm nhấn xanh xám trung tính. Viewer dùng nền xám sáng, mesh xám và quỹ đạo cam. Giữ Noto Sans và kích thước chữ đã cải thiện.

Chạy lại 6 luồng Chromium: đều qua. Kiểm tra mẫu tương phản ở trang chủ, tài liệu và explorer: thấp nhất 4.70:1. Các ảnh desktop/mobile đã được đối chiếu; chưa xác nhận cảm nhận trên màn hình thật của người dùng.

## Điều chỉnh cấu trúc: danh sách báo cáo dùng chung

- Trang chủ lấy các tài liệu đã xuất bản thuộc danh mục Báo cáo từ API, không gắn số liệu/mesh SLAM vào giao diện chung. Thẻ Thêm báo cáo mới mở luồng đăng nhập và tải bài. Chủ đề và số lượng lấy từ dữ liệu, hỗ trợ tìm kiếm không dấu và sắp xếp.
- Menu chung: Báo cáo / Thư viện tài liệu / Quản lý bài đăng. Demo SLAM chỉ liên kết trong bài triển khai SLAM; có đường quay về bài. Trang chủ không tải mesh hoặc quỹ đạo.
- Browser test dùng báo cáo hiệu chuẩn động cơ ngoài chủ đề SLAM: đăng nháp, xuất bản, thấy thẻ mới ở trang chủ, tìm kiếm/đọc, sửa metadata, tải lại, thu hồi xuất bản và xác nhận thẻ biến mất. Không sửa dữ liệu thật.
- Đối chiếu ảnh desktop/mobile của danh sách và thẻ thêm báo cáo. Bài mới dùng trình đọc theo loại file; không tự sinh demo 3D từ tài liệu tải lên.
