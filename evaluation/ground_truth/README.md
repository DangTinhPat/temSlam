# Ground truth output

Chỉ recorder/evaluator được đọc trạng thái thật từ MuJoCo. Ghi pose/quỹ đạo thật tại đây, tách biệt hoàn toàn khỏi sensor stream và runtime SLAM. Ghép cặp estimate với ground truth chỉ làm sau khi thuật toán đã xuất kết quả.
