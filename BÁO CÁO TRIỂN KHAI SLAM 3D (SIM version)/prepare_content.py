"""Package explicitly selected project materials; never traverses/serves the repo."""
import json
from pathlib import Path
import re
import shutil

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
SOURCES = [
 ('report', ROOT / 'BÁO CÁO TRIỂN KHAI SLAM 3D (SIM version).md', 'Triển khai SLAM 3D trong mô phỏng', 'Báo cáo', 'Toàn bộ hành trình từ cảm biến thưa đến bản đồ bề mặt: kiến trúc, kết quả và giới hạn.', ['SLAM','MuJoCo']),
 ('architecture', REPO / 'technology/scan3d_fusion/README.md', 'Bên trong pipeline RGB–ToF', 'Công nghệ', 'DELTAR, cuVSLAM, odometry và TSDF phối hợp như thế nào?', ['DELTAR','TSDF']),
 ('status', REPO / 'technology/scan3d_fusion/CURRENT_STATUS.md', 'Trạng thái & giới hạn công nghệ', 'Công nghệ', 'Những gì hệ thống đã làm được, các giả thiết và phần chưa kiểm chứng.', ['Prototype','Giới hạn']),
 ('room-results', REPO / 'evaluation/room_dense_results.md', 'Thêm góc nhìn, tăng hỗ trợ bề mặt', 'Báo cáo', 'So sánh 912 và 3.198 khung trên cùng phiên dữ liệu đã ghi.', ['Reconstruction','Đánh giá']),
 ('floor-analysis', REPO / 'evaluation/uneven_floor_diagnosis.md', 'Vì sao sàn còn lồi lõm?', 'Báo cáo', 'Phân rã sai số depth và pose, cùng các thí nghiệm đã loại.', ['Depth','Sai số']),
 ('live-guide', REPO / 'technology/scan3d_fusion/live/README.md', 'Chạy một phiên quét mô phỏng', 'Hướng dẫn', 'Khởi động, theo dõi phiên chạy và đọc dữ liệu đầu ra.', ['MuJoCo','Thực hành']),
 ('navigation', REPO / 'technology/scan3d_fusion/navigation/README.md', 'Lộ trình localization & navigation', 'Công nghệ', 'Thiết kế giai đoạn tiếp theo trên bản đồ đã quét. Chưa triển khai end-to-end.', ['Navigation','Thiết kế'])]


def main():
    content = ROOT / 'content'
    assets = ROOT / 'static' / 'articles'
    demo = ROOT / 'static' / 'demo'
    for directory in [content, assets, demo]:
        directory.mkdir(parents=True, exist_ok=True)
    lookup = {path.resolve(): key for key, path, *_ in SOURCES}
    catalog = []
    for key, path, title, category, summary, tags in SOURCES:
        if not path.exists():
            raise SystemExit(f'Missing source: {path}')
        def rewrite(match):
            image, label, target = match.groups()
            if target.startswith(('https://','http://','#','mailto:')):
                return match.group(0)
            target = target.split('#')[0]
            resolved = (path.parent / target).resolve()
            if resolved in lookup and not image:
                return f'[{label}](/library/{lookup[resolved]})'
            if resolved.is_file() and resolved.is_relative_to(REPO) and resolved.suffix.lower() in ('.png','.jpg','.jpeg','.webp'):
                name = key + '-' + resolved.name
                shutil.copyfile(resolved, assets / name)
                return f'{image}[{label}](/static/articles/{name})'
            return label + (f' — `{target}`' if not image else ' (ảnh nguồn chưa đóng gói)')
        text = re.sub(r'(!?)\[([^\]]*)\]\(([^)]+)\)', rewrite, path.read_text())
        (content / f'{key}.md').write_text(text)
        catalog.append(dict(id=key,title=title,category=category,summary=summary,tags=tags,date='2026-11-13' if key=='navigation' else '2026-11-12'))
    (content / 'catalog.json').write_text(json.dumps(catalog, ensure_ascii=False, indent=2))
    source = REPO / 'datasets/processed/scan3d_fusion/room-floor-preserved-20261112'
    if source.exists():
        shutil.copyfile(source / 'final_mesh.stl', demo / 'room.stl')
        seq = json.loads((source / 'live_sequence.json').read_text())
        frames = [{'t': f['time_s'], 'p': [round(f['camera_to_world'][i][3],5) for i in range(3)],
                   'd': [round(f['camera_to_world'][i][2],5) for i in range(3)]} for f in seq['frames']]
        (demo / 'trajectory.json').write_text(json.dumps({'frames':frames,'pose_source':seq['pose_source'],
            'depth_source':seq['depth_source'],'name':'room-floor-preserved-20261112'},separators=(',',':')))
        print(f'Packaged real mesh and {len(frames)} camera poses.')
    else:
        print('Reference dataset unavailable. Explorer will show an explicit unavailable state.')
    print(f'Packaged {len(catalog)} documents. Uploaded library data left untouched.')

if __name__ == '__main__':
    main()
