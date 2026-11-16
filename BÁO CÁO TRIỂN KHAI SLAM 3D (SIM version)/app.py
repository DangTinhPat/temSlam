"""Local-first RB research portal. Run with python app.py; admin setup via --set-password."""
import argparse
from datetime import timedelta
import getpass
import hashlib
import json
import os
from pathlib import Path
import secrets
import sqlite3
import time
import uuid

import bleach
from flask import Flask, abort, g, jsonify, request, send_file, send_from_directory, session
import markdown
from markdown.extensions.toc import slugify_unicode
from werkzeug.exceptions import HTTPException
from werkzeug.security import check_password_hash, generate_password_hash

ROOT = Path(__file__).resolve().parent
CATEGORIES = ('Báo cáo', 'Công nghệ', 'Hướng dẫn', 'Dữ liệu')
MIMES = {'md': 'text/markdown; charset=utf-8', 'pdf': 'application/pdf', 'png': 'image/png',
         'jpg': 'image/jpeg', 'jpeg': 'image/jpeg', 'webp': 'image/webp', 'mp4': 'video/mp4', 'webm': 'video/webm'}


def create_app(config=None):
    app = Flask(__name__, static_folder=str(ROOT / 'static'))
    app.config.update(DATA_DIR=os.environ.get('RB_PORTAL_DATA', str(ROOT / 'instance')),
                      MAX_CONTENT_LENGTH=64 * 1024 * 1024, SESSION_COOKIE_HTTPONLY=True,
                      SESSION_COOKIE_SAMESITE='Lax', SESSION_COOKIE_SECURE=os.environ.get('RB_PORTAL_HTTPS') == '1',
                      PERMANENT_SESSION_LIFETIME=timedelta(hours=8))
    app.config.update(config or {})
    data = Path(app.config['DATA_DIR'])
    data.mkdir(parents=True, exist_ok=True)
    (data / 'uploads').mkdir(exist_ok=True)
    if not app.config.get('SECRET_KEY'):
        keyfile = data / 'session.key'
        try:
            fd = os.open(keyfile, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError:
            pass
        else:
            with os.fdopen(fd, 'w') as f:
                f.write(secrets.token_hex(32))
        app.config['SECRET_KEY'] = keyfile.read_text().strip()

    def db():
        if 'db' not in g:
            g.db = sqlite3.connect(data / 'portal.sqlite3', timeout=15)
            g.db.row_factory = sqlite3.Row
        return g.db

    @app.teardown_appcontext
    def close_db(_error):
        connection = g.pop('db', None)
        if connection is not None:
            connection.close()

    with app.app_context():
        db().executescript('''
        CREATE TABLE IF NOT EXISTS documents (
          id TEXT PRIMARY KEY, title TEXT NOT NULL, summary TEXT NOT NULL, category TEXT NOT NULL,
          tags TEXT NOT NULL, kind TEXT NOT NULL, filename TEXT NOT NULL, original_name TEXT NOT NULL,
          published INTEGER NOT NULL DEFAULT 0, created TEXT NOT NULL, source TEXT NOT NULL DEFAULT 'upload');
        CREATE TABLE IF NOT EXISTS settings (key TEXT PRIMARY KEY, value TEXT NOT NULL);
        CREATE TABLE IF NOT EXISTS login_failures (address TEXT NOT NULL, at REAL NOT NULL);
        ''')
        manifest = ROOT / 'content' / 'catalog.json'
        if manifest.exists():
            for item in json.loads(manifest.read_text()):
                db().execute('INSERT OR IGNORE INTO documents VALUES (?,?,?,?,?,?,?,?,?,?,?)',
                             (item['id'], item['title'], item['summary'], item['category'], json.dumps(item['tags']),
                              'md', item['id'] + '.md', item['id'] + '.md', 1, item['date'], 'curated'))
        db().commit()

    def admin_hash():
        if app.config.get('ADMIN_HASH'):
            return app.config['ADMIN_HASH']
        row = db().execute("SELECT value FROM settings WHERE key='password_hash'").fetchone()
        return row['value'] if row else ''

    def is_admin():
        current = admin_hash()
        return bool(current and session.get('admin') == hashlib.sha256(current.encode()).hexdigest())

    def require_admin():
        if not is_admin():
            abort(401, description='Vui lòng đăng nhập quản trị.')

    @app.before_request
    def csrf_protection():
        if request.method in ('POST', 'PATCH', 'PUT', 'DELETE'):
            given = request.headers.get('X-CSRF-Token', '')
            expected = session.get('csrf', '')
            if not expected or not given.isascii() or not secrets.compare_digest(given, expected):
                abort(403, description='Phiên làm việc đã thay đổi. Tải lại trang rồi thử lại.')

    @app.after_request
    def security_headers(response):
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['X-Frame-Options'] = 'SAMEORIGIN'
        response.headers['Referrer-Policy'] = 'same-origin'
        response.headers.setdefault('Content-Security-Policy', "default-src 'self'; script-src 'self' 'sha256-jBZ1cnU+/wLpocw4dWcqLx+ZgNGt2nyZD7XAKepXK/4='; style-src 'self' 'unsafe-inline'; img-src 'self' data:; object-src 'none'; frame-src 'self'; base-uri 'none'; form-action 'self'")
        if request.path.startswith(('/api/', '/files/')):
            response.headers['Cache-Control'] = 'no-store'
        return response

    @app.errorhandler(HTTPException)
    def http_error(error):
        messages = {413: 'Tệp quá lớn. Giới hạn mỗi lần tải lên là 64 MB.'}
        message = 'Không tìm thấy nội dung hoặc nội dung chưa được xuất bản.' if error.code == 404 else messages.get(error.code, error.description)
        if request.path.startswith('/api/'):
            return jsonify(error=message), error.code
        return message, error.code

    @app.get('/')
    @app.get('/explore')
    @app.get('/technology')
    @app.get('/experiments')
    @app.get('/library')
    @app.get('/library/<doc_id>')
    @app.get('/admin')
    def index(doc_id=None):
        if doc_id:
            document(doc_id)
        return send_file(ROOT / 'index.html')

    @app.get('/legacy')
    def legacy():
        return send_file(ROOT / 'report-legacy.html')

    @app.get('/Images_attachments/<path:name>')
    def illustrations(name):
        return send_from_directory(ROOT / 'Images_attachments', name)

    @app.get('/vendor/<path:name>')
    def vendor(name):
        allowed = {'build/three.module.js', 'build/three.core.js', 'examples/jsm/controls/OrbitControls.js',
                   'examples/jsm/loaders/STLLoader.js'}
        if name not in allowed:
            abort(404)
        return send_from_directory(ROOT / 'node_modules' / 'three', name)

    @app.get('/api/session')
    def session_info():
        if 'csrf' not in session:
            session['csrf'] = secrets.token_hex(32)
        return jsonify(authenticated=is_admin(), configured=bool(admin_hash()), csrf=session['csrf'])

    @app.post('/api/login')
    def login():
        payload = request.get_json(silent=True)
        if not isinstance(payload, dict) or not isinstance(payload.get('password'), str) or len(payload['password']) > 256:
            abort(400, description='Mật khẩu không hợp lệ.')
        address = request.remote_addr or 'local'
        now = time.time()
        db().execute('DELETE FROM login_failures WHERE at < ?', (now - 900,))
        count = db().execute('SELECT COUNT(*) FROM login_failures WHERE address=?', (address,)).fetchone()[0]
        db().commit()
        if count >= 8:
            abort(429, description='Đã thử quá nhiều lần. Vui lòng đợi 15 phút.')
        hashed = admin_hash()
        if not hashed or not check_password_hash(hashed, payload['password']):
            db().execute('INSERT INTO login_failures VALUES (?,?)', (address, now))
            db().commit()
            abort(401, description='Mật khẩu không đúng hoặc chưa thiết lập tài khoản.')
        session.clear()
        session.permanent = True
        session['admin'] = hashlib.sha256(hashed.encode()).hexdigest()
        session['csrf'] = secrets.token_hex(32)
        db().execute('DELETE FROM login_failures WHERE address=?', (address,))
        db().commit()
        return session_info()

    @app.post('/api/logout')
    def logout():
        session.clear()
        return session_info()

    def public_row(row):
        return {**{k: row[k] for k in ('id', 'title', 'summary', 'category', 'kind', 'created', 'source')},
                'tags': json.loads(row['tags']), 'published': bool(row['published'])}

    def document(doc_id):
        row = db().execute('SELECT * FROM documents WHERE id=?', (doc_id,)).fetchone()
        if row is None or (not row['published'] and not is_admin()):
            abort(404)
        return row

    def file_path(row):
        return (ROOT / 'content' if row['source'] == 'curated' else data / 'uploads') / row['filename']

    @app.get('/api/documents')
    def documents():
        include_drafts = request.args.get('admin') == '1'
        if include_drafts:
            require_admin()
        rows = db().execute('SELECT * FROM documents ' + ('' if include_drafts else 'WHERE published=1 ') + 'ORDER BY created DESC, title').fetchall()
        return jsonify([public_row(row) for row in rows])

    @app.get('/api/documents/<doc_id>')
    def detail(doc_id):
        row = document(doc_id)
        output = public_row(row)
        output['file_url'] = '/files/' + doc_id
        if row['kind'] == 'md':
            text = file_path(row).read_text(encoding='utf-8')
            tags = set(bleach.sanitizer.ALLOWED_TAGS) | {'h1','h2','h3','h4','h5','h6','p','pre','hr','br','img','table','thead','tbody','tr','th','td','del','span'}
            output['html'] = bleach.clean(markdown.markdown(text, extensions=['tables','fenced_code','toc'], extension_configs={'toc': {'slugify': slugify_unicode}}),
                tags=tags, attributes={'a':['href','title'], 'img':['src','alt','title'], 'code':['class'], **{tag:['id'] for tag in ('h1','h2','h3','h4','h5','h6')}},
                protocols=['https','http','mailto'], strip=True)
        return jsonify(output)

    @app.get('/files/<doc_id>')
    def download(doc_id):
        row = document(doc_id)
        response = send_file(file_path(row), mimetype=MIMES[row['kind']],
                            as_attachment=row['kind'] in ('md', 'pdf') or request.args.get('download') == '1',
                            download_name=row['original_name'], conditional=True)
        response.headers['Content-Security-Policy'] = "default-src 'none'; sandbox"
        return response

    def metadata(payload, existing=None):
        result = dict(existing or {})
        for key, limit in [('title', 160), ('summary', 600), ('category', 40)]:
            value = payload.get(key, result.get(key, 'Báo cáo' if key == 'category' else ''))
            if not isinstance(value, str) or len(value.strip()) > limit or (key != 'summary' and not value.strip()):
                abort(400, description='Tiêu đề, mô tả hoặc danh mục không hợp lệ.')
            result[key] = value.strip()
        if result['category'] not in CATEGORIES:
            abort(400, description='Danh mục không hợp lệ.')
        tags = payload.get('tags', result.get('tags', ''))
        if not isinstance(tags, str) or len(tags) > 200:
            abort(400, description='Nhãn cần ngắn hơn 200 ký tự.')
        result['tags'] = json.dumps([x.strip() for x in tags.split(',') if x.strip()][:8], ensure_ascii=False)
        published = payload.get('published', result.get('published', False))
        if published not in (True, False, 'true', 'false', 0, 1):
            abort(400, description='Trạng thái xuất bản không hợp lệ.')
        result['published'] = int(published is True or published == 'true' or published == 1)
        return result

    @app.post('/api/documents')
    def create_document():
        require_admin()
        fields = metadata(request.form)
        file = request.files.get('file')
        if not file or not file.filename:
            abort(400, description='Chọn một tệp để tải lên.')
        kind = Path(file.filename).suffix.lower().lstrip('.')
        if kind not in MIMES:
            abort(400, description='Hỗ trợ Markdown, PDF, PNG, JPG, WebP, MP4 và WebM.')
        head = file.stream.read(32)
        file.stream.seek(0)
        valid = bool(head)
        signatures = {'pdf': head.startswith(b'%PDF-'), 'png': head.startswith(b'\x89PNG\r\n\x1a\n'),
                      'jpg': head.startswith(b'\xff\xd8\xff'), 'jpeg': head.startswith(b'\xff\xd8\xff'),
                      'webp': head.startswith(b'RIFF') and head[8:12] == b'WEBP',
                      'mp4': head[4:8] == b'ftyp', 'webm': head.startswith(b'\x1aE\xdf\xa3')}
        if kind == 'md':
            raw = file.read(2 * 1024 * 1024 + 1)
            try:
                valid = valid and len(raw) <= 2 * 1024 * 1024 and bool(raw.decode('utf-8').strip()) and b'\0' not in raw
            except UnicodeDecodeError:
                valid = False
            file.stream.seek(0)
        else:
            valid = valid and signatures[kind]
        if not valid:
            abort(400, description='Nội dung tệp không đúng định dạng, rỗng hoặc Markdown vượt 2 MB.')
        key = uuid.uuid4().hex
        filename = key + '.' + kind
        target = data / 'uploads' / filename
        try:
            file.save(target)
            db().execute('INSERT INTO documents VALUES (?,?,?,?,?,?,?,?,?,?,?)',
                (key, fields['title'], fields['summary'], fields['category'], fields['tags'], kind, filename,
                 Path(file.filename).name, fields['published'], time.strftime('%Y-%m-%d'), 'upload'))
            db().commit()
        except Exception:
            target.unlink(missing_ok=True)
            raise
        return jsonify(id=key), 201

    @app.patch('/api/documents/<doc_id>')
    def update_document(doc_id):
        require_admin()
        row = document(doc_id)
        payload = request.get_json(silent=True)
        if not isinstance(payload, dict):
            abort(400)
        values = dict(row)
        values['tags'] = ', '.join(json.loads(row['tags']))
        fields = metadata(payload, values)
        db().execute('UPDATE documents SET title=?,summary=?,category=?,tags=?,published=? WHERE id=?',
                     (fields['title'],fields['summary'],fields['category'],fields['tags'],fields['published'],doc_id))
        db().commit()
        return jsonify(public_row(document(doc_id)))

    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--set-password', action='store_true')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--host', default='127.0.0.1')
    args = parser.parse_args()
    app = create_app()
    if args.set_password:
        password = getpass.getpass('Mật khẩu quản trị mới (ít nhất 12 ký tự): ')
        if len(password) < 12 or len(password) > 256 or password != getpass.getpass('Nhập lại mật khẩu: '):
            raise SystemExit('Mật khẩu cần 12–256 ký tự và hai lần nhập giống nhau.')
        with sqlite3.connect(Path(app.config['DATA_DIR']) / 'portal.sqlite3') as connection:
            connection.execute("INSERT OR REPLACE INTO settings VALUES ('password_hash',?)", (generate_password_hash(password),))
        print('Đã lưu mật khẩu quản trị. Các phiên đăng nhập cũ đã hết hiệu lực.')
        return
    from waitress import serve
    print(f'RB Research Portal: http://{args.host}:{args.port}', flush=True)
    serve(app, host=args.host, port=args.port)


if __name__ == '__main__':
    main()
