import io
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pytest
from werkzeug.security import generate_password_hash
from app import create_app

@pytest.fixture
def app(tmp_path):
    return create_app({'TESTING': True, 'DATA_DIR': str(tmp_path), 'SECRET_KEY': 'test-only',
                       'ADMIN_HASH': generate_password_hash('test-password-123')})

@pytest.fixture
def client(app):
    return app.test_client()

def login(client):
    csrf = client.get('/api/session').json['csrf']
    response = client.post('/api/login', json={'password': 'test-password-123'}, headers={'X-CSRF-Token': csrf})
    assert response.status_code == 200
    return {'X-CSRF-Token': response.json['csrf']}

def upload(client, headers, published='false', filename='notes.md', body=b'# A new study\nUseful **result**.'):
    return client.post('/api/documents', data={'title': 'New study', 'summary': 'Measured result',
        'category': 'Báo cáo', 'tags': 'SLAM, ToF', 'published': published,
        'file': (io.BytesIO(body), filename)}, headers=headers)

def test_write_requires_login_and_csrf(client):
    csrf = client.get('/api/session').json['csrf']
    assert upload(client, {'X-CSRF-Token': csrf}).status_code == 401
    headers = login(client)
    assert upload(client, {}).status_code == 403
    assert upload(client, headers).status_code == 201

def test_draft_publish_unpublish_and_persistence(client, app):
    headers = login(client)
    result = upload(client, headers)
    assert result.status_code == 201
    key = result.json['id']
    public = app.test_client()
    assert public.get('/api/documents/' + key).status_code == 404
    assert public.get('/files/' + key).status_code == 404
    assert key not in [x['id'] for x in public.get('/api/documents').json]
    assert client.patch('/api/documents/' + key, json={'published': True}, headers=headers).status_code == 200
    assert '<strong>result</strong>' in public.get('/api/documents/' + key).json['html']
    restarted = create_app(app.config).test_client()
    assert restarted.get('/api/documents/' + key).status_code == 200
    assert client.patch('/api/documents/' + key, json={'published': False}, headers=headers).status_code == 200
    assert public.get('/api/documents/' + key).status_code == 404

def test_invalid_upload_and_empty_metadata(client):
    headers = login(client)
    assert upload(client, headers, filename='evil.html').status_code == 400
    assert upload(client, headers, filename='fake.pdf', body=b'not pdf').status_code == 400
    assert upload(client, headers, filename='empty.md', body=b'').status_code == 400
    assert upload(client, headers, filename='bad.md', body=b'\xff').status_code == 400
    assert client.post('/api/documents', data={'title': ' ', 'file': (io.BytesIO(b'hello'),'a.md')}, headers=headers).status_code == 400

def test_markdown_xss_removed(client):
    headers = login(client)
    result = upload(client, headers, 'true', body=b'# Fine\n<script>alert(1)</script>\n[bad](javascript:alert(1))\n<img src=x onerror=alert(1)>')
    html = client.get('/api/documents/' + result.json['id']).json['html']
    assert '<script' not in html and 'javascript:' not in html and 'onerror=' not in html

def test_assets_do_not_expose_private_files(client):
    for path in ['/instance/portal.sqlite3','/app.py','/files/../../app.py','/vendor/../../app.py','/.venv/pyvenv.cfg']:
        assert client.get(path).status_code == 404

def test_bad_login_and_logout(client):
    csrf = client.get('/api/session').json['csrf']
    assert client.post('/api/login', json={'password':'wrong'},headers={'X-CSRF-Token':csrf}).status_code == 401
    headers=login(client)
    assert client.post('/api/logout',headers=headers).status_code==200
    assert upload(client,headers).status_code in (401,403)

def test_upload_limit(client, app):
    headers=login(client)
    app.config['MAX_CONTENT_LENGTH']=1024
    assert upload(client,headers,body=b'x'*2048).status_code==413

def test_malformed_csrf_is_rejected(client):
    client.get('/api/session')
    assert client.post('/api/logout', headers={'X-CSRF-Token':'é'}).status_code == 403

def test_download_keeps_sandbox_policy(client):
    headers=login(client)
    key=upload(client,headers,'true').json['id']
    assert client.get('/files/'+key).headers['Content-Security-Policy'] == "default-src 'none'; sandbox"

def test_article_toc_resolves_to_unicode_heading_ids(client):
    import re
    html=client.get('/api/documents/architecture').json['html']
    anchors=re.findall(r'href="#([^\"]+)"',html)
    assert anchors
    ids=set(re.findall(r'id="([^\"]+)"',html))
    assert set(anchors) <= ids
