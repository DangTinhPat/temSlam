"""Disposable loopback server for browser tests; never uses production instance."""
import sys
import tempfile
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import create_app
from werkzeug.security import generate_password_hash
from waitress import serve
with tempfile.TemporaryDirectory(prefix='rb-portal-ui-') as directory:
    app=create_app({'DATA_DIR':directory,'SECRET_KEY':'test-server-only','ADMIN_HASH':generate_password_hash('browser-test-only-password')})
    serve(app, host='127.0.0.1', port=8081)
