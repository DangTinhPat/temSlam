import {defineConfig} from '@playwright/test';
export default defineConfig({testDir:'./tests',testMatch:'*.spec.js',workers:1,timeout:45000,
  use:{baseURL:'http://127.0.0.1:8081',viewport:{width:1440,height:1000},launchOptions:{args:['--enable-unsafe-swiftshader']},screenshot:'only-on-failure'},
  webServer:{command:'env -u PYTHONPATH .venv/bin/python tests/serve_test.py',url:'http://127.0.0.1:8081',reuseExistingServer:false,timeout:15000}});
