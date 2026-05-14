"""端到端验证：节点管理独立页 + 系统状态恢复纯监控。"""
import importlib.util as u
import threading, time, urllib.request, json

spec = u.spec_from_file_location('m', r'd:/Main/云控制/人形机器人/g1act_ws/src/web_frontend/dev_server.py')
m = u.module_from_spec(spec); spec.loader.exec_module(m)
app = m.create_app()

t = threading.Thread(target=lambda: app.run(host='127.0.0.1', port=18099, threaded=True, use_reloader=False), daemon=True)
t.start()
time.sleep(1.0)

base = 'http://127.0.0.1:18099'

def fetch(url):
    return urllib.request.urlopen(url, timeout=3).read().decode('utf-8', errors='replace')

# 1. HTML 检查
html = fetch(base + '/')
# 侧栏入口
assert 'data-view="nodes"' in html and '节点管理' in html, "missing nodes nav-item"
# 新视图
assert 'data-view="nodes"' in html and 'class="node-cards"' in html, "missing nodes view container"
assert 'data-proc="camera"' in html and 'data-proc="yolo"' in html and 'data-proc="rgbd"' in html, "missing 3 node cards"
# 内联参数表单
for pkey in ('camera_namespace', 'image_topic', 'conf_threshold', 'color_topic', 'depth_topic', 'interval_sec'):
    assert f'data-pkey="{pkey}"' in html, f"missing param input {pkey}"
# 系统状态页应该不含 process-launchers / camera-launcher
status_section_start = html.find('data-view="status"')
status_section_end = html.find('data-view="settings"', status_section_start)
status_section = html[status_section_start:status_section_end]
assert 'camera-launcher' not in status_section, "status view should NOT contain camera-launcher"
assert 'process-launchers' not in status_section, "status view should NOT contain process-launchers"
print(f"[OK] HTML: nodes view + 3 cards + 系统状态已清理")

# 2. CSS 节点管理样式
css = fetch(base + '/static/css/style.css')
for cls in ('.nodes-wrap', '.nodes-banner', '.node-cards', '.node-card', '.node-head', '.node-body', '.node-meta', '.node-params', '.params-title', '.btn-mini'):
    assert cls in css, f"missing CSS {cls}"
print(f"[OK] style.css: 全部节点管理样式齐全")

# 3. JS VIEW_META + saveProcParams
js = fetch(base + '/static/js/app.js')
assert "nodes:    { title: '节点管理'" in js, "missing nodes in VIEW_META"
assert 'function saveProcParams' in js, "missing saveProcParams"
assert 'function loadProcParamsForm' in js, "missing loadProcParamsForm"
assert "name === 'nodes'" in js, "missing nodes view switch hook"
assert "'server':" in js, "missing server icon"
print(f"[OK] app.js: VIEW_META.nodes + saveProcParams + server icon")

# 4. API 路由仍可用
def get(p):
    return json.loads(urllib.request.urlopen(base + p, timeout=3).read().decode())
def post(p, d=None):
    body = json.dumps(d or {}).encode()
    req = urllib.request.Request(base + p, data=body, headers={'Content-Type':'application/json'}, method='POST')
    return json.loads(urllib.request.urlopen(req, timeout=3).read().decode())

s = get('/api/state')
for k in ('camera', 'yolo', 'rgbd'):
    assert k in s
print(f"[OK] /api/state: 3 processes present")

# 启动 + 修改参数 + 状态查询
r = post('/api/process/yolo/start')
assert r.get('ok')
print(f"[OK] /api/process/yolo/start: {r['msg']}")

r = post('/api/process/yolo/params', {'conf_threshold': '0.65'})
assert r.get('ok') and r['params']['conf_threshold'] == '0.65'
print(f"[OK] /api/process/yolo/params: conf={r['params']['conf_threshold']}")

post('/api/process/yolo/stop')
print("\n=== NODES PAGE VERIFIED ===")
