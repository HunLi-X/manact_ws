// G1 NavGrasp Web 控制面板 — 前端逻辑
// 模块：路由、状态轮询、日志、命令下发、各视图数据绑定

// ======================================================================
// 状态映射
// ======================================================================
const STATE_COLORS = {
  IDLE:     { dot: 'bg-slate-400',   text: '空闲' },
  WORKING:  { dot: 'bg-cyan-500',    text: '执行中' },
  GRABBING: { dot: 'bg-red-500',     text: '抓取中' },
  MENU:     { dot: 'bg-emerald-500', text: '可放下' },
};

const VIEW_META = {
  grasp:   { title: '目标抓取',   subtitle: '完整抓取任务流程' },
  detect:  { title: '目标识别',   subtitle: '仅显示视觉检测，不触发运动' },
  control: { title: '运动控制',   subtitle: '手动遥控机器人' },
  status:  { title: '系统状态',   subtitle: '话题健康与日志总览' },
  coming:  { title: '更多模块',   subtitle: '敬请期待' },
};

// ======================================================================
// 视图路由（hash 路由，简单可靠）
// ======================================================================
function switchView(name) {
  if (!VIEW_META[name]) name = 'grasp';

  document.querySelectorAll('.view').forEach(el => {
    el.classList.toggle('hidden', el.dataset.view !== name);
  });
  document.querySelectorAll('.nav-item').forEach(el => {
    el.classList.toggle('active', el.dataset.view === name);
  });

  const meta = VIEW_META[name];
  document.getElementById('page-title').textContent = meta.title;
  document.getElementById('page-subtitle').textContent = meta.subtitle;

  if (location.hash !== '#' + name) {
    history.replaceState(null, '', '#' + name);
  }
}

document.querySelectorAll('.nav-item').forEach(el => {
  el.addEventListener('click', e => {
    e.preventDefault();
    switchView(el.dataset.view);
  });
});

window.addEventListener('hashchange', () => {
  switchView(location.hash.replace('#', '') || 'grasp');
});

// 启动时根据 hash 切换
switchView(location.hash.replace('#', '') || 'grasp');


// ======================================================================
// 速度控制（运动控制模块）
// ======================================================================
const speedSlider = document.getElementById('c-speed-slider');
const speedValue = document.getElementById('c-speed-value');
if (speedSlider) {
  speedSlider.addEventListener('input',
    () => speedValue.textContent = (+speedSlider.value).toFixed(2));
}

function getSpeed() {
  return speedSlider ? parseFloat(speedSlider.value) : 0.2;
}


// ======================================================================
// 命令下发
// ======================================================================
async function postCmd(path, body = {}) {
  try {
    const r = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const data = await r.json();
    if (!r.ok) appendLog(data.error || 'error', 'error');
    else if (data && data.ok === false) appendLog(data.error || 'error', 'error');
  } catch (e) {
    appendLog('网络错误: ' + e.message, 'error');
  }
}

// 抓取/放下等任务按钮（data-cmd）
document.querySelectorAll('button[data-cmd]').forEach(btn => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.cmd;
    switch (cmd) {
      case 'search':       postCmd('/api/cmd/search'); break;
      case 'grab':         postCmd('/api/cmd/grab'); break;
      case 'putdown':      postCmd('/api/cmd/putdown'); break;
      case 'turn_putdown': postCmd('/api/cmd/turn_putdown'); break;
      case 'stop':         postCmd('/api/cmd/stop'); break;
    }
  });
});

// 方向盘按钮（data-dcmd）— 运动控制模块
document.querySelectorAll('button[data-dcmd]').forEach(btn => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.dcmd;
    const v = getSpeed();
    const vyaw = v * 2.0;
    switch (cmd) {
      case 'forward':  postCmd('/api/cmd/manual', { vx:  v, vy: 0, vyaw: 0 }); break;
      case 'backward': postCmd('/api/cmd/manual', { vx: -v, vy: 0, vyaw: 0 }); break;
      case 'left':     postCmd('/api/cmd/manual', { vx: 0, vy: 0, vyaw:  vyaw }); break;
      case 'right':    postCmd('/api/cmd/manual', { vx: 0, vy: 0, vyaw: -vyaw }); break;
      case 'stop':     postCmd('/api/cmd/stop'); break;
    }
  });
});


// ======================================================================
// 日志面板（同步写入 grasp 和 status 两个日志框）
// ======================================================================
function appendLog(msg, level = 'info') {
  const ts = new Date().toLocaleTimeString('zh-CN', { hour12: false });
  const text = `[${ts}] ${msg}`;
  ['log-box', 's-log-box'].forEach(id => {
    const box = document.getElementById(id);
    if (!box) return;
    const line = document.createElement('div');
    line.className = `log-line log-${level}`;
    line.textContent = text;
    box.appendChild(line);
    if (box.children.length > 300) box.removeChild(box.firstChild);
    box.scrollTop = box.scrollHeight;
  });
}

function clearLog() {
  ['log-box', 's-log-box'].forEach(id => {
    const box = document.getElementById(id);
    if (box) box.innerHTML = '';
  });
}


// ======================================================================
// 状态轮询（500ms）
// ======================================================================
let lastLogIdx = 0;

function fmtOrDash(v, digits = 2, unit = '') {
  return v != null ? v.toFixed(digits) + unit : '—';
}

function setTxt(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

async function pollState() {
  try {
    const r = await fetch('/api/state?since=' + lastLogIdx);
    if (!r.ok) return;
    const s = await r.json();

    // 顶部全局状态
    const st = STATE_COLORS[s.state] || { dot: 'bg-slate-400', text: s.state };
    const stateDot = document.getElementById('state-dot');
    if (stateDot) stateDot.className = 'state-dot ' + st.dot;
    setTxt('state-label', st.text);
    setTxt('fps-label', 'FPS ' + (s.fps || 0).toFixed(0));
    setTxt('det-label', '检测 ' + (s.det_count || 0));

    // 抓取模块
    setTxt('g-info-state',  s.state);
    setTxt('g-info-target', s.target_class || '—');
    setTxt('g-info-u',      fmtOrDash(s.target_u, 3));
    setTxt('g-info-bbox',   fmtOrDash(s.bbox_max, 2));
    setTxt('g-info-dist',   fmtOrDash(s.distance, 2, ' m'));
    setTxt('g-info-count',  s.det_count != null ? s.det_count : '—');

    // 检测模块
    setTxt('d-fps',    (s.fps || 0).toFixed(1));
    setTxt('d-target', s.target_class || '—');
    setTxt('d-count',  s.det_count != null ? s.det_count : '—');
    setTxt('d-u',      fmtOrDash(s.target_u, 3));
    setTxt('d-bbox',   fmtOrDash(s.bbox_max, 2));
    setTxt('d-dist',   fmtOrDash(s.distance, 2, ' m'));

    // 运动控制模块
    setTxt('c-info-state', s.state);
    setTxt('c-info-mode', s.state === 'WORKING' ? '自动任务' : '手动遥控');

    // 系统状态模块
    setTxt('s-fps',        (s.fps || 0).toFixed(1));
    setTxt('s-det-count',  s.det_count != null ? s.det_count : '—');
    setTxt('s-target',     s.target_class || '—');
    setTxt('s-u',          fmtOrDash(s.target_u, 3));
    setTxt('s-bbox',       fmtOrDash(s.bbox_max, 2));
    setTxt('s-dist',       fmtOrDash(s.distance, 2, ' m'));
    setTxt('s-state',      s.state);

    // 日志追加
    if (s.logs && s.logs.length > 0) {
      for (const entry of s.logs) appendLog(entry.msg, entry.level);
      lastLogIdx = s.log_idx;
    }
  } catch (e) { /* 暂时忽略网络异常 */ }
}

setInterval(pollState, 500);
pollState();
appendLog('Web 面板已连接', 'info');
