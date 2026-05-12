// ======================================================================
// G1 NavGrasp Web 控制面板 — 前端逻辑
// 模块：路由、状态轮询、日志、命令下发、各视图数据绑定、工作流可视化
// ======================================================================

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
  grasp:    { title: '目标抓取',   subtitle: '完整抓取任务流程' },
  detect:   { title: '目标识别',   subtitle: '仅显示视觉检测，不触发运动' },
  control:  { title: '运动控制',   subtitle: '手动遥控机器人' },
  status:   { title: '系统状态',   subtitle: '话题健康与日志总览' },
  settings: { title: '系统设置',   subtitle: '运行时参数热更新 + 界面偏好' },
  coming:   { title: '更多模块',   subtitle: '敬请期待' },
};

// ======================================================================
// 视图路由（hash 路由）
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

switchView(location.hash.replace('#', '') || 'grasp');


// ======================================================================
// 速度滑块（运动控制）
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
    if (!r.ok || (data && data.ok === false)) {
      appendLog(data.error || 'error', 'error');
    }
  } catch (e) {
    appendLog('网络错误: ' + e.message, 'error');
  }
}

// 抓取/放下等任务按钮（data-cmd）
document.querySelectorAll('[data-cmd]').forEach(btn => {
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
document.querySelectorAll('[data-dcmd]').forEach(btn => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.dcmd;
    const v = getSpeed();
    const vyaw = v * 2.0;
    let vx = 0, vy = 0, vyawOut = 0;
    switch (cmd) {
      case 'forward':  vx =  v; break;
      case 'backward': vx = -v; break;
      case 'left':     vyawOut =  vyaw; break;
      case 'right':    vyawOut = -vyaw; break;
      case 'stop':     postCmd('/api/cmd/stop'); return;
    }
    postCmd('/api/cmd/manual', { vx, vy, vyaw: vyawOut });
    // 立即更新速度映射显示（乐观更新）
    setTxt('c-vx',   vx.toFixed(2));
    setTxt('c-vy',   vy.toFixed(2));
    setTxt('c-vyaw', vyawOut.toFixed(2));
  });
});


// ======================================================================
// 日志面板
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
// 工作流步骤条可视化
// ======================================================================
// 根据后端状态 + 距离/对齐状况推断当前步骤
function inferWorkflowStep(s) {
  if (s.state === 'IDLE')     return 'IDLE';
  if (s.state === 'GRABBING') return 'GRAB';
  if (s.state === 'MENU')     return 'DONE';

  if (s.state === 'WORKING') {
    // 没有检测到目标 → 搜索中
    if (s.target_u == null) return 'SEARCH';
    // u 偏离中心 > 0.1 → 对齐中
    if (Math.abs(s.target_u - 0.5) > 0.1) return 'ALIGN';
    // 居中但未到达 → 接近中
    return 'APPROACH';
  }
  return 'IDLE';
}

// 更新工作流步骤条（active + done 状态）
function updateWorkflow(currentStep) {
  const order = ['SEARCH', 'ALIGN', 'APPROACH', 'GRAB', 'DONE'];
  const idx = order.indexOf(currentStep);
  document.querySelectorAll('.wf-step').forEach(el => {
    const step = el.dataset.step;
    const i = order.indexOf(step);
    el.classList.remove('active', 'done');
    if (i < 0) return;
    if (currentStep === 'IDLE') return;
    if (i < idx) el.classList.add('done');
    else if (i === idx) el.classList.add('active');
  });
}


// ======================================================================
// 健康度环形可视化
// ======================================================================
const RING_CIRC = 2 * Math.PI * 40; // r=40

function updateHealthRing(percent) {
  const ring = document.getElementById('health-ring-fill');
  if (!ring) return;
  const clamped = Math.max(0, Math.min(100, percent));
  const offset = RING_CIRC * (1 - clamped / 100);
  ring.style.strokeDashoffset = offset;
  // 颜色分级
  let color = '#10B981';
  if (clamped < 40) color = '#EF4444';
  else if (clamped < 70) color = '#F59E0B';
  ring.style.stroke = color;
  setTxt('health-percent', Math.round(clamped) + '%');
}

// 简化健康度计算：FPS + 检测 + 任务 三因子
function computeHealth(s) {
  let score = 0;
  // FPS 占 40%
  const fps = s.fps || 0;
  score += Math.min(fps / 25, 1) * 40;
  // 检测可用 占 30%
  if (s.det_count != null) score += 30;
  // 任务状态非 None 占 30%
  if (s.state && s.state !== 'UNKNOWN') score += 30;
  return score;
}


// ======================================================================
// 工具函数
// ======================================================================
function fmtOrDash(v, digits = 2, unit = '') {
  return v != null ? v.toFixed(digits) + unit : '—';
}

function setTxt(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

function setBarPct(id, pct) {
  const bar = document.getElementById(id);
  if (!bar) return;
  const clamped = Math.max(0, Math.min(100, pct));
  bar.style.width = clamped + '%';
}


// ======================================================================
// 状态轮询（500ms）
// ======================================================================
let lastLogIdx = 0;
let connFailCount = 0;

async function pollState() {
  try {
    const r = await fetch('/api/state?since=' + lastLogIdx);
    if (!r.ok) { connFailCount++; return; }
    connFailCount = 0;
    const s = await r.json();

    updateConnIndicator(true, s.mock === true);

    // ----- 顶部全局状态 -----
    const st = STATE_COLORS[s.state] || { dot: 'bg-slate-400', text: s.state };
    const stateDot = document.getElementById('state-dot');
    if (stateDot) stateDot.className = 'state-dot ' + st.dot;
    setTxt('state-label', st.text);
    setTxt('fps-label', 'FPS ' + (s.fps || 0).toFixed(0));
    setTxt('det-label', '检测 ' + (s.det_count || 0));

    // ----- 视图 1：目标抓取 -----
    updateWorkflow(inferWorkflowStep(s));
    setTxt('g-state-pill', s.state);
    setTxt('g-info-target', s.target_class || '—');
    setTxt('g-info-u',      fmtOrDash(s.target_u, 3));
    setTxt('g-info-dist',   fmtOrDash(s.distance, 2));
    setTxt('g-info-bbox',   fmtOrDash(s.bbox_max, 2));
    setTxt('g-info-count',  s.det_count != null ? s.det_count : '—');
    setTxt('g-info-fps',    (s.fps || 0).toFixed(1));

    // ----- 视图 2：目标识别 -----
    setTxt('d-fps',    (s.fps || 0).toFixed(1));
    setTxt('d-target', s.target_class || '—');
    setTxt('d-count',  s.det_count != null ? s.det_count : '—');
    setTxt('d-u',      fmtOrDash(s.target_u, 3));
    setTxt('d-bbox',   fmtOrDash(s.bbox_max, 2));
    setTxt('d-dist',   fmtOrDash(s.distance, 2, ' m'));

    // ----- 视图 3：运动控制 -----
    setTxt('c-info-state', s.state);
    setTxt('c-info-mode', s.state === 'WORKING' ? '自动任务' : '手动遥控');
    setTxt('c-info-fps', (s.fps || 0).toFixed(1));

    // ----- 视图 4：系统状态仪表盘 -----
    setTxt('s-fps',        (s.fps || 0).toFixed(1));
    setTxt('s-det-count',  s.det_count != null ? s.det_count : '—');
    setTxt('s-target',     s.target_class || '—');
    setTxt('s-u',          fmtOrDash(s.target_u, 3));
    setTxt('s-bbox',       fmtOrDash(s.bbox_max, 2));
    setTxt('s-dist',       fmtOrDash(s.distance, 2, ' m'));
    setTxt('s-state',      s.state);

    // 进度条
    setBarPct('s-fps-bar', ((s.fps || 0) / 30) * 100);
    // 深度距离：0 ~ 2m 映射到 100% ~ 0%（近时满条）
    if (s.distance != null) {
      setBarPct('s-dist-bar', Math.max(0, 100 - (s.distance / 2) * 100));
    } else {
      setBarPct('s-dist-bar', 0);
    }
    // u 位置条：距中心越近越满
    if (s.target_u != null) {
      setBarPct('s-u-bar', Math.max(0, 100 - Math.abs(s.target_u - 0.5) * 200));
    } else {
      setBarPct('s-u-bar', 0);
    }

    // 健康度环
    updateHealthRing(computeHealth(s));

    // ----- 日志追加 -----
    if (s.logs && s.logs.length > 0) {
      for (const entry of s.logs) appendLog(entry.msg, entry.level);
      lastLogIdx = s.log_idx;
    }
  } catch (e) {
    connFailCount++;
    updateConnIndicator(false);
  }
}

// 连接指示器（侧边栏底部）
function updateConnIndicator(ok, isMock = false) {
  const label = document.getElementById('conn-label');
  const dot = document.querySelector('.footer-pulse .pulse-dot');
  if (!label) return;
  if (ok) {
    if (isMock) {
      label.textContent = '本地预览';
      label.style.color = '#D97706';
      if (dot) dot.style.background = '#F59E0B';
    } else {
      label.textContent = '已连接';
      label.style.color = '#059669';
      if (dot) dot.style.background = '#10B981';
    }
  } else if (connFailCount > 3) {
    label.textContent = '连接断开';
    label.style.color = '#DC2626';
    if (dot) dot.style.background = '#EF4444';
  }
}

setInterval(pollState, getLocalPref('poll_interval', 500));
pollState();
appendLog('Web 面板已连接', 'info');


// ======================================================================
// 系统设置模块
// ======================================================================
// 本地偏好（存 localStorage）默认值
const LOCAL_PREF_DEFAULTS = {
  default_view: 'grasp',
  log_toast: false,
  poll_interval: 500,
};

function getLocalPref(key, fallback) {
  try {
    const v = localStorage.getItem('g1_pref_' + key);
    if (v == null) return fallback != null ? fallback : LOCAL_PREF_DEFAULTS[key];
    return JSON.parse(v);
  } catch (e) {
    return fallback != null ? fallback : LOCAL_PREF_DEFAULTS[key];
  }
}

function setLocalPref(key, value) {
  localStorage.setItem('g1_pref_' + key, JSON.stringify(value));
}

function resetLocalPrefs() {
  Object.keys(LOCAL_PREF_DEFAULTS).forEach(k => localStorage.removeItem('g1_pref_' + k));
  showToast('已重置本地偏好，刷新后生效', 'info');
  // 立刻把表单值刷回默认
  document.querySelectorAll('[data-local="true"]').forEach(el => {
    applyValueToInput(el, LOCAL_PREF_DEFAULTS[el.dataset.key]);
  });
}

function applyValueToInput(el, val) {
  if (el.type === 'checkbox') el.checked = !!val;
  else el.value = val == null ? '' : val;
}

function readValueFromInput(el) {
  if (el.type === 'checkbox') return el.checked;
  if (el.type === 'number') return el.value === '' ? null : parseFloat(el.value);
  return el.value;
}

// 加载配置：从后端拉取 + 本地偏好填充
async function reloadSettings() {
  // 1. 本地偏好
  document.querySelectorAll('[data-local="true"]').forEach(el => {
    applyValueToInput(el, getLocalPref(el.dataset.key));
  });

  // 2. 后端参数
  try {
    const r = await fetch('/api/config');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const data = await r.json();
    const values = data.values || {};
    document.querySelectorAll('[data-key]').forEach(el => {
      if (el.dataset.local === 'true') return;
      const key = el.dataset.key;
      if (key in values && values[key] != null) {
        applyValueToInput(el, values[key]);
      }
    });
    showToast('配置已加载', 'info');
  } catch (e) {
    showToast('加载配置失败: ' + e.message, 'error');
  }
}

// 保存某一分组
async function saveSettingsGroup(groupName) {
  const form = document.querySelector(`[data-settings-group="${groupName}"]`);
  if (!form) return;

  const backendUpdates = {};
  let localUpdateCount = 0;

  form.querySelectorAll('[data-key]').forEach(el => {
    const key = el.dataset.key;
    const val = readValueFromInput(el);
    if (el.dataset.local === 'true') {
      setLocalPref(key, val);
      localUpdateCount++;
    } else if (val !== null && val !== '') {
      backendUpdates[key] = val;
    }
  });

  // 后端更新
  if (Object.keys(backendUpdates).length > 0) {
    try {
      const r = await fetch('/api/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(backendUpdates),
      });
      const data = await r.json();
      if (!r.ok || data.ok === false) throw new Error(data.error || 'save failed');
      const updated = data.updated || [];
      const skipped = data.skipped || [];
      if (skipped.length > 0) {
        showToast(`已保存 ${updated.length} 项，${skipped.length} 项跳过`, 'error');
        for (const s of skipped) appendLog(`[配置] ${s.key} 跳过: ${s.reason}`, 'warn');
      } else {
        showToast(`已保存 ${updated.length} 项配置`, 'info');
      }
    } catch (e) {
      showToast('保存失败: ' + e.message, 'error');
      return;
    }
  } else if (localUpdateCount > 0) {
    showToast(`已保存 ${localUpdateCount} 项本地偏好`, 'info');
  } else {
    showToast('没有需要保存的更改', 'info');
  }
}

// 保存按钮事件绑定
document.querySelectorAll('[data-save]').forEach(btn => {
  btn.addEventListener('click', () => saveSettingsGroup(btn.dataset.save));
});

// Toast 提示
let toastTimer = null;
function showToast(msg, level = 'info') {
  const el = document.getElementById('settings-toast');
  if (!el) return;
  el.textContent = msg;
  el.classList.remove('hidden', 'error');
  if (level === 'error') el.classList.add('error');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.add('hidden'), 2500);
}

// 初次进入设置页时懒加载
let settingsLoaded = false;
const origSwitchView = switchView;
switchView = function(name) {
  origSwitchView(name);
  if (name === 'settings' && !settingsLoaded) {
    settingsLoaded = true;
    reloadSettings();
  }
};

// 如果 hash 初始就是 settings，立即加载
if (location.hash === '#settings') {
  settingsLoaded = true;
  reloadSettings();
}

// 应用启动视图偏好（若未指定 hash）
if (!location.hash) {
  const defaultView = getLocalPref('default_view', 'grasp');
  if (defaultView && defaultView !== 'grasp') switchView(defaultView);
}

