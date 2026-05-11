// G1 NavGrasp Web 控制面板 — 前端逻辑
// 负责：状态轮询、日志追加、命令下发

const STATE_COLORS = {
  IDLE:     { dot: 'bg-slate-400',   text: '空闲' },
  WORKING:  { dot: 'bg-cyan-500',    text: '执行中' },
  GRABBING: { dot: 'bg-red-500',     text: '抓取中' },
  MENU:     { dot: 'bg-emerald-500', text: '可放下' },
};

const speedSlider = document.getElementById('speed-slider');
const speedValue = document.getElementById('speed-value');
speedSlider.addEventListener('input',
  () => speedValue.textContent = (+speedSlider.value).toFixed(2));

function getSpeed() { return parseFloat(speedSlider.value); }

async function postCmd(path, body = {}) {
  try {
    const r = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const data = await r.json();
    if (!r.ok) appendLog(data.error || 'error', 'error');
  } catch (e) {
    appendLog('网络错误: ' + e.message, 'error');
  }
}

document.querySelectorAll('button[data-cmd]').forEach(btn => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.cmd;
    const v = getSpeed();
    const vyaw = v * 2.0;   // 转向速度按线速度的 2 倍
    switch (cmd) {
      case 'forward':      postCmd('/api/cmd/manual', { vx:  v, vy: 0, vyaw: 0 }); break;
      case 'backward':     postCmd('/api/cmd/manual', { vx: -v, vy: 0, vyaw: 0 }); break;
      case 'left':         postCmd('/api/cmd/manual', { vx: 0, vy: 0, vyaw:  vyaw }); break;
      case 'right':        postCmd('/api/cmd/manual', { vx: 0, vy: 0, vyaw: -vyaw }); break;
      case 'stop':         postCmd('/api/cmd/stop'); break;
      case 'search':       postCmd('/api/cmd/search'); break;
      case 'grab':         postCmd('/api/cmd/grab'); break;
      case 'putdown':      postCmd('/api/cmd/putdown'); break;
      case 'turn_putdown': postCmd('/api/cmd/turn_putdown'); break;
    }
  });
});

function appendLog(msg, level = 'info') {
  const box = document.getElementById('log-box');
  const line = document.createElement('div');
  const ts = new Date().toLocaleTimeString('zh-CN', { hour12: false });
  line.className = `log-line log-${level}`;
  line.textContent = `[${ts}] ${msg}`;
  box.appendChild(line);
  if (box.children.length > 300) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
}

function clearLog() {
  document.getElementById('log-box').innerHTML = '';
}

let lastLogIdx = 0;
async function pollState() {
  try {
    const r = await fetch('/api/state?since=' + lastLogIdx);
    if (!r.ok) return;
    const s = await r.json();

    const st = STATE_COLORS[s.state] || { dot: 'bg-slate-400', text: s.state };
    document.getElementById('state-dot').className = 'state-dot ' + st.dot;
    document.getElementById('state-label').textContent = st.text;

    document.getElementById('fps-label').textContent = 'FPS ' + (s.fps || 0).toFixed(0);
    document.getElementById('det-label').textContent = '检测 ' + (s.det_count || 0);

    document.getElementById('info-state').textContent = s.state;
    document.getElementById('info-target').textContent = s.target_class || '—';
    document.getElementById('info-u').textContent = s.target_u != null ? s.target_u.toFixed(3) : '—';
    document.getElementById('info-bbox').textContent = s.bbox_max != null ? s.bbox_max.toFixed(2) : '—';
    document.getElementById('info-dist').textContent = s.distance != null ? s.distance.toFixed(2) + ' m' : '—';
    document.getElementById('info-count').textContent = s.det_count != null ? s.det_count : '—';

    if (s.logs && s.logs.length > 0) {
      for (const entry of s.logs) appendLog(entry.msg, entry.level);
      lastLogIdx = s.log_idx;
    }
  } catch (e) { /* 暂时忽略网络异常 */ }
}

setInterval(pollState, 500);
pollState();
appendLog('Web 面板已连接', 'info');
