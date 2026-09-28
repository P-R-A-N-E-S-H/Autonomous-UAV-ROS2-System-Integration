/**
 * app.js
 * Master Application Controller & Live Telemetry Poller.
 * Role: Member 3 - ROS2 & System Integration Engineer.
 */

let visualizer = null;
let lastTelemetry = null;

document.addEventListener('DOMContentLoaded', () => {
  visualizer = new MissionVisualizer('flight-canvas');
  startTelemetryPolling();
  requestAnimationFrame(renderLoop);
});

function switchTab(tabId) {
  document.querySelectorAll('.tab-pane').forEach(el => el.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));

  const targetPane = document.getElementById(tabId);
  if (targetPane) targetPane.classList.add('active');

  const activeBtn = Array.from(document.querySelectorAll('.tab-btn')).find(b => b.getAttribute('onclick').includes(tabId));
  if (activeBtn) activeBtn.classList.add('active');

  if (tabId === 'tab-live-gcs' && visualizer) {
    setTimeout(() => visualizer.resize(), 100);
  }
}

function toggleCanvasView() {
  if (visualizer) {
    visualizer.isIsometric = !visualizer.isIsometric;
  }
}

function clearTrail() {
  if (visualizer) {
    visualizer.flightPath = [];
  }
}

function renderLoop() {
  if (visualizer) {
    visualizer.render();
  }
  requestAnimationFrame(renderLoop);
}

async function startTelemetryPolling() {
  const poll = async () => {
    try {
      const resp = await fetch('/api/telemetry');
      if (resp.ok) {
        const data = await resp.json();
        lastTelemetry = data;
        updateUI(data);
        if (visualizer) {
          visualizer.updateData(data);
        }
      }
    } catch (e) {
      console.warn("Telemetry fetch error:", e);
    }
    setTimeout(poll, 65); // ~15Hz polling
  };
  poll();
}

function updateUI(t) {
  // 1. Top HUD
  const fsmState = t.fsm.state_name;
  const hudState = document.getElementById('hud-state');
  hudState.textContent = fsmState;
  
  if (fsmState === 'EMERGENCY_HOLD') {
    hudState.className = 'metric-val text-red';
  } else if (fsmState === 'TAKEOFF' || fsmState === 'SEMANTIC_GOAL_NAV') {
    hudState.className = 'metric-val text-cyan';
  } else {
    hudState.className = 'metric-val text-emerald';
  }

  const batPct = t.drone.battery_pct;
  const hudBat = document.getElementById('hud-battery');
  hudBat.innerHTML = `${batPct.toFixed(1)}% <small>(${t.drone.battery_voltage}V)</small>`;
  if (batPct < 20) {
    hudBat.className = 'metric-val text-red';
  } else if (batPct < 30) {
    hudBat.className = 'metric-val text-amber';
  } else {
    hudBat.className = 'metric-val text-emerald';
  }

  const hudSlam = document.getElementById('hud-slam');
  if (t.drone.slam_healthy) {
    hudSlam.textContent = 'HEALTHY';
    hudSlam.className = 'metric-val text-emerald';
  } else {
    hudSlam.textContent = 'TRACKING LOST';
    hudSlam.className = 'metric-val text-red';
  }

  document.getElementById('hud-latency').textContent = `${t.topics['/mavros/setpoint_position/local'].latency_ms.toFixed(1)} ms`;

  // 2. Canvas Overlays
  const p = t.drone.position;
  document.getElementById('overlay-pos').textContent = `X: ${p.x.toFixed(2)} | Y: ${p.y.toFixed(2)} | Z: ${p.z.toFixed(2)}`;
  
  const v = t.drone.velocity;
  const speed = Math.sqrt(v.vx*v.vx + v.vy*v.vy + v.vz*v.vz);
  document.getElementById('overlay-vel').textContent = `${speed.toFixed(2)} m/s`;
  document.getElementById('overlay-yaw').textContent = `${t.drone.orientation.yaw_deg.toFixed(1)}°`;
  
  const goalEl = document.getElementById('overlay-goal');
  goalEl.textContent = t.fsm.active_goal || 'None (Station-Keeping)';

  // 3. FSM Visualizer Nodes
  document.querySelectorAll('.fsm-node').forEach(node => node.classList.remove('active-glow'));
  const activeNode = document.getElementById(`fsm-${fsmState}`);
  if (activeNode) {
    activeNode.classList.add('active-glow');
  }

  // 4. Live 4x4 Transformation Matrix Display
  const yawRad = (t.drone.orientation.yaw_deg * Math.PI) / 180;
  const cosY = Math.cos(yawRad).toFixed(3);
  const sinY = Math.sin(yawRad).toFixed(3);
  const mDiv = document.getElementById('live-matrix-display');
  if (mDiv) {
    mDiv.innerHTML = 
      `| ${cosY.padStart(6)}  ${(-sinY).padStart(6)}   0.000 |  ${p.x.toFixed(3).padStart(6)} |<br>` +
      `| ${sinY.padStart(6)}   ${cosY.padStart(6)}   0.000 |  ${p.y.toFixed(3).padStart(6)} |<br>` +
      `|  0.000   0.000   1.000 |  ${p.z.toFixed(3).padStart(6)} |<br>` +
      `|  0.000   0.000   0.000 |  1.000 |`;
  }

  // 5. Event Logs
  const logBox = document.getElementById('event-log-container');
  if (logBox && t.events) {
    logBox.innerHTML = t.events.map(ev => 
      `<div class="log-entry"><span class="log-time">[${ev.time}]</span> <span class="log-src">${ev.source}:</span> ${ev.msg}</div>`
    ).join('');
  }
}

async function sendCmd(action, payload = {}) {
  try {
    const res = await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action, ...payload })
    });
    return await res.json();
  } catch (e) {
    console.error("Failed to dispatch command:", e);
  }
}

function sendVLMGoal(prompt, className, targetCoords) {
  sendCmd('VLM_GOAL', {
    prompt: prompt,
    class: className,
    target: targetCoords
  });
}
