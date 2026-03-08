/* ==========================================
   simulation.js — FK/IK Math + Interactive Canvas
   ========================================== */

(function () {
  'use strict';

  // ===== ARM PARAMETERS =====
  const d1 = 5.0;   // base height (cm)
  const a2 = 10.5;  // upper arm length (cm)
  const a3 = 10.0;  // forearm length (cm)

  // ===== EMOTION PRESETS (servo → geometric angles) =====
  // Mapping: θ1 = servo_base - 90, θ2 = 90 - servo_shoulder, θ3 = servo_elbow - 90
  const PRESETS = {
    NEUTRAL:  { t1: 0,   t2: 90,  t3: 0,   label: '😐 Neutral' },
    HAPPY:    { t1: 10,  t2: 70,  t3: 30,  label: '😊 Happy' },
    SAD:      { t1: -10, t2: 40,  t3: -30, label: '😢 Sad' },
    ANGRY:    { t1: 20,  t2: 55,  t3: -50, label: '😡 Angry' },
    SURPRISE: { t1: 0,   t2: 85,  t3: 60,  label: '😲 Surprise' }
  };

  // ===== STATE =====
  let mode = 'fk';      // 'fk' or 'ik'
  let theta1 = 0;       // base rotation (deg)
  let theta2 = 90;      // shoulder (deg, from horizontal)
  let theta3 = 0;       // elbow (deg, relative to upper arm)
  let targetR = null;    // IK target radial (cm)
  let targetZ = null;    // IK target height (cm)
  let trace = [];        // end-effector trace points
  let animating = false;
  let activePreset = null;

  // Canvas setup
  const canvas = document.getElementById('sim-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  // Drawing constants
  const SCALE = 14;       // pixels per cm
  const ORIGIN_X = 180;   // pixel x of base center
  const ORIGIN_Y = 390;   // pixel y of ground level

  // DOM elements
  const sliderT1 = document.getElementById('slider-t1');
  const sliderT2 = document.getElementById('slider-t2');
  const sliderT3 = document.getElementById('slider-t3');
  const valT1 = document.getElementById('val-t1');
  const valT2 = document.getElementById('val-t2');
  const valT3 = document.getElementById('val-t3');
  const eeX = document.getElementById('ee-x');
  const eeY = document.getElementById('ee-y');
  const eeZ = document.getElementById('ee-z');

  // ===== FORWARD KINEMATICS =====
  function deg2rad(d) { return d * Math.PI / 180; }
  function rad2deg(r) { return r * 180 / Math.PI; }

  function forwardKinematics(t1Deg, t2Deg, t3Deg) {
    const t1 = deg2rad(t1Deg);
    const t2 = deg2rad(t2Deg);
    const t3 = deg2rad(t3Deg);
    const t23 = t2 + t3;

    // In the side view (r-z plane), θ1 doesn't change the profile
    const r = a2 * Math.cos(t2) + a3 * Math.cos(t23);
    const z = d1 + a2 * Math.sin(t2) + a3 * Math.sin(t23);

    // 3D position
    const px = Math.cos(t1) * r;
    const py = Math.sin(t1) * r;
    const pz = z;

    return { px, py, pz, r, z };
  }

  function getJointPositions(t2Deg, t3Deg) {
    const t2 = deg2rad(t2Deg);
    const t3 = deg2rad(t3Deg);
    const t23 = t2 + t3;

    return {
      base:     { r: 0, z: 0 },
      shoulder: { r: 0, z: d1 },
      elbow:    { r: a2 * Math.cos(t2), z: d1 + a2 * Math.sin(t2) },
      ee:       { r: a2 * Math.cos(t2) + a3 * Math.cos(t23), z: d1 + a2 * Math.sin(t2) + a3 * Math.sin(t23) }
    };
  }

  // ===== INVERSE KINEMATICS =====
  function inverseKinematics(rTarget, zTarget) {
    const s = zTarget - d1;
    const distSq = rTarget * rTarget + s * s;
    const dist = Math.sqrt(distSq);

    // Reachability check
    const rMax = a2 + a3;
    const rMin = Math.abs(a2 - a3);
    if (dist > rMax || dist < rMin) return null;

    // Elbow angle (law of cosines)
    const D = (distSq - a2 * a2 - a3 * a3) / (2 * a2 * a3);
    const Dclamped = Math.max(-1, Math.min(1, D));
    const t3 = Math.atan2(Math.sqrt(1 - Dclamped * Dclamped), Dclamped); // elbow-up

    // Shoulder angle
    const t2 = Math.atan2(s, rTarget) - Math.atan2(a3 * Math.sin(t3), a2 + a3 * Math.cos(t3));

    return { t2: rad2deg(t2), t3: rad2deg(t3) };
  }

  // ===== CANVAS DRAWING =====
  function worldToCanvas(r, z) {
    return {
      x: ORIGIN_X + r * SCALE,
      y: ORIGIN_Y - z * SCALE
    };
  }

  function canvasToWorld(cx, cy) {
    return {
      r: (cx - ORIGIN_X) / SCALE,
      z: (ORIGIN_Y - cy) / SCALE
    };
  }

  function drawGrid() {
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
    ctx.lineWidth = 1;

    // Vertical lines
    for (let r = -15; r <= 25; r += 5) {
      const p = worldToCanvas(r, 0);
      ctx.beginPath();
      ctx.moveTo(p.x, 0);
      ctx.lineTo(p.x, canvas.height);
      ctx.stroke();
    }

    // Horizontal lines
    for (let z = -5; z <= 30; z += 5) {
      const p = worldToCanvas(0, z);
      ctx.beginPath();
      ctx.moveTo(0, p.y);
      ctx.lineTo(canvas.width, p.y);
      ctx.stroke();
    }

    // Axes
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
    ctx.lineWidth = 1;

    // Z-axis (vertical)
    const zBottom = worldToCanvas(0, -2);
    const zTop = worldToCanvas(0, 28);
    ctx.beginPath();
    ctx.moveTo(zBottom.x, zBottom.y);
    ctx.lineTo(zTop.x, zTop.y);
    ctx.stroke();

    // R-axis (horizontal through shoulder)
    const rLeft = worldToCanvas(-12, d1);
    const rRight = worldToCanvas(24, d1);
    ctx.beginPath();
    ctx.setLineDash([4, 4]);
    ctx.moveTo(rLeft.x, rLeft.y);
    ctx.lineTo(rRight.x, rRight.y);
    ctx.stroke();
    ctx.setLineDash([]);

    // Axis labels
    ctx.font = '10px JetBrains Mono, monospace';
    ctx.fillStyle = 'rgba(255, 255, 255, 0.2)';
    ctx.fillText('r (cm)', canvas.width - 50, ORIGIN_Y - d1 * SCALE - 5);
    ctx.fillText('z (cm)', ORIGIN_X + 5, 15);
  }

  function drawWorkspace() {
    // Draw reachable arc from shoulder
    ctx.strokeStyle = 'rgba(0, 255, 200, 0.08)';
    ctx.lineWidth = 1;
    const shoulder = worldToCanvas(0, d1);

    // Outer boundary (a2 + a3)
    const rOuter = (a2 + a3) * SCALE;
    ctx.beginPath();
    ctx.arc(shoulder.x, shoulder.y, rOuter, -Math.PI, 0);
    ctx.stroke();

    // Inner boundary (|a2 - a3|)
    const rInner = Math.abs(a2 - a3) * SCALE;
    if (rInner > 2) {
      ctx.beginPath();
      ctx.arc(shoulder.x, shoulder.y, rInner, -Math.PI, 0);
      ctx.stroke();
    }

    // Fill workspace area
    ctx.fillStyle = 'rgba(0, 255, 200, 0.02)';
    ctx.beginPath();
    ctx.arc(shoulder.x, shoulder.y, rOuter, -Math.PI, 0);
    if (rInner > 2) {
      ctx.arc(shoulder.x, shoulder.y, rInner, 0, -Math.PI, true);
    }
    ctx.closePath();
    ctx.fill();
  }

  function drawTrace() {
    if (trace.length < 2) return;
    ctx.strokeStyle = 'rgba(0, 255, 200, 0.25)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    const p0 = worldToCanvas(trace[0].r, trace[0].z);
    ctx.moveTo(p0.x, p0.y);
    for (let i = 1; i < trace.length; i++) {
      const p = worldToCanvas(trace[i].r, trace[i].z);
      ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();

    // Dots
    trace.forEach((pt, i) => {
      const p = worldToCanvas(pt.r, pt.z);
      const alpha = 0.1 + 0.3 * (i / trace.length);
      ctx.fillStyle = `rgba(0, 255, 200, ${alpha})`;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 2, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  function drawArm(joints) {
    const base = worldToCanvas(joints.base.r, joints.base.z);
    const shoulder = worldToCanvas(joints.shoulder.r, joints.shoulder.z);
    const elbow = worldToCanvas(joints.elbow.r, joints.elbow.z);
    const ee = worldToCanvas(joints.ee.r, joints.ee.z);

    // Base platform
    ctx.fillStyle = '#253545';
    ctx.strokeStyle = '#3a5060';
    ctx.lineWidth = 2;
    const bw = 40, bh = 12;
    ctx.fillRect(base.x - bw / 2, base.y - bh / 2, bw, bh);
    ctx.strokeRect(base.x - bw / 2, base.y - bh / 2, bw, bh);

    // Base column (base to shoulder)
    ctx.strokeStyle = '#405565';
    ctx.lineWidth = 8;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(base.x, base.y);
    ctx.lineTo(shoulder.x, shoulder.y);
    ctx.stroke();

    // Upper arm (shoulder to elbow)
    ctx.strokeStyle = '#00cc99';
    ctx.lineWidth = 7;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(shoulder.x, shoulder.y);
    ctx.lineTo(elbow.x, elbow.y);
    ctx.stroke();

    // Link border
    ctx.strokeStyle = '#00ffc8';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(shoulder.x, shoulder.y);
    ctx.lineTo(elbow.x, elbow.y);
    ctx.stroke();

    // Forearm (elbow to end-effector)
    ctx.strokeStyle = '#0077cc';
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(elbow.x, elbow.y);
    ctx.lineTo(ee.x, ee.y);
    ctx.stroke();

    // Link border
    ctx.strokeStyle = '#0099ff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(elbow.x, elbow.y);
    ctx.lineTo(ee.x, ee.y);
    ctx.stroke();

    // Joints
    drawJoint(shoulder.x, shoulder.y, 9, '#00ffc8');
    drawJoint(elbow.x, elbow.y, 7, '#0099ff');

    // End-effector (diamond)
    drawEndEffector(ee.x, ee.y);

    // Joint angle arcs
    drawAngleArc(shoulder, 0, theta2, 'rgba(0, 255, 200, 0.3)', 30);
    drawAngleArc(elbow, theta2, theta2 + theta3, 'rgba(0, 153, 255, 0.3)', 25);

    // Labels
    ctx.font = '10px JetBrains Mono, monospace';
    ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
    ctx.fillText('Base', base.x - 12, base.y + 22);
    ctx.fillText(`Shoulder`, shoulder.x + 14, shoulder.y + 4);
    ctx.fillText(`Elbow`, elbow.x + 12, elbow.y + 4);

    ctx.font = 'bold 11px JetBrains Mono, monospace';
    ctx.fillStyle = '#ff3366';
    ctx.fillText(`EE`, ee.x + 10, ee.y - 8);
  }

  function drawJoint(x, y, radius, color) {
    // Glow
    ctx.fillStyle = color.replace(')', ', 0.2)').replace('rgb', 'rgba');
    ctx.beginPath();
    ctx.arc(x, y, radius + 4, 0, Math.PI * 2);
    ctx.fill();

    // Outer ring
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.stroke();

    // Inner fill
    ctx.fillStyle = '#0a0e14';
    ctx.beginPath();
    ctx.arc(x, y, radius - 2, 0, Math.PI * 2);
    ctx.fill();

    // Center dot
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(x, y, 2, 0, Math.PI * 2);
    ctx.fill();
  }

  function drawEndEffector(x, y) {
    const s = 7;
    ctx.fillStyle = '#ff3366';
    ctx.beginPath();
    ctx.moveTo(x, y - s);
    ctx.lineTo(x + s, y);
    ctx.lineTo(x, y + s);
    ctx.lineTo(x - s, y);
    ctx.closePath();
    ctx.fill();

    // Glow
    ctx.fillStyle = 'rgba(255, 51, 102, 0.2)';
    ctx.beginPath();
    ctx.arc(x, y, s + 4, 0, Math.PI * 2);
    ctx.fill();
  }

  function drawAngleArc(center, startDeg, endDeg, color, radius) {
    // Angles in canvas coordinates (y is flipped)
    const startRad = -deg2rad(startDeg);
    const endRad = -deg2rad(endDeg);

    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(center.x, center.y, radius, startRad, endRad, endDeg > startDeg);
    ctx.stroke();
  }

  function drawTarget() {
    if (targetR === null || targetZ === null) return;
    const p = worldToCanvas(targetR, targetZ);

    // Crosshair
    ctx.strokeStyle = '#ff3366';
    ctx.lineWidth = 1;
    const cs = 12;
    ctx.beginPath();
    ctx.moveTo(p.x - cs, p.y);
    ctx.lineTo(p.x + cs, p.y);
    ctx.moveTo(p.x, p.y - cs);
    ctx.lineTo(p.x, p.y + cs);
    ctx.stroke();

    // Circle
    ctx.beginPath();
    ctx.arc(p.x, p.y, 6, 0, Math.PI * 2);
    ctx.stroke();

    // Glow
    ctx.fillStyle = 'rgba(255, 51, 102, 0.15)';
    ctx.beginPath();
    ctx.arc(p.x, p.y, 10, 0, Math.PI * 2);
    ctx.fill();

    // Label
    ctx.font = '9px JetBrains Mono, monospace';
    ctx.fillStyle = '#ff3366';
    ctx.fillText(`(${targetR.toFixed(1)}, ${targetZ.toFixed(1)})`, p.x + 14, p.y - 4);
  }

  function drawBaseIndicator() {
    // Small circular indicator for base rotation
    const cx = canvas.width - 55;
    const cy = canvas.height - 55;
    const r = 35;

    // Background circle
    ctx.fillStyle = 'rgba(17, 24, 32, 0.9)';
    ctx.beginPath();
    ctx.arc(cx, cy, r + 2, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();

    // Range arc (servo 30-150 → θ1 -60 to 60)
    ctx.strokeStyle = 'rgba(0, 255, 200, 0.15)';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(cx, cy, r - 5, deg2rad(-60 - 90), deg2rad(60 - 90));
    ctx.stroke();

    // Current angle indicator
    const angle = deg2rad(theta1 - 90); // -90 because canvas 0° is right
    const ix = cx + (r - 5) * Math.cos(angle);
    const iy = cy + (r - 5) * Math.sin(angle);

    ctx.strokeStyle = '#00ffc8';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(ix, iy);
    ctx.stroke();

    ctx.fillStyle = '#00ffc8';
    ctx.beginPath();
    ctx.arc(ix, iy, 3, 0, Math.PI * 2);
    ctx.fill();

    // Label
    ctx.font = '9px JetBrains Mono, monospace';
    ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
    ctx.textAlign = 'center';
    ctx.fillText('θ₁ Base', cx, cy + r + 14);
    ctx.fillStyle = '#00ffc8';
    ctx.fillText(`${theta1}°`, cx, cy + 3);
    ctx.textAlign = 'left';
  }

  function drawInfo() {
    // Mode label
    ctx.font = 'bold 11px JetBrains Mono, monospace';
    ctx.fillStyle = mode === 'fk' ? '#00ffc8' : '#ff3366';
    ctx.fillText(mode === 'fk' ? '[ FK MODE ]' : '[ IK MODE ]', 12, 20);

    // Active emotion
    if (activePreset) {
      ctx.font = '10px JetBrains Mono, monospace';
      ctx.fillStyle = 'rgba(255, 255, 255, 0.3)';
      ctx.fillText(`Pose: ${activePreset}`, 12, 36);
    }
  }

  function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Background
    ctx.fillStyle = '#0d1219';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    drawGrid();
    drawWorkspace();
    drawTrace();

    const joints = getJointPositions(theta2, theta3);
    drawArm(joints);
    drawTarget();
    drawBaseIndicator();
    drawInfo();
  }

  // ===== UI UPDATES =====
  function updateSliders() {
    if (sliderT1) sliderT1.value = theta1;
    if (sliderT2) sliderT2.value = theta2;
    if (sliderT3) sliderT3.value = theta3;
    updateDisplay();
  }

  function updateDisplay() {
    if (valT1) valT1.textContent = `${theta1}°`;
    if (valT2) valT2.textContent = `${theta2}°`;
    if (valT3) valT3.textContent = `${theta3}°`;

    const fk = forwardKinematics(theta1, theta2, theta3);
    if (eeX) eeX.textContent = fk.px.toFixed(2);
    if (eeY) eeY.textContent = fk.py.toFixed(2);
    if (eeZ) eeZ.textContent = fk.pz.toFixed(2);

    // Add to trace (limit length)
    trace.push({ r: fk.r, z: fk.pz });
    if (trace.length > 500) trace.shift();

    draw();
  }

  // ===== SLIDER EVENT HANDLERS =====
  function onSliderInput() {
    if (animating) return;
    theta1 = parseInt(sliderT1.value);
    theta2 = parseInt(sliderT2.value);
    theta3 = parseInt(sliderT3.value);
    activePreset = null;
    clearPresetActive();
    updateDisplay();
  }

  if (sliderT1) sliderT1.addEventListener('input', onSliderInput);
  if (sliderT2) sliderT2.addEventListener('input', onSliderInput);
  if (sliderT3) sliderT3.addEventListener('input', onSliderInput);

  // ===== CANVAS CLICK (IK) =====
  canvas.addEventListener('click', function (e) {
    if (mode !== 'ik') return;

    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const cx = (e.clientX - rect.left) * scaleX;
    const cy = (e.clientY - rect.top) * scaleY;

    const world = canvasToWorld(cx, cy);
    targetR = world.r;
    targetZ = world.z;

    const solution = inverseKinematics(targetR, targetZ);
    if (solution) {
      activePreset = null;
      clearPresetActive();
      animateTo(theta1, solution.t2, solution.t3);
    } else {
      // Flash the target in red (unreachable)
      draw();
      ctx.font = 'bold 11px JetBrains Mono, monospace';
      ctx.fillStyle = '#ff3366';
      const p = worldToCanvas(targetR, targetZ);
      ctx.fillText('UNREACHABLE', p.x + 14, p.y + 14);
    }
  });

  // ===== ANIMATION =====
  function animateTo(t1Target, t2Target, t3Target) {
    animating = true;
    const t1Start = theta1, t2Start = theta2, t3Start = theta3;
    const duration = 600; // ms
    const startTime = performance.now();

    function easeInOutCubic(t) {
      return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
    }

    function step(now) {
      const elapsed = now - startTime;
      const progress = Math.min(elapsed / duration, 1);
      const eased = easeInOutCubic(progress);

      theta1 = Math.round(t1Start + (t1Target - t1Start) * eased);
      theta2 = Math.round(t2Start + (t2Target - t2Start) * eased);
      theta3 = Math.round(t3Start + (t3Target - t3Start) * eased);

      updateSliders();

      if (progress < 1) {
        requestAnimationFrame(step);
      } else {
        theta1 = Math.round(t1Target);
        theta2 = Math.round(t2Target);
        theta3 = Math.round(t3Target);
        updateSliders();
        animating = false;
      }
    }

    requestAnimationFrame(step);
  }

  // ===== GLOBAL FUNCTIONS (called from HTML) =====
  window.setMode = function (m) {
    mode = m;
    document.getElementById('btn-fk').classList.toggle('active', m === 'fk');
    document.getElementById('btn-ik').classList.toggle('active', m === 'ik');
    if (m === 'fk') {
      targetR = null;
      targetZ = null;
    }
    draw();
  };

  window.setEmotionPreset = function (emotion) {
    const preset = PRESETS[emotion];
    if (!preset) return;
    activePreset = emotion;
    clearPresetActive();

    // Highlight active button
    document.querySelectorAll('.preset-btn').forEach(btn => {
      if (btn.textContent.toUpperCase().includes(emotion)) {
        btn.classList.add('active');
      }
    });

    animateTo(preset.t1, preset.t2, preset.t3);
  };

  window.clearTrace = function () {
    trace = [];
    draw();
  };

  function clearPresetActive() {
    document.querySelectorAll('.preset-btn').forEach(btn => {
      btn.classList.remove('active');
    });
  }

  // ===== INITIAL DRAW =====
  updateDisplay();

})();
