/* ==========================================
   simulation3d.js — 3D Robotic Arm Simulation
   Three.js based, matching actual hardware:
   - D9 Base: 60°–120° (neutral 90°)
   - D10 Shoulder: 0°–60° (neutral 0°, 0=up)
   - D11 Elbow: 30°–150° (neutral 90°)
   ========================================== */

(function () {
  'use strict';

  // ===== ARM PHYSICAL PARAMETERS =====
  const BASE_HEIGHT   = 3.0;   // base platform height (cm)
  const SHOULDER_HEIGHT = 5.0; // shoulder pivot above ground (cm)
  const UPPER_ARM_LEN = 9.0;   // upper arm link length (cm)
  const FOREARM_LEN   = 8.0;   // forearm link length (cm)

  // ===== SERVO LIMITS (matching Arduino firmware) =====
  const BASE_MIN = 60, BASE_MAX = 120, BASE_NEUTRAL = 90;
  const SHOULDER_MIN = 0, SHOULDER_MAX = 60, SHOULDER_NEUTRAL = 0;
  const ELBOW_MIN = 30, ELBOW_MAX = 150, ELBOW_NEUTRAL = 90;

  // ===== EMOTION PRESETS (actual servo values from code) =====
  const PRESETS = {
    NEUTRAL:  { base: 90,  shoulder: 0,  elbow: 90  },
    HAPPY:    { base: 100, shoulder: 20, elbow: 120 },
    SAD:      { base: 80,  shoulder: 50, elbow: 60  },
    ANGRY:    { base: 110, shoulder: 35, elbow: 40  },
    SURPRISE: { base: 90,  shoulder: 5,  elbow: 150 },
    EXCITED:  { base: 100, shoulder: 10, elbow: 140 },
    BORED:    { base: 95,  shoulder: 10, elbow: 80  }
  };

  // ===== CURRENT STATE (servo angles) =====
  let servoBase     = 90;
  let servoShoulder = 0;
  let servoElbow    = 90;
  let mode = 'fk';
  let animating = false;

  // ===== THREE.JS SETUP =====
  const container = document.getElementById('sim-3d');
  if (!container) return;

  const W = 560, H = 420;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0d1219);

  const camera = new THREE.PerspectiveCamera(45, W / H, 0.1, 1000);
  camera.position.set(22, 18, 22);
  camera.lookAt(0, 8, 0);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(W, H);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  // Orbit controls
  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 8, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 12;
  controls.maxDistance = 60;
  controls.update();

  // ===== LIGHTS =====
  const ambientLight = new THREE.AmbientLight(0x404060, 0.6);
  scene.add(ambientLight);

  const mainLight = new THREE.DirectionalLight(0xffffff, 0.8);
  mainLight.position.set(15, 25, 15);
  mainLight.castShadow = true;
  mainLight.shadow.mapSize.width = 1024;
  mainLight.shadow.mapSize.height = 1024;
  scene.add(mainLight);

  const fillLight = new THREE.DirectionalLight(0x00ffc8, 0.2);
  fillLight.position.set(-10, 10, -10);
  scene.add(fillLight);

  // ===== GROUND + GRID =====
  const groundGeo = new THREE.PlaneGeometry(40, 40);
  const groundMat = new THREE.MeshStandardMaterial({
    color: 0x111820,
    metalness: 0.3,
    roughness: 0.8
  });
  const ground = new THREE.Mesh(groundGeo, groundMat);
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  const gridHelper = new THREE.GridHelper(40, 20, 0x1a2535, 0x151c25);
  scene.add(gridHelper);

  // Axis lines
  const axisLen = 12;
  const xAxisGeo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0.02, 0), new THREE.Vector3(axisLen, 0.02, 0)
  ]);
  const yAxisGeo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, axisLen, 0)
  ]);
  const zAxisGeo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0.02, 0), new THREE.Vector3(0, 0.02, axisLen)
  ]);
  scene.add(new THREE.Line(xAxisGeo, new THREE.LineBasicMaterial({ color: 0xff4444, opacity: 0.4, transparent: true })));
  scene.add(new THREE.Line(yAxisGeo, new THREE.LineBasicMaterial({ color: 0x44ff44, opacity: 0.4, transparent: true })));
  scene.add(new THREE.Line(zAxisGeo, new THREE.LineBasicMaterial({ color: 0x4444ff, opacity: 0.4, transparent: true })));

  // ===== MATERIALS =====
  const matBase = new THREE.MeshStandardMaterial({
    color: 0x253545, metalness: 0.6, roughness: 0.4
  });
  const matJoint = new THREE.MeshStandardMaterial({
    color: 0x00ffc8, metalness: 0.3, roughness: 0.3, emissive: 0x00ffc8, emissiveIntensity: 0.15
  });
  const matUpperArm = new THREE.MeshStandardMaterial({
    color: 0x00cc99, metalness: 0.4, roughness: 0.4
  });
  const matForearm = new THREE.MeshStandardMaterial({
    color: 0x0077cc, metalness: 0.4, roughness: 0.4
  });
  const matEE = new THREE.MeshStandardMaterial({
    color: 0xff3366, metalness: 0.3, roughness: 0.3, emissive: 0xff3366, emissiveIntensity: 0.3
  });
  const matOLED = new THREE.MeshStandardMaterial({
    color: 0x000000, metalness: 0.1, roughness: 0.9
  });
  const matOLEDFrame = new THREE.MeshStandardMaterial({
    color: 0x1a2030, metalness: 0.5, roughness: 0.5
  });

  // ===== BUILD ARM HIERARCHY =====
  // Base platform (sits on ground)
  const basePlatformGeo = new THREE.CylinderGeometry(2.5, 3.0, 1.0, 32);
  const basePlatform = new THREE.Mesh(basePlatformGeo, matBase);
  basePlatform.position.y = 0.5;
  basePlatform.castShadow = true;
  scene.add(basePlatform);

  // Base rotation pivot (rotates around Y)
  const basePivot = new THREE.Group();
  basePivot.position.y = 1.0;
  basePlatform.add(basePivot);

  // Base column (from platform up to shoulder)
  const columnGeo = new THREE.CylinderGeometry(1.0, 1.2, SHOULDER_HEIGHT - 1.0, 16);
  const column = new THREE.Mesh(columnGeo, matBase);
  column.position.y = (SHOULDER_HEIGHT - 1.0) / 2;
  column.castShadow = true;
  basePivot.add(column);

  // Shoulder joint ball
  const shoulderBallGeo = new THREE.SphereGeometry(1.1, 16, 16);
  const shoulderBall = new THREE.Mesh(shoulderBallGeo, matJoint);
  shoulderBall.position.y = SHOULDER_HEIGHT - 1.0;
  shoulderBall.castShadow = true;
  basePivot.add(shoulderBall);

  // Shoulder pivot (rotates around X — lifts arm)
  const shoulderPivot = new THREE.Group();
  shoulderPivot.position.y = SHOULDER_HEIGHT - 1.0;
  basePivot.add(shoulderPivot);

  // Upper arm link
  const upperArmGeo = new THREE.CylinderGeometry(0.55, 0.55, UPPER_ARM_LEN, 12);
  const upperArm = new THREE.Mesh(upperArmGeo, matUpperArm);
  // Upper arm extends along local Y
  upperArm.position.y = UPPER_ARM_LEN / 2;
  upperArm.castShadow = true;
  shoulderPivot.add(upperArm);

  // Elbow joint ball
  const elbowBallGeo = new THREE.SphereGeometry(0.85, 16, 16);
  const elbowBall = new THREE.Mesh(elbowBallGeo, matJoint);
  elbowBall.position.y = UPPER_ARM_LEN;
  elbowBall.castShadow = true;
  shoulderPivot.add(elbowBall);

  // Elbow pivot (rotates around X — flexes forearm)
  const elbowPivot = new THREE.Group();
  elbowPivot.position.y = UPPER_ARM_LEN;
  shoulderPivot.add(elbowPivot);

  // Forearm link
  const forearmGeo = new THREE.CylinderGeometry(0.45, 0.45, FOREARM_LEN, 12);
  const forearm = new THREE.Mesh(forearmGeo, matForearm);
  forearm.position.y = FOREARM_LEN / 2;
  forearm.castShadow = true;
  elbowPivot.add(forearm);

  // End-effector / gripper tip
  const eeGeo = new THREE.SphereGeometry(0.7, 12, 12);
  const eeMesh = new THREE.Mesh(eeGeo, matEE);
  eeMesh.position.y = FOREARM_LEN;
  eeMesh.castShadow = true;
  elbowPivot.add(eeMesh);

  // OLED display on end-effector
  const oledFrameGeo = new THREE.BoxGeometry(2.8, 1.6, 0.3);
  const oledFrame = new THREE.Mesh(oledFrameGeo, matOLEDFrame);
  oledFrame.position.y = FOREARM_LEN + 0.2;
  oledFrame.position.z = 0.7;
  elbowPivot.add(oledFrame);

  const oledScreenGeo = new THREE.PlaneGeometry(2.4, 1.2);
  const oledScreenMat = new THREE.MeshBasicMaterial({ color: 0x000000 });
  const oledScreen = new THREE.Mesh(oledScreenGeo, oledScreenMat);
  oledScreen.position.y = FOREARM_LEN + 0.2;
  oledScreen.position.z = 0.86;
  elbowPivot.add(oledScreen);

  // OLED face canvas (for rendering emotion faces)
  const oledCanvas = document.createElement('canvas');
  oledCanvas.width = 128;
  oledCanvas.height = 64;
  const oledCtx = oledCanvas.getContext('2d');
  const oledTexture = new THREE.CanvasTexture(oledCanvas);
  oledScreenMat.map = oledTexture;
  oledScreenMat.needsUpdate = true;

  // ===== OLED FACE RENDERING (matching Arduino pixel art) =====
  function drawOLEDFace(emotion) {
    const c = oledCtx;
    c.fillStyle = '#000000';
    c.fillRect(0, 0, 128, 64);
    c.fillStyle = '#00ffc8';
    c.strokeStyle = '#00ffc8';
    c.lineWidth = 2;

    switch (emotion) {
      case 'HAPPY':
        // Eyes
        c.beginPath(); c.arc(40, 24, 6, 0, Math.PI * 2); c.fill();
        c.beginPath(); c.arc(88, 24, 6, 0, Math.PI * 2); c.fill();
        // Smile
        c.beginPath();
        c.moveTo(45, 42); c.lineTo(55, 50); c.lineTo(73, 50); c.lineTo(83, 42);
        c.stroke();
        break;

      case 'EXCITED':
        // Big eyes
        c.beginPath(); c.arc(40, 22, 8, 0, Math.PI * 2); c.fill();
        c.beginPath(); c.arc(88, 22, 8, 0, Math.PI * 2); c.fill();
        // Big smile
        c.beginPath();
        c.moveTo(40, 42); c.quadraticCurveTo(64, 60, 88, 42);
        c.stroke();
        break;

      case 'SAD':
        // Eyes
        c.beginPath(); c.arc(40, 24, 6, 0, Math.PI * 2); c.fill();
        c.beginPath(); c.arc(88, 24, 6, 0, Math.PI * 2); c.fill();
        // Frown
        c.beginPath();
        c.moveTo(45, 52); c.lineTo(55, 44); c.lineTo(73, 44); c.lineTo(83, 52);
        c.stroke();
        break;

      case 'ANGRY':
        // Eyes
        c.beginPath(); c.arc(40, 26, 6, 0, Math.PI * 2); c.fill();
        c.beginPath(); c.arc(88, 26, 6, 0, Math.PI * 2); c.fill();
        // Angry eyebrows
        c.beginPath(); c.moveTo(30, 18); c.lineTo(50, 12); c.stroke();
        c.beginPath(); c.moveTo(78, 12); c.lineTo(98, 18); c.stroke();
        // Flat mouth
        c.beginPath(); c.moveTo(44, 46); c.lineTo(84, 46); c.stroke();
        break;

      case 'SURPRISE':
        // Wide eyes
        c.beginPath(); c.arc(40, 22, 6, 0, Math.PI * 2); c.fill();
        c.beginPath(); c.arc(88, 22, 6, 0, Math.PI * 2); c.fill();
        // Open mouth
        c.beginPath(); c.arc(64, 48, 8, 0, Math.PI * 2); c.stroke();
        break;

      case 'BORED':
        // Half-closed eyes (lines)
        c.beginPath(); c.moveTo(34, 24); c.lineTo(46, 24); c.stroke();
        c.beginPath(); c.moveTo(82, 24); c.lineTo(94, 24); c.stroke();
        // Flat mouth
        c.beginPath(); c.moveTo(44, 46); c.lineTo(84, 46); c.stroke();
        break;

      default: // NEUTRAL
        // Eyes
        c.beginPath(); c.arc(40, 24, 6, 0, Math.PI * 2); c.fill();
        c.beginPath(); c.arc(88, 24, 6, 0, Math.PI * 2); c.fill();
        // Straight mouth
        c.beginPath(); c.moveTo(44, 46); c.lineTo(84, 46); c.stroke();
    }

    oledTexture.needsUpdate = true;
  }

  // ===== SERVO-TO-3D ANGLE CONVERSION =====
  // The real servos:
  //   Base servo 90° = facing forward, < 90 = right, > 90 = left
  //   Shoulder servo 0° = arm pointing straight up, 60° = tilted down/forward
  //   Elbow servo 90° = straight continuation, < 90 = bent back, > 90 = bent forward
  function updateArmPose() {
    // Base: servo 90 = no rotation. Map (60-120) to rotation around Y.
    // Servo value < 90 = rotate right (+Z direction for us), > 90 = rotate left
    const baseAngle = -(servoBase - 90) * (Math.PI / 180);
    basePivot.rotation.y = baseAngle;

    // Shoulder: servo 0 = arm pointing straight up (along Y).
    // servo 60 = arm tilted 60° from vertical toward horizontal.
    // In Three.js: rotate shoulderPivot around local X (tilt forward/away from body).
    // At servo 0: no tilt (arm up). At servo 60: 60° tilt.
    const shoulderAngle = servoShoulder * (Math.PI / 180);
    shoulderPivot.rotation.x = shoulderAngle;

    // Elbow: servo 90 = forearm continues straight from upper arm.
    // servo 30 = bent 60° backward, servo 150 = bent 60° forward.
    // Relative to upper arm direction.
    const elbowAngle = (servoElbow - 90) * (Math.PI / 180);
    elbowPivot.rotation.x = elbowAngle;
  }

  // ===== UI ELEMENTS =====
  const sliderT1 = document.getElementById('slider-t1');
  const sliderT2 = document.getElementById('slider-t2');
  const sliderT3 = document.getElementById('slider-t3');
  const valT1 = document.getElementById('val-t1');
  const valT2 = document.getElementById('val-t2');
  const valT3 = document.getElementById('val-t3');
  const serialCmd = document.getElementById('serial-cmd');

  function updateUI() {
    if (valT1) valT1.textContent = servoBase + '°';
    if (valT2) valT2.textContent = servoShoulder + '°';
    if (valT3) valT3.textContent = servoElbow + '°';
    if (serialCmd) serialCmd.textContent = `B:${servoBase} S:${servoShoulder} E:${servoElbow}`;
  }

  function updateSliders() {
    if (sliderT1) sliderT1.value = servoBase;
    if (sliderT2) sliderT2.value = servoShoulder;
    if (sliderT3) sliderT3.value = servoElbow;
    updateUI();
  }

  function getEmotionFromPose() {
    for (const [name, p] of Object.entries(PRESETS)) {
      if (p.base === servoBase && p.shoulder === servoShoulder && p.elbow === servoElbow) {
        return name;
      }
    }
    return 'NEUTRAL';
  }

  // ===== SLIDER INPUT HANDLERS =====
  function onSliderInput() {
    if (animating) return;
    servoBase = parseInt(sliderT1.value);
    servoShoulder = parseInt(sliderT2.value);
    servoElbow = parseInt(sliderT3.value);
    clearPresetActive();
    updateArmPose();
    updateUI();
    drawOLEDFace(getEmotionFromPose());
  }

  if (sliderT1) sliderT1.addEventListener('input', onSliderInput);
  if (sliderT2) sliderT2.addEventListener('input', onSliderInput);
  if (sliderT3) sliderT3.addEventListener('input', onSliderInput);

  // ===== ANIMATION (smooth servo movement like Arduino) =====
  function animateTo(targetBase, targetShoulder, targetElbow, emotion) {
    animating = true;
    const startBase = servoBase;
    const startShoulder = servoShoulder;
    const startElbow = servoElbow;
    const duration = 700;
    const startTime = performance.now();

    function easeInOutCubic(t) {
      return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
    }

    function step(now) {
      const elapsed = now - startTime;
      const progress = Math.min(elapsed / duration, 1);
      const eased = easeInOutCubic(progress);

      servoBase = Math.round(startBase + (targetBase - startBase) * eased);
      servoShoulder = Math.round(startShoulder + (targetShoulder - startShoulder) * eased);
      servoElbow = Math.round(startElbow + (targetElbow - startElbow) * eased);

      updateArmPose();
      updateSliders();

      if (progress < 1) {
        requestAnimationFrame(step);
      } else {
        servoBase = targetBase;
        servoShoulder = targetShoulder;
        servoElbow = targetElbow;
        updateArmPose();
        updateSliders();
        drawOLEDFace(emotion || getEmotionFromPose());
        animating = false;
      }
    }

    requestAnimationFrame(step);
  }

  // ===== GLOBAL FUNCTIONS =====
  window.setMode = function (m) {
    mode = m;
    document.getElementById('btn-fk').classList.toggle('active', m === 'fk');
    document.getElementById('btn-ik').classList.toggle('active', m === 'ik');
  };

  window.setEmotionPreset = function (emotion) {
    const preset = PRESETS[emotion];
    if (!preset) return;
    clearPresetActive();

    document.querySelectorAll('.preset-btn').forEach(btn => {
      if (btn.textContent.toUpperCase().includes(emotion)) {
        btn.classList.add('active');
      }
    });

    animateTo(preset.base, preset.shoulder, preset.elbow, emotion);
  };

  function clearPresetActive() {
    document.querySelectorAll('.preset-btn').forEach(btn => btn.classList.remove('active'));
  }

  // ===== RENDER LOOP =====
  function render() {
    requestAnimationFrame(render);
    controls.update();

    // Gentle glow pulse on end-effector
    const pulse = 0.2 + 0.1 * Math.sin(Date.now() * 0.003);
    matEE.emissiveIntensity = pulse;

    renderer.render(scene, camera);
  }

  // ===== RESPONSIVE =====
  function onResize() {
    const wrap = container.parentElement;
    if (!wrap) return;
    const w = Math.min(wrap.clientWidth, 560);
    const h = Math.round(w * 420 / 560);
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', onResize);

  // ===== INIT =====
  updateArmPose();
  updateSliders();
  drawOLEDFace('NEUTRAL');
  render();

})();
