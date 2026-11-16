import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

export async function mountViewer(host, interactive = false) {
  host.innerHTML = '<div class="viewer-loading" role="status">Đang tải bản scan thật…</div>';
  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(40, 1, .05, 150);
    camera.up.set(0, 0, 1);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.minDistance = 2;
    controls.maxDistance = 35;
    controls.maxPolarAngle = Math.PI * .49;
    scene.add(new THREE.HemisphereLight(0xffffff, 0x62666d, 1.7));
    const sun = new THREE.DirectionalLight(0xffffff, 2.0);
    sun.position.set(-5, -8, 14); scene.add(sun);
    const fill = new THREE.DirectionalLight(0xe5e9ef, .8);
    fill.position.set(6, 6, 7); scene.add(fill);
    const [geometry, response] = await Promise.all([
      new STLLoader().loadAsync('/static/demo/room.stl'), fetch('/static/demo/trajectory.json')
    ]);
    if (!response.ok) throw new Error('Chưa có dữ liệu quỹ đạo.');
    const data = await response.json();
    if (!Array.isArray(data.frames) || !data.frames.length) throw new Error('Quỹ đạo chưa có khung hình.');
    const frames = data.frames;
    geometry.computeBoundingBox();
    const bounds = geometry.boundingBox, center = bounds.getCenter(new THREE.Vector3());
    const material = new THREE.MeshStandardMaterial({ color: 0x9aa0a6, roughness: .92, metalness: 0, side: THREE.DoubleSide });
    const mesh = new THREE.Mesh(geometry, material); scene.add(mesh);
    const grid = new THREE.GridHelper(20, 40, 0x7e858d, 0x989da2);
    grid.rotation.x = Math.PI / 2; grid.position.set(center.x, center.y, bounds.min.z - .025);
    grid.material.transparent = true; grid.material.opacity = .35; scene.add(grid);
    const points = frames.map(f => new THREE.Vector3(...f.p));
    const path = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), new THREE.LineBasicMaterial({ color: 0xb6662f, transparent: true, opacity: .7 }));
    scene.add(path);
    const marker = new THREE.Mesh(new THREE.SphereGeometry(.075, 16, 12), new THREE.MeshBasicMaterial({ color: 0xa65023, depthTest: false }));
    marker.renderOrder = 10; scene.add(marker);
    const direction = new THREE.ArrowHelper(new THREE.Vector3(1,0,0), points[0], .55, 0xa65023, .14, .09);
    scene.add(direction);
    let dirty = true, playing = false, current = 0, lastTime = 0, simulated = frames[0].t;
    const play = document.querySelector('#play'), seek = document.querySelector('#seek'), speed = document.querySelector('#speed');
    const frameLabel = document.querySelector('#frame-label'), timeLabel = document.querySelector('#time-label');
    function render() { dirty = true; }
    function pose(index) {
      current = Math.max(0, Math.min(index, frames.length - 1));
      marker.position.copy(points[current]); direction.position.copy(points[current]);
      direction.setDirection(new THREE.Vector3(...frames[current].d).normalize());
      if (interactive) {
        seek.value = current;
        frameLabel.textContent = `Khung ${(current + 1).toLocaleString('vi-VN')} / ${frames.length.toLocaleString('vi-VN')}`;
        timeLabel.textContent = `${(frames[current].t - frames[0].t).toFixed(1)} s / ${(frames.at(-1).t - frames[0].t).toFixed(1)} s`;
      }
      host.dataset.frame = current;
      render();
    }
    function reset(top = false) {
      controls.target.set(center.x, center.y, center.z * .55);
      camera.position.set(center.x + (top ? .001 : 8.8), center.y + (top ? -.001 : -10.8), center.z + (top ? 15 : 10.8));
      controls.update(); render();
    }
    function setPlaying(value) {
      playing = value;
      if (play) { play.textContent = value ? 'Ⅱ Dừng' : '▷ Phát'; play.setAttribute('aria-pressed', String(value)); }
      host.dataset.playing = String(value);
    }
    function resize() {
      const {width, height} = host.getBoundingClientRect();
      renderer.setSize(width, height); camera.aspect = width / height; camera.updateProjectionMatrix(); render();
    }
    host.replaceChildren(renderer.domElement);
    renderer.domElement.setAttribute('aria-label', 'Bản scan phòng 3D. Kéo chuột để xoay, cuộn để zoom. Các nút bên cạnh hỗ trợ đổi góc nhìn.');
    renderer.domElement.tabIndex = 0;
    renderer.domElement.addEventListener('keydown', e => {
      const movements = {ArrowLeft: [-.5,0,0], ArrowRight:[.5,0,0], ArrowUp:[0,0,.5], ArrowDown:[0,0,-.5]};
      if (movements[e.key]) { e.preventDefault(); camera.position.add(new THREE.Vector3(...movements[e.key])); controls.update(); render(); }
      if (e.key === '+' || e.key === '-') { e.preventDefault(); camera.position.sub(controls.target).multiplyScalar(e.key==='+'?.9:1.1).add(controls.target); controls.update(); render(); }
    });
    controls.addEventListener('change', render);
    new ResizeObserver(resize).observe(host);
    reset(); resize(); pose(0);
    if (interactive) {
      seek.max = frames.length - 1;
      for (const element of document.querySelectorAll('[data-viewer-control]')) element.disabled = false;
      document.querySelector('#reset-view').onclick = () => reset();
      document.querySelector('#top-view').onclick = () => reset(true);
      document.querySelector('#layer-mesh').onchange = e => { mesh.visible = e.target.checked; render(); };
      document.querySelector('#layer-route').onchange = e => { path.visible = marker.visible = direction.visible = e.target.checked; render(); };
      document.querySelector('#layer-grid').onchange = e => { grid.visible = e.target.checked; render(); };
      document.querySelector('#wireframe').onchange = e => { material.wireframe = e.target.checked; render(); };
      play.onclick = () => {
        if (current === frames.length - 1) { pose(0); simulated = frames[0].t; }
        setPlaying(!playing);
      };
      seek.oninput = () => { setPlaying(false); pose(Number(seek.value)); simulated = frames[current].t; };
    }
    document.addEventListener('visibilitychange', () => { if (document.hidden) setPlaying(false); });
    renderer.domElement.addEventListener('webglcontextlost', e => {
      e.preventDefault(); setPlaying(false);
      host.insertAdjacentHTML('beforeend', '<div class="viewer-error" role="alert">Trình xem mất kết nối GPU. <a class="btn" href="/explore">Tải lại trình xem</a></div>');
    });
    function animate(now) {
      const delta = Math.min((now-lastTime)/1000, .1); lastTime = now;
      if (!document.hidden) {
        if (playing) {
          simulated += delta * Number(speed.value);
          let next = current;
          while (next < frames.length - 1 && frames[next + 1].t <= simulated) next++;
          if (next !== current) pose(next);
          if (current === frames.length - 1) setPlaying(false);
        }
        if (controls.update()) dirty = true;
        if (dirty) { renderer.render(scene, camera); dirty = false; }
      }
      requestAnimationFrame(animate);
    }
    requestAnimationFrame(animate);
    host.dataset.ready = 'true';
  } catch (error) {
    renderer?.dispose();
    host.innerHTML = '<div class="viewer-error" role="alert"><strong>Chưa mở được bản scan 3D</strong><span>Cần WebGL và bộ dữ liệu demo đã đóng gói.</span><a class="btn small" href="/explore">Thử tải lại</a><a class="text-link" href="/library/report">Xem hình ảnh trong báo cáo ↗</a></div>';
    host.dataset.error = 'true';
    console.error('Viewer:', error);
  }
}
