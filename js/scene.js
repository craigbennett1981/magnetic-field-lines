// Three.js renderer/scene/camera/lights + orbit-style mouse/wheel camera
// controls. Foundational — no simulation-state dependency at all, so every
// other module can import from here without risk of a circular reference.
export const viewport = document.getElementById('viewport');
export const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
viewport.appendChild(renderer.domElement);
export const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);
export const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 5000);
scene.add(new THREE.HemisphereLight(0x8899bb, 0x111418, 0.9));
const keyL = new THREE.DirectionalLight(0xffffff, 0.8); keyL.position.set(1,1.4,0.8); scene.add(keyL);
const fillL = new THREE.DirectionalLight(0x88aaff, 0.25); fillL.position.set(-1,-0.4,-0.8); scene.add(fillL);

export const ctrl = {theta:0.9, phi:1.05, radius:10, target:new THREE.Vector3()};
export function applyCamera(){
  const sp=Math.sin(ctrl.phi), cp=Math.cos(ctrl.phi);
  camera.position.set(
    ctrl.target.x+ctrl.radius*sp*Math.cos(ctrl.theta),
    ctrl.target.y+ctrl.radius*cp,
    ctrl.target.z+ctrl.radius*sp*Math.sin(ctrl.theta));
  camera.lookAt(ctrl.target);
}
let drag=null;
renderer.domElement.addEventListener('pointerdown', e=>{
  drag={x:e.clientX,y:e.clientY,pan:(e.shiftKey||e.button===2||e.button===1)};
  renderer.domElement.setPointerCapture(e.pointerId);
});
renderer.domElement.addEventListener('pointermove', e=>{
  if(!drag) return;
  const dx=e.clientX-drag.x, dy=e.clientY-drag.y; drag.x=e.clientX; drag.y=e.clientY;
  if(drag.pan){
    const s=ctrl.radius*0.0016;
    const right=new THREE.Vector3().setFromMatrixColumn(camera.matrix,0);
    const up=new THREE.Vector3().setFromMatrixColumn(camera.matrix,1);
    ctrl.target.addScaledVector(right,-dx*s).addScaledVector(up,dy*s);
  }else{
    ctrl.theta+=dx*0.006;
    ctrl.phi=Math.min(Math.PI-0.05,Math.max(0.05,ctrl.phi-dy*0.006));
  }
  applyCamera();
});
renderer.domElement.addEventListener('pointerup', ()=>drag=null);
renderer.domElement.addEventListener('wheel', e=>{
  e.preventDefault(); ctrl.radius*=Math.exp(e.deltaY*0.0011); applyCamera();
},{passive:false});
renderer.domElement.addEventListener('contextmenu', e=>e.preventDefault());
export function resize(){
  const w=viewport.clientWidth,h=viewport.clientHeight;
  renderer.setSize(w,h); camera.aspect=w/h; camera.updateProjectionMatrix();
}
addEventListener('resize',resize); resize();
