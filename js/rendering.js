// Mesh/arrow sync with physics state, and the main render/lockstep loop.
//
// Lockstep simulation clock: frame() always renders every tick (so the
// camera/orbit stay responsive), but only steps physics and kicks the next
// trace once the *previous* trace has landed — driven from trace completion
// (maybeContinueLockstep, called from tracer.js's onTraceWorkerMessage), not
// the rAF tick. This makes "lines out of sync with the object" structurally
// impossible: the object never advances to a pose whose field lines aren't
// ready yet. The honest trade-off is that motion becomes stepped, at a rate
// bounded by trace speed, instead of always continuously smooth.
import {ST, SIM, MATERIALS, timebar, setStatus} from './state.js';
import {scene, camera, renderer} from './scene.js';
import {step, buildSources, softSolve, syncGroupMembers, arrowVec, refreshDerived} from './physics.js';
import {traceAll, requestTrace} from './tracer.js';

export function syncMeshes(){
  for(const b of ST.bodies){
    b.mesh.position.set(...b.pos);
    b.mesh.quaternion.copy(b.quat);
    if(b.arrow){
      b.arrow.position.set(...b.pos);
      const v=arrowVec(b);
      const l=Math.hypot(...v);
      if(l>1e-12) b.arrow.setDirection(new THREE.Vector3(v[0]/l,v[1]/l,v[2]/l));
    }
  }
}
export function updateArrow(b){
  if(b.arrow){scene.remove(b.arrow); b.arrow=null;}
  const v=arrowVec(b), l=Math.hypot(...v);
  if(l<1e-12) return;
  const mat=MATERIALS[b.material];
  b.arrow=new THREE.ArrowHelper(
    new THREE.Vector3(v[0]/l,v[1]/l,v[2]/l), new THREE.Vector3(...b.pos),
    b.arrowLen, mat.cat==='perm'?0xffffff:0x9ab6cc, b.arrowLen*0.25, b.arrowLen*0.12);
  scene.add(b.arrow);
}

// ---------- main loop ----------
// true once nothing is (or could still be) actively changing the field:
// no kinematic motion running, no free body/group with meaningful
// velocity/omega, and no soft/eddy state still relaxing. Conservative by
// design — anything ambiguous reports "not settled" so a live retrace
// still happens rather than risking a stale picture.
export function sceneIsSettled(){
  if(ST.anySoft||ST.anyEddy) return false;
  const moving=e=>e.motion.mode!=='static'&&e.motion.mode!=='free'
    ||(e.motion.mode==='free'&&(Math.hypot(...e.vel)>1e-4||Math.hypot(...e.omega)>1e-4));
  for(const b of ST.bodies){ if(b.groupId==null&&moving(b)) return false; }
  for(const g of ST.groups){ if(moving(g)) return false; }
  return true;
}
// Gates physics/motion advancement itself on trace completion instead of
// letting motion run free every rAF tick while lines asynchronously "catch
// up" (the approach every prior fix tried and which could never fully
// remove the lag). The object simply does not move to its next pose until
// the field lines for that pose have finished computing, so the two can
// never visibly disagree — at the cost of motion being stepped (at a rate
// bounded by trace speed) rather than always continuously smooth. Called
// both to bootstrap/self-heal from frame()'s rAF tick (when idle) and,
// immediately on trace completion, from maybeContinueLockstep() — the
// latter is the real driver, since it fires as soon as a cycle's lines are
// ready rather than waiting for the next tick.
export function advanceAndRetrace(){
  if(!ST.playing||!ST.bodies.length) return;
  const now=performance.now();
  const dtElapsed=ST.lastAdvanceReal==null?0:Math.min(0.5,(now-ST.lastAdvanceReal)/1000);
  ST.lastAdvanceReal=now;
  if(dtElapsed>0){
    const dt=dtElapsed*SIM.speed;
    // substep count now scales with a whole lockstep cycle's real duration
    // rather than one ~16ms rAF tick, so the old cap of 4 (sized for
    // 60fps-tick dt) would blow up h and destabilize the integrator; keep h
    // close to SIM.h regardless of how long the cycle took.
    const n=Math.max(1,Math.min(500,Math.round(dt/SIM.h)));
    const h=dt/n;
    for(let i=0;i<n;i++) step(h);
    syncMeshes();
    timebar.textContent='t = '+ST.simTime.toFixed(3)+' s  ('+SIM.speed+'×)';
  }
  if(!document.getElementById('live').checked) return;
  const settled=sceneIsSettled();
  if(!settled){ setStatus('computing…',true); traceAll(false); }
  else if(!ST.wasSettled) traceAll(true);   // just came to rest — upgrade to one full-quality trace
  ST.wasSettled=settled;
}
export function maybeContinueLockstep(){
  if(ST.playing&&document.getElementById('live').checked) advanceAndRetrace();
}
export function frame(ts){
  requestAnimationFrame(frame);
  const dtReal=Math.min(0.05,(ts-ST.lastTS)/1000||0); ST.lastTS=ts;
  if(ST.playing&&ST.bodies.length){
    if(!document.getElementById('live').checked){
      // Live tracing is off, so there's no in-flight trace to fall out of
      // sync with (lines only refresh on pause / the manual Trace button) —
      // motion can just run smoothly every tick, as before this change.
      let dt=dtReal*SIM.speed;
      const n=Math.max(1,Math.min(4,Math.round(dt/SIM.h)));
      const h=dt/n;
      for(let i=0;i<n;i++) step(h);
      syncMeshes();
      timebar.textContent='t = '+ST.simTime.toFixed(3)+' s  ('+SIM.speed+'×)';
    } else if(!ST.tracing){
      // Lockstep: only advance once the previous live trace has landed.
      // In practice onTraceWorkerMessage's maybeContinueLockstep() already
      // re-kicks the next cycle the instant a trace completes, so this
      // call is mostly a bootstrap (first play) / self-heal (chain stalled
      // because the scene just started moving again after settling).
      advanceAndRetrace();
    }
  }
  renderer.render(scene,camera);
}
requestAnimationFrame(frame);

export function play(){
  if(!ST.bodies.length) return;
  ST.playing=true;
  document.getElementById('play').innerHTML='&#10074;&#10074; Pause';
  ST.bodies.forEach(b=>{if(b.isEddy)b.edInit=false;});
}
export function pause(){
  ST.playing=false;
  ST.lastAdvanceReal=null;   // don't let paused wall-clock time count as a catch-up jump on resume
  const p=document.getElementById('play');
  if(p) p.innerHTML='&#9654; Play';
  if(ST.bodies.length){ buildSources(); softSolve(10); ST.bodies.forEach(updateArrow); syncMeshes(); requestTrace(true); }
}
document.getElementById('play').addEventListener('click',()=>ST.playing?pause():play());
document.getElementById('reset').addEventListener('click',()=>{
  pause(); ST.simTime=0; timebar.textContent='';
  for(const b of ST.bodies){
    b.pos=[...b.homePos]; b.pos0=[...b.homePos];
    b.quat.copy(b.homeQuat); b.quat0.copy(b.homeQuat); b.tRef=0;
    b.vel=[0,0,0]; b.omega=[0,0,0]; b._stuckWith=null;
    b.cellM.fill(0); b.edD.fill(0); b.edInit=false;
  }
  // groups reset their own pose too, then overwrite members' pos/quat from
  // above with the group-derived ones — a grouped body's individual
  // homePos would otherwise break its fixed offset within the group
  for(const g of ST.groups){
    g.pos=[...g.homePos]; g.pos0=[...g.homePos];
    g.quat.copy(g.homeQuat); g.quat0.copy(g.homeQuat); g.tRef=0;
    g.vel=[0,0,0]; g.omega=[0,0,0]; g._stuckWith=null;
    syncGroupMembers(g);
  }
  buildSources(); softSolve(24); ST.bodies.forEach(updateArrow); syncMeshes(); requestTrace(true);
});
document.getElementById('units').addEventListener('change',e=>{SIM.u=+e.target.value; refreshDerived();});
document.getElementById('speed').addEventListener('change',e=>{SIM.speed=+e.target.value;});
document.getElementById('gravity').addEventListener('change',e=>{SIM.gravity=e.target.checked;});
document.getElementById('lines').addEventListener('input',e=>{
  document.getElementById('linesVal').textContent=e.target.value;});
document.getElementById('trace').addEventListener('click',()=>requestTrace(true));
