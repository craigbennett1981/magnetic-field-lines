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

// ---------- timeline history: record / restore / scrub ----------
// One lightweight snapshot per recorded point — pose plus soft/eddy
// magnetization state (cellM/edD/edBprev/edInit), NOT field-line geometry.
// Field lines get re-traced on demand when scrubbing to a point (fast,
// especially with the GPU tracer) rather than stored, which keeps history
// memory cheap even over a long recording (the alternative — storing full
// line geometry per point too — would cost tens of MB instead of a few).
// Soft/eddy state has to be captured too, not just pos/quat: buildSources()
// derives the field sources bodies actually emit from cellM/edD, so a scrub
// target that only restored pose would show CURRENT (wrong) magnetization
// on a historical rotor/plate instead of what it actually had at that time.
function snapshotEntity(e){
  return {
    pos:[...e.pos], quat:{x:e.quat.x,y:e.quat.y,z:e.quat.z,w:e.quat.w},
    pos0:[...e.pos0], quat0:{x:e.quat0.x,y:e.quat0.y,z:e.quat0.z,w:e.quat0.w},
    tRef:e.tRef, vel:[...e.vel], omega:[...e.omega],
  };
}
function recordHistoryPoint(){
  // if we're recording from somewhere other than the live edge (shouldn't
  // normally happen — play() already truncates on resume — but guard
  // against it anyway) drop any stale "future" first so history stays one
  // linear timeline, never a tree.
  if(ST.historyIndex>=0 && ST.historyIndex<ST.history.length-1) ST.history.length=ST.historyIndex+1;
  ST.history.push({
    simTime: ST.simTime,
    bodies: ST.bodies.map(b=>({
      ...snapshotEntity(b),
      cellM:b.cellM.slice(), edD:b.edD.slice(), edBprev:b.edBprev.slice(), edInit:b.edInit,
      stuckWithIdx: b._stuckWith ? ST.bodies.indexOf(b._stuckWith) : -1,
    })),
    groups: ST.groups.map(g=>({...snapshotEntity(g), stuckWithIdx:-1})),
  });
  ST.historyIndex=ST.history.length-1;
  ST.lastHistorySimTime=ST.simTime;
  updateScrubUI();
}
// throttles recording to ~100 points across the current maxTime, in SIM
// time (not real time or step count), so granularity stays reasonable
// however fast/slow playback or lockstep cycles are running
function recordHistoryPointIfDue(){
  const interval=ST.maxTime>0?ST.maxTime/100:0.1;
  if(ST.history.length===0||ST.simTime-ST.lastHistorySimTime>=interval) recordHistoryPoint();
}
// (re)starts the timeline from the current state — called once bodies are
// freshly built/reset, so history[0] is always "t=0, as-loaded/as-reset"
export function resetHistory(){
  ST.history=[]; ST.historyIndex=-1; ST.lastHistorySimTime=0;
  recordHistoryPoint();
}
export function restoreHistoryPoint(idx){
  idx=Math.max(0,Math.min(ST.history.length-1,idx));
  const snap=ST.history[idx];
  if(!snap) return;
  ST.simTime=snap.simTime;
  snap.bodies.forEach((s,bi)=>{
    const b=ST.bodies[bi]; if(!b) return;
    b.pos=[...s.pos]; b.pos0=[...s.pos0]; b.tRef=s.tRef;
    b.quat.set(s.quat.x,s.quat.y,s.quat.z,s.quat.w);
    b.quat0.set(s.quat0.x,s.quat0.y,s.quat0.z,s.quat0.w);
    b.vel=[...s.vel]; b.omega=[...s.omega];
    b.cellM.set(s.cellM); b.edD.set(s.edD); b.edBprev.set(s.edBprev); b.edInit=s.edInit;
  });
  // resolve stuck-contact partners after every body's own state is in place
  snap.bodies.forEach((s,bi)=>{ if(ST.bodies[bi]) ST.bodies[bi]._stuckWith = s.stuckWithIdx>=0?ST.bodies[s.stuckWithIdx]:null; });
  snap.groups.forEach((s,gi)=>{
    const g=ST.groups[gi]; if(!g) return;
    g.pos=[...s.pos]; g.pos0=[...s.pos0]; g.tRef=s.tRef;
    g.quat.set(s.quat.x,s.quat.y,s.quat.z,s.quat.w);
    g.quat0.set(s.quat0.x,s.quat0.y,s.quat0.z,s.quat0.w);
    g.vel=[...s.vel]; g.omega=[...s.omega];
  });
  ST.historyIndex=idx;
  buildSources(); softSolve(2);
  ST.bodies.forEach(updateArrow);
  syncMeshes();
  timebar.textContent='t = '+ST.simTime.toFixed(3)+' s (scrub)';
  updateScrubUI();
}
// drag/scrub to an arbitrary time — snaps to the nearest recorded point
// (recorded ~100 times across maxTime, so this is fine-grained enough to
// feel continuous) and kicks a fresh live-quality retrace for it, since the
// stored snapshot has no field-line geometry of its own.
export function scrubTo(targetSimTime){
  if(!ST.history.length) return;
  if(ST.playing){
    // implicit pause: dragging the timeline while playing should stop live
    // advancement, same as a video scrubber — but skip pause()'s own
    // buildSources/retrace since restoreHistoryPoint below immediately
    // does that anyway against the scrubbed-to state instead.
    ST.playing=false; ST.lastAdvanceReal=null;
    const p=document.getElementById('play'); if(p) p.innerHTML='&#9654; Play';
  }
  let best=0,bestDiff=Infinity;
  for(let i=0;i<ST.history.length;i++){
    const d=Math.abs(ST.history[i].simTime-targetSimTime);
    if(d<bestDiff){ bestDiff=d; best=i; }
  }
  restoreHistoryPoint(best);
  // If a trace is already in flight (from an earlier scrub position, or a
  // live cycle that was still running when scrubbing started), don't pile
  // up another overlapping worker job — dragging fast could otherwise queue
  // up many full traces the workers would have to grind through one at a
  // time. Instead flag that the current position needs its own trace once
  // the in-flight one lands; tracer.js's onTraceWorkerMessage checks this
  // flag and skips rendering that now-stale result in favor of an
  // immediate re-trace, so the displayed lines never get stuck showing an
  // earlier scrub position with nothing left to correct them.
  if(ST.tracing) ST.scrubTraceWanted=true;
  else traceAll(false);
}
// True only while the user has an actual pointer down on the bar — NOT the
// same as "the bar has DOM focus": a native <input type=range> keeps focus
// after a click/drag ends (until something else is clicked), so gating the
// live-sync update on document.activeElement instead of this flag meant the
// very first interaction silently froze the bar out of sync forever after.
let scrubDragging=false;
function updateScrubUI(){
  const bar=document.getElementById('scrubBar');
  if(!bar) return;
  const maxT=ST.history.length?ST.history[ST.history.length-1].simTime:0;
  bar.min='0'; bar.max=String(maxT||0.001); bar.step=String((maxT/200)||0.01);
  if(!scrubDragging) bar.value=String(ST.simTime);
  // stays enabled even with only one recorded point (e.g. right after
  // load/reset, before Play has run) — a disabled range reads as broken;
  // scrubbing with nothing yet to scrub through is just a harmless no-op.
  const lbl=document.getElementById('scrubLabel');
  if(lbl) lbl.textContent='t = '+ST.simTime.toFixed(2)+' / '+maxT.toFixed(2)+' s';
}

// ---------- main loop ----------
// true once nothing is (or could still be) actively changing the field:
// no kinematic motion running, no free body/group with meaningful
// velocity/omega, and no soft/eddy state still relaxing. Conservative by
// design — anything ambiguous reports "not settled" so a live retrace
// still happens rather than risking a stale picture. Soft/eddy scenes never
// self-report settled by this definition, so for those the maxTime cap
// below is what actually bounds playback.
export function sceneIsSettled(){
  if(ST.anySoft||ST.anyEddy) return false;
  const moving=e=>e.motion.mode!=='static'&&e.motion.mode!=='free'
    ||(e.motion.mode==='free'&&(Math.hypot(...e.vel)>1e-4||Math.hypot(...e.omega)>1e-4));
  for(const b of ST.bodies){ if(b.groupId==null&&moving(b)) return false; }
  for(const g of ST.groups){ if(moving(g)) return false; }
  return true;
}
// Records a history point for the pose just advanced to, then auto-stops
// playback (full pause — button reverts, motion freezes) once the scene has
// settled or simTime has reached the configurable max time. Returns true if
// playback was stopped, so callers can skip any further work (e.g. kicking
// a live retrace) for this tick.
function afterAdvance(){
  recordHistoryPointIfDue();
  if(ST.simTime>=ST.maxTime||sceneIsSettled()){ pause(); return true; }
  return false;
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
    if(afterAdvance()) return;
  }
  if(!document.getElementById('live').checked) return;
  setStatus('computing…',true); traceAll(false);
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
      afterAdvance();
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
  // resuming from a scrubbed-back point abandons whatever "future" history
  // had been recorded past it — playback from here on generates a fresh
  // one (identical to the old one if nothing was edited, since physics is
  // deterministic, but there's no redo-branch bookkeeping to maintain)
  if(ST.historyIndex>=0&&ST.historyIndex<ST.history.length-1) ST.history.length=ST.historyIndex+1;
  ST.lastHistorySimTime=ST.simTime;
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
  resetHistory();
  buildSources(); softSolve(24); ST.bodies.forEach(updateArrow); syncMeshes(); requestTrace(true);
});
document.getElementById('units').addEventListener('change',e=>{SIM.u=+e.target.value; refreshDerived();});
document.getElementById('speed').addEventListener('change',e=>{SIM.speed=+e.target.value;});
document.getElementById('gravity').addEventListener('change',e=>{SIM.gravity=e.target.checked;});
document.getElementById('lines').addEventListener('input',e=>{
  document.getElementById('linesVal').textContent=e.target.value;});
document.getElementById('trace').addEventListener('click',()=>requestTrace(true));
document.getElementById('maxTime').addEventListener('change',e=>{
  const v=+e.target.value;
  ST.maxTime=v>0?v:10;
  updateScrubUI();
});
{
  const scrubBar=document.getElementById('scrubBar');
  scrubBar.addEventListener('pointerdown',()=>{ scrubDragging=true; });
  scrubBar.addEventListener('pointerup',()=>{ scrubDragging=false; });
  scrubBar.addEventListener('pointercancel',()=>{ scrubDragging=false; });
  // keyboard interaction (arrow keys, click-to-focus-then-arrow) never fires
  // pointerdown/up, so fall back to clearing the flag once no more 'input'
  // events have landed for a moment
  let keyboardScrubTimer=null;
  scrubBar.addEventListener('input',e=>{
    scrubTo(+e.target.value);
    if(!scrubDragging){
      clearTimeout(keyboardScrubTimer);
      scrubDragging=true;
      keyboardScrubTimer=setTimeout(()=>{ scrubDragging=false; },200);
    }
  });
}
