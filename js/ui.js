// Body/group cards, file input (drag-drop + browse), and the procedurally
// generated demo scenes. This is also the app's effective entry point: it
// imports (transitively) every other module and wires up all the DOM event
// listeners that make the page interactive.
import {ST, MATERIALS, MAT_GROUPS, SIM, setStatus, APP_VERSION} from './state.js';
import {parseSTL} from './geometry.js';
import {refreshDerived, allocSources, buildSources, softSolve, updatePermQ,
        refreshGroupMassInertia, loadGeometry} from './physics.js';
import {updateArrow, syncMeshes} from './rendering.js';
import {requestTrace} from './tracer.js';

document.getElementById('appVersion').textContent = 'version '+APP_VERSION;

// ---------- body UI ----------
function matOptionsHTML(sel){
  let html='';
  for(const [g,label] of Object.entries(MAT_GROUPS)){
    html+='<optgroup label="'+label+'">';
    for(const [k,m] of Object.entries(MATERIALS))
      if(m.cat===g) html+='<option value="'+k+'"'+(k===sel?' selected':'')+'>'+m.name+'</option>';
    html+='</optgroup>';
  }
  return html;
}
function motionOptionsHTML(sel){
  const opts={static:'Static',free:'Free (dynamics)',spin:'Spin',osc:'Oscillate',slide:'Slide'};
  return Object.entries(opts).map(([k,v])=>'<option value="'+k+'"'+(k===sel?' selected':'')+'>'+v+'</option>').join('');
}

// ---------- groups: rigidly weld several bodies so they move as one ----------
export function createGroup(memberIdxs){
  if(memberIdxs.length<2) return;
  let totalMass=0, cx=0,cy=0,cz=0;
  for(const bi of memberIdxs){
    const b=ST.bodies[bi];
    totalMass+=b.mass; cx+=b.pos[0]*b.mass; cy+=b.pos[1]*b.mass; cz+=b.pos[2]*b.mass;
  }
  if(!(totalMass>0)){
    totalMass=memberIdxs.length; cx=cy=cz=0;
    for(const bi of memberIdxs){ const b=ST.bodies[bi]; cx+=b.pos[0]; cy+=b.pos[1]; cz+=b.pos[2]; }
  }
  const pos=[cx/totalMass, cy/totalMass, cz/totalMass];
  const g={
    id: ST.nextGroupId++,
    members: [...memberIdxs],
    motion:{mode:'static', axis:[0,1,0], rpm:120, amp:ST.sceneDiag*0.1, hz:1, v:[0,0,0]},
    pos, pos0:[...pos], quat:new THREE.Quaternion(), quat0:new THREE.Quaternion(), tRef:ST.simTime,
    homePos:[...pos], homeQuat:new THREE.Quaternion(),
    vel:[0,0,0], omega:[0,0,0],
    mass:1, invMass:1, I:1, invI:1, radius:1,
    _stuckWith:null
  };
  g.name='Group '+g.id;
  // group's own frame starts axis-aligned with world, so each member's
  // local offset is simply its current world offset from the group center
  for(const bi of memberIdxs){
    const b=ST.bodies[bi];
    b.groupId=g.id;
    b.localPos=[b.pos[0]-pos[0], b.pos[1]-pos[1], b.pos[2]-pos[2]];
    b.localQuat=b.quat.clone();
    b.vel=[0,0,0]; b.omega=[0,0,0]; b._stuckWith=null;
  }
  ST.groups.push(g);
  refreshGroupMassInertia(g);
  ST.anyFree = ST.bodies.some(x=>x.groupId==null&&x.motion.mode==='free') || ST.groups.some(x=>x.motion.mode==='free');
  buildBodyUI();
}
export function ungroupBodies(g){
  for(const bi of g.members){
    const b=ST.bodies[bi];
    b.groupId=null;
    b.pos0=[...b.pos]; b.quat0.copy(b.quat); b.tRef=ST.simTime;
    b.vel=[0,0,0]; b.omega=[0,0,0]; b._stuckWith=null;
    b.motion={mode:'static', axis:[0,1,0], rpm:120, amp:ST.sceneDiag*0.1, hz:1, v:[0,0,0]};
  }
  ST.groups=ST.groups.filter(x=>x!==g);
  ST.anyFree = ST.bodies.some(x=>x.groupId==null&&x.motion.mode==='free') || ST.groups.some(x=>x.motion.mode==='free');
  buildBodyUI();
}
// material controls: reused both for a standalone body's own card and for
// each member's nested card inside a group (magnetization stays per-body
// even when grouped — only motion is shared)
function materialBlockHTML(b){
  const mat=MATERIALS[b.material];
  let matCtl='';
  if(mat.cat==='perm'){
    matCtl='<div class="vec"><em>M dir</em>'+
      [0,1,2].map(k=>'<input type="number" step="0.1" value="'+b.Mloc[k].toFixed(1)+'" data-k="m'+k+'" aria-label="M'+k+'">').join('')+
      '</div><div class="vec"><em>Br (T)</em>'+
      '<input type="number" step="0.05" value="'+b.Br+'" data-k="br">'+
      '<button class="small" data-k="flip">flip N↔S</button></div>';
  }else if(mat.cat==='coil'){
    // welding/placing a soft-iron body near this one amplifies it for free
    // (js/physics.js softSolve already reads driving field from any
    // charge-emitting body) — no separate "core" property needed here
    matCtl='<div class="vec"><em>axis</em>'+
      [0,1,2].map(k=>'<input type="number" step="0.1" value="'+b.Mloc[k].toFixed(1)+'" data-k="m'+k+'" aria-label="axis'+k+'">').join('')+
      '</div><div class="vec"><em>N · I(A)</em>'+
      '<input type="number" step="10" value="'+b.turns+'" data-k="turns" aria-label="turns">'+
      '<input type="number" step="0.5" value="'+b.current+'" data-k="cur" aria-label="current (A)">'+
      '</div><div class="vec"><em>length</em>'+
      '<input type="number" step="0.5" value="'+b.coilLen.toFixed(2)+'" data-k="len" aria-label="coil length">'+
      '<button class="small" data-k="flip">flip N↔S</button></div>'+
      '<div class="mat-note" data-k="coilnote">≈ '+b.Br.toFixed(3)+' T equivalent (μ₀NI/L)</div>';
  }else if(mat.cat==='soft'){
    matCtl='<div class="mat-note">induced M · μr '+mat.muR.toLocaleString()+' · sat '+mat.Ms+' T'+
      (b.isEddy?' · eddy on':'')+'</div>';
  }else if(mat.cat==='para'||mat.cat==='dia'){
    matCtl='<div class="mat-note">χ '+mat.chi.toExponential(1)+
      (b.isEddy?' · conductive → eddy currents active':' · negligible')+'</div>';
  }else matCtl='<div class="mat-note">inert</div>';
  return '<div class="vec" style="margin-bottom:6px"><select data-k="mat">'+matOptionsHTML(b.material)+'</select></div>'+matCtl;
}
function bindMaterialBlock(container,b){
  container.querySelector('[data-k=mat]').addEventListener('change',e=>{
    b.material=e.target.value;
    const nm=MATERIALS[b.material];
    if(nm.cat==='perm') b.Br=nm.Br;
    else if(nm.cat==='coil'){ b.turns=200; b.current=2; b.coilLen=Math.max(1e-3,b.radius*1.4); }
    b.cellM.fill(0); b.edD.fill(0); b.edInit=false;
    refreshDerived(); allocSources(); buildSources(); softSolve(24);
    ST.bodies.forEach(updateArrow); buildBodyUI(); requestTrace(true);
  });
  container.querySelectorAll('input[type=number]').forEach(inp=>{
    inp.addEventListener('change',()=>{
      const k=inp.dataset.k, v=+inp.value||0;
      if(k==='br') b.Br=v;
      else if(k==='m0'||k==='m1'||k==='m2') b.Mloc[+k[1]]=v;
      else if(k==='turns') b.turns=v;
      else if(k==='cur') b.current=v;
      else if(k==='len') b.coilLen=Math.max(1e-6,v);
      updatePermQ(b); buildSources(); softSolve(6);
      ST.bodies.forEach(updateArrow); syncMeshes(); requestTrace(true);
      const note=container.querySelector('[data-k=coilnote]');
      if(note) note.textContent='≈ '+b.Br.toFixed(3)+' T equivalent (μ₀NI/L)';
    });
  });
  const flip=container.querySelector('[data-k=flip]');
  if(flip) flip.addEventListener('click',()=>{
    b.Mloc=b.Mloc.map(x=>-x);
    updatePermQ(b); buildSources(); softSolve(6);
    ST.bodies.forEach(updateArrow); buildBodyUI(); requestTrace(true);
  });
}
// motion controls: reused for a standalone body's own motion, or a group's
// shared motion — the caller supplies which `motion` object to bind to and
// what to do when the mode itself changes (each needs different reference
// bookkeeping: a lone body resets its own pos0/quat0, a group resets its own)
function motionBlockHTML(m){
  let motCtl='';
  if(m.mode==='spin')
    motCtl='<div class="vec"><em>axis</em>'+
      [0,1,2].map(k=>'<input type="number" step="0.5" value="'+m.axis[k]+'" data-k="ax'+k+'">').join('')+
      '</div><div class="vec"><em>rpm</em><input type="number" step="10" value="'+m.rpm+'" data-k="rpm"></div>';
  else if(m.mode==='osc')
    motCtl='<div class="vec"><em>axis</em>'+
      [0,1,2].map(k=>'<input type="number" step="0.5" value="'+m.axis[k]+'" data-k="ax'+k+'">').join('')+
      '</div><div class="vec"><em>amp / Hz</em>'+
      '<input type="number" step="1" value="'+m.amp.toFixed(1)+'" data-k="amp">'+
      '<input type="number" step="0.1" value="'+m.hz+'" data-k="hz"></div>';
  else if(m.mode==='slide')
    motCtl='<div class="vec"><em>vel u/s</em>'+
      [0,1,2].map(k=>'<input type="number" step="1" value="'+m.v[k]+'" data-k="v'+k+'">').join('')+'</div>';
  return '<div class="vec" style="margin-top:6px"><em>motion</em><select class="inline" data-k="mot">'+motionOptionsHTML(m.mode)+'</select></div>'+motCtl;
}
function bindMotionBlock(container,motion,onModeChange){
  container.querySelector('[data-k=mot]').addEventListener('change',e=>{
    motion.mode=e.target.value;
    onModeChange();
  });
  container.querySelectorAll('input[type=number]').forEach(inp=>{
    inp.addEventListener('change',()=>{
      const k=inp.dataset.k, v=+inp.value||0;
      if(k.startsWith('ax')) motion.axis[+k[2]]=v;
      else if(k==='rpm') motion.rpm=v;
      else if(k==='amp') motion.amp=v;
      else if(k==='hz') motion.hz=v;
      else if(k[0]==='v') motion.v[+k[1]]=v;
      buildSources(); ST.bodies.forEach(updateArrow); syncMeshes(); requestTrace(true);
    });
  });
}
// glow a body's mesh in the 3D view while it's checked for grouping, so
// it's obvious which objects are about to be welded together
function setBodyHighlight(b,on){
  b.mesh.material.emissive.setHex(on?0xe8a33d:0x000000);
}
function buildStandaloneCard(b,bi){
  const card=document.createElement('div'); card.className='body-card';
  const hex='#'+b.color.toString(16).padStart(6,'0');
  const massTxt=(b.mass<0.001?(b.mass*1000).toFixed(2)+' g':b.mass.toFixed(3)+' kg');
  const head=document.createElement('div'); head.className='head';
  head.innerHTML='<input type="checkbox" class="body-select" data-bi="'+bi+'" title="select for grouping">'+
    '<span class="chip" style="background:'+hex+'"></span>'+
    '<span class="name">'+b.name+'</span><span class="tris">'+(b.tris.length/9)+' tris · '+massTxt+'</span>';
  const matDiv=document.createElement('div'); matDiv.innerHTML=materialBlockHTML(b);
  const motDiv=document.createElement('div'); motDiv.innerHTML=motionBlockHTML(b.motion);
  card.appendChild(head); card.appendChild(matDiv); card.appendChild(motDiv);
  bindMaterialBlock(matDiv,b);
  bindMotionBlock(motDiv,b.motion,()=>{
    b.pos0=[...b.pos]; b.quat0.copy(b.quat); b.tRef=ST.simTime;
    b.vel=[0,0,0]; b.omega=[0,0,0];
    ST.anyFree=ST.bodies.some(x=>x.groupId==null&&x.motion.mode==='free')||ST.groups.some(x=>x.motion.mode==='free');
    buildBodyUI();
  });
  head.querySelector('.body-select').addEventListener('change',e=>{
    setBodyHighlight(b,e.target.checked);
    card.classList.toggle('selected',e.target.checked);
  });
  return card;
}
function buildGroupCard(g){
  const card=document.createElement('div'); card.className='group-card';
  const head=document.createElement('div'); head.className='head';
  const chips=g.members.map(bi=>'<span class="chip" style="background:#'+ST.bodies[bi].color.toString(16).padStart(6,'0')+'"></span>').join('');
  const massTxt=(g.mass<0.001?(g.mass*1000).toFixed(2)+' g':g.mass.toFixed(3)+' kg');
  head.innerHTML='<span class="chip-stack">'+chips+'</span>'+
    '<span class="name">'+g.name+'</span>'+
    '<span class="meta">'+g.members.length+' bodies · '+massTxt+'</span>'+
    '<button class="small" data-k="ungroup">Ungroup</button>';
  const motDiv=document.createElement('div'); motDiv.innerHTML=motionBlockHTML(g.motion);
  card.appendChild(head); card.appendChild(motDiv);
  bindMotionBlock(motDiv,g.motion,()=>{
    g.pos0=[...g.pos]; g.quat0.copy(g.quat); g.tRef=ST.simTime;
    g.vel=[0,0,0]; g.omega=[0,0,0];
    ST.anyFree=ST.bodies.some(x=>x.groupId==null&&x.motion.mode==='free')||ST.groups.some(x=>x.motion.mode==='free');
    buildBodyUI();
  });
  head.querySelector('[data-k=ungroup]').addEventListener('click',()=>ungroupBodies(g));
  for(const bi of g.members){
    const b=ST.bodies[bi];
    const mCard=document.createElement('div'); mCard.className='member-card';
    const mHead=document.createElement('div'); mHead.className='head';
    mHead.innerHTML='<span class="chip" style="background:#'+b.color.toString(16).padStart(6,'0')+'"></span>'+
      '<span class="name">'+b.name+'</span><span class="tris">'+(b.tris.length/9)+' tris</span>';
    const matDiv=document.createElement('div'); matDiv.innerHTML=materialBlockHTML(b);
    mCard.appendChild(mHead); mCard.appendChild(matDiv);
    bindMaterialBlock(matDiv,b);
    card.appendChild(mCard);
  }
  return card;
}
export function buildBodyUI(){
  const host=document.getElementById('bodies');
  host.innerHTML='';
  document.getElementById('bodyCount').textContent='· '+ST.bodies.length;
  // the rebuild below always recreates checkboxes unchecked, so drop any
  // highlight left over from the selection that was just acted on (or lost)
  ST.bodies.forEach(b=>setBodyHighlight(b,false));

  let groupBtn=null;
  if(ST.bodies.length>1){
    const toolbar=document.createElement('div'); toolbar.className='group-toolbar';
    groupBtn=document.createElement('button'); groupBtn.className='small'; groupBtn.textContent='Group selected'; groupBtn.disabled=true;
    groupBtn.title='Check 2+ bodies below, then group them to move/rotate as one';
    groupBtn.addEventListener('click',()=>{
      const sel=[...host.querySelectorAll('.body-select:checked')].map(el=>+el.dataset.bi);
      if(sel.length>=2) createGroup(sel);
    });
    toolbar.appendChild(groupBtn);
    host.appendChild(toolbar);
  }

  const inGroup=new Set();
  ST.groups.forEach(g=>{ host.appendChild(buildGroupCard(g)); g.members.forEach(bi=>inGroup.add(bi)); });
  ST.bodies.forEach((b,bi)=>{ if(!inGroup.has(bi)) host.appendChild(buildStandaloneCard(b,bi)); });

  if(groupBtn){
    const selects=host.querySelectorAll('.body-select');
    const updateGroupBtn=()=>{ groupBtn.disabled=host.querySelectorAll('.body-select:checked').length<2; };
    selects.forEach(cb=>cb.addEventListener('change',updateGroupBtn));
  }
}

// ---------- file input ----------
const drop=document.getElementById('drop'), fileInput=document.getElementById('file');
drop.addEventListener('click',()=>fileInput.click());
drop.addEventListener('dragover',e=>{e.preventDefault();drop.classList.add('over');});
drop.addEventListener('dragleave',()=>drop.classList.remove('over'));
drop.addEventListener('drop',e=>{
  e.preventDefault(); drop.classList.remove('over');
  const f=e.dataTransfer.files[0]; if(f) readFile(f);
});
fileInput.addEventListener('change',()=>{if(fileInput.files[0])readFile(fileInput.files[0]);});
function readFile(f){
  setStatus('parsing '+f.name+' …',true);
  const r=new FileReader();
  r.onload=()=>{try{
      const pos=parseSTL(r.result);
      if(!pos.length) throw new Error('no triangles found');
      loadGeometry(pos,f.name);
    }catch(err){setStatus('parse failed: '+err.message);}};
  r.readAsArrayBuffer(f);
}

// ---------- demos ----------
function makeBox(cx,cy,cz,sx,sy,sz,out){
  const h=[sx/2,sy/2,sz/2];
  const V=(x,y,z)=>[cx+x*h[0],cy+y*h[1],cz+z*h[2]];
  const quad=(a,b,c,d)=>{out.push(...a,...b,...c,...a,...c,...d);};
  quad(V(1,-1,-1),V(1,1,-1),V(1,1,1),V(1,-1,1));
  quad(V(-1,-1,-1),V(-1,-1,1),V(-1,1,1),V(-1,1,-1));
  quad(V(-1,1,-1),V(-1,1,1),V(1,1,1),V(1,1,-1));
  quad(V(-1,-1,-1),V(1,-1,-1),V(1,-1,1),V(-1,-1,1));
  quad(V(-1,-1,1),V(1,-1,1),V(1,1,1),V(-1,1,1));
  quad(V(-1,-1,-1),V(-1,1,-1),V(1,1,-1),V(1,-1,-1));
}
document.getElementById('demo1').addEventListener('click',()=>{
  const arr=[]; makeBox(-22,0,0,20,10,10,arr); makeBox(22,0,0,20,10,10,arr);
  loadGeometry(Float32Array.from(arr),'demo: snap',()=>{
    ST.bodies[0].Mloc=[1,0,0]; ST.bodies[0].motion.mode='static';
    ST.bodies[1].Mloc=[1,0,0]; ST.bodies[1].motion.mode='free';
    document.getElementById('gravity').checked=false; SIM.gravity=false;
    document.getElementById('speed').value='1'; SIM.speed=1;
    ST.anyFree=true;
    setTimeout(()=>setStatus('press Play — the free magnet accelerates and snaps onto the static one'),100);
  });
});
document.getElementById('demo2').addEventListener('click',()=>{
  const arr=[]; makeBox(-18,0,0,16,8,8,arr); makeBox(16,0,0,4,26,26,arr);
  loadGeometry(Float32Array.from(arr),'demo: rotor + iron',()=>{
    ST.bodies[0].Mloc=[1,0,0];
    ST.bodies[0].motion={mode:'spin',axis:[0,1,0],rpm:90,amp:5,hz:1,v:[0,0,0]};
    ST.bodies[1].material='iron'; ST.bodies[1].motion.mode='static';
    document.getElementById('gravity').checked=false; SIM.gravity=false;
    document.getElementById('speed').value='1'; SIM.speed=1;
    setTimeout(()=>setStatus('press Play — magnet spins at 90 rpm, iron plate re-magnetizes each frame'),100);
  });
});
document.getElementById('demo3').addEventListener('click',()=>{
  const arr=[]; makeBox(0,26,0,12,8,12,arr); makeBox(0,0,0,44,6,44,arr);
  loadGeometry(Float32Array.from(arr),'demo: drop on copper',()=>{
    ST.bodies[0].Mloc=[0,1,0]; ST.bodies[0].motion.mode='free';
    ST.bodies[1].material='copper'; ST.bodies[1].motion.mode='static';
    document.getElementById('gravity').checked=true; SIM.gravity=true;
    document.getElementById('speed').value='0.25'; SIM.speed=0.25;
    ST.anyFree=true;
    setTimeout(()=>setStatus('press Play — magnet falls onto copper; eddy dipoles oppose the approach (0.25×)'),100);
  });
});
document.getElementById('demo4').addEventListener('click',()=>{
  const arr=[];
  makeBox(-22.5,0,0,7,8,8,arr);     // rotor segment A
  makeBox(-14.5,0,0,7,8,8,arr);     // rotor segment B — bolted to A to form one longer 2-pole rotor (kept apart by a hairline gap so they load as two separate bodies to weld, not one fused mesh)
  makeBox(16,0,0,4,26,26,arr);      // stator coil — real brushless motors put the driven windings on the (stationary) stator and permanent magnets on the rotor, not the other way round
  loadGeometry(Float32Array.from(arr),'demo: brushless motor',()=>{
    ST.bodies[0].Mloc=[1,0,0];
    ST.bodies[1].Mloc=[1,0,0];
    createGroup([0,1]);
    ST.groups[0].motion={mode:'spin',axis:[0,1,0],rpm:90,amp:5,hz:1,v:[0,0,0]};
    ST.bodies[2].material='electromagnet';
    ST.bodies[2].Mloc=[1,0,0]; ST.bodies[2].turns=300; ST.bodies[2].current=5; ST.bodies[2].coilLen=4;
    ST.bodies[2].motion.mode='static';
    document.getElementById('gravity').checked=false; SIM.gravity=false;
    document.getElementById('speed').value='1'; SIM.speed=1;
    setTimeout(()=>setStatus('press Play — the grouped 2-piece rotor spins at 90 rpm past a fixed, energized stator coil — a simplified brushless motor'),100);
  });
});
