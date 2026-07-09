/* =====================================================================
 FIELDLINE 2 — magneto-quasistatic sandbox.
 Models (all reduced-order, qualitative):
  - Permanent magnets: equivalent surface charges q = mu0M.(nA), Br from material.
  - Soft material: moment method — interior cells each carry mu0M solved
    self-consistently (Gauss-Seidel, relaxed), Frohlich-Kennelly saturation
    chi(H) = chi0/(1+chi0|mu0H|/Ms), per-cell demag via chi_eff = chi/(1+chi/3).
  - Eddy currents: each conductive body carries a coarse grid of "eddy
    dipoles" d obeying tau d' = -d - tau*2*pi*a^3 dB/dt (single-pole model of
    a conducting sphere; exact exponential integrator, so any dt is stable).
    Captures Lenz opposition, braking, and frequency dependence; not skin depth.
  - Rigid bodies: F on charges qB/mu0, on dipoles grad(d.B)/mu0, torques,
    gravity, impulse contacts with stick damping.
 Unit system: geometry stays in STL units; B in tesla is unit-invariant for
 both charge and dipole sums, so only mass, force, tau and gravity use the
 unit scale u (metres per STL unit).
===================================================================== */
import {ST, MU0, INV4PI, MATERIALS, CONDUCTIVE, SIM, PALETTE, setStatus, timebar} from './state.js';
import {splitBodies, areaCentroid, clusterTris, voxelFill, downsampleCells} from './geometry.js';
import {scene, ctrl, applyCamera} from './scene.js';
import {syncMeshes, updateArrow, pause, resetHistory} from './rendering.js';
import {buildBodyUI} from './ui.js';
import {sendBodyShapesToWorkers, requestTrace} from './tracer.js';

// ---------- body construction ----------
export async function loadGeometry(pos, sourceName, setup){
  clearAll();
  const {parts,diag}=splitBodies(pos);
  ST.sceneDiag=diag;
  const clusterCell=diag/34, forceCell=diag/14;
  setStatus('voxelizing bodies…',true);
  for(let i=0;i<parts.length;i++){
    let tris=parts[i];
    const center=areaCentroid(tris);
    tris=Float32Array.from(tris);
    for(let k=0;k<tris.length;k+=3){tris[k]-=center[0];tris[k+1]-=center[1];tris[k+2]-=center[2];}
    const cl=clusterTris(tris,clusterCell);
    const fc=clusterTris(tris,forceCell);
    const fill=voxelFill(tris, 300);
    await new Promise(r=>setTimeout(r,0));
    // bounding radius + bbox extents for defaults
    let rad=0, mn=[1e30,1e30,1e30], mx=[-1e30,-1e30,-1e30];
    for(let k=0;k<tris.length;k+=3){
      rad=Math.max(rad,Math.hypot(tris[k],tris[k+1],tris[k+2]));
      for(let a=0;a<3;a++){mn[a]=Math.min(mn[a],tris[k+a]);mx[a]=Math.max(mx[a],tris[k+a]);}
    }
    const ext=[mx[0]-mn[0],mx[1]-mn[1],mx[2]-mn[2]];
    const axis=[0,0,0]; axis[ext.indexOf(Math.max(...ext))]=1;
    const eddy=downsampleCells(fill.cellP,fill.cellV,27);
    const fSamp=downsampleCells(fill.cellP,fill.cellV,24);
    const color=PALETTE[i%PALETTE.length];
    const geo=new THREE.BufferGeometry();
    geo.setAttribute('position',new THREE.Float32BufferAttribute(tris,3));
    geo.computeVertexNormals();
    const mesh=new THREE.Mesh(geo,new THREE.MeshPhongMaterial({
      color,shininess:22,transparent:true,opacity:0.92,side:THREE.DoubleSide}));
    scene.add(mesh);
    const b={
      name:'B'+(i+1), color, tris, mesh, arrow:null,
      material:'ndfeb_n42', Br:MATERIALS.ndfeb_n42.Br, Mloc:[...axis],
      clP:cl.pos, clS:cl.S, nCl:cl.n, q:new Float64Array(cl.n),
      fcP:fc.pos, fcS:fc.S, nFc:fc.n, fq:new Float64Array(fc.n),
      fill, cellP:fill.cellP, cellV:fill.cellV, nCell:fill.cellV.length,
      edP:eddy.p, edV:eddy.v, nEd:eddy.v.length,
      fsP:fSamp.p, fsV:fSamp.v,
      radius:rad, arrowLen:Math.max(Math.hypot(...ext)*0.85, diag*0.08),
      // pose/state
      pos:[center[0],center[1],center[2]], pos0:[...center],
      homePos:[...center], homeQuat:new THREE.Quaternion(), tRef:0,
      quat:new THREE.Quaternion(), quat0:new THREE.Quaternion(),
      vel:[0,0,0], omega:[0,0,0],
      motion:{mode:'static', axis:[0,1,0], rpm:120, amp:diag*0.1, hz:1, v:[0,0,0]},
      // grouping: when groupId is set, this body's pose is driven entirely
      // by that group each frame (see syncGroupMembers) and its own
      // motion/vel/omega above are unused — localPos/localQuat are its
      // fixed offset from the group's reference frame
      groupId:null, localPos:[0,0,0], localQuat:new THREE.Quaternion(),
      // solved state
      cellM:new Float64Array(fill.cellV.length*3),
      edD:new Float64Array(eddy.v.length*3),
      edBprev:new Float64Array(eddy.v.length*3), edInit:false,
      mass:1, invMass:1, I:1, invI:1
    };
    ST.bodies.push(b);
  }
  ST.sceneCenter=[0,0,0];
  ST.bodies.forEach(b=>{for(let k=0;k<3;k++)ST.sceneCenter[k]+=b.pos[k]/ST.bodies.length;});
  ST.gridHelper=new THREE.GridHelper(diag*2.4,24,0x2a3846,0x1a2530);
  let minY=1e30; ST.bodies.forEach(b=>minY=Math.min(minY,b.pos[1]-b.radius));
  ST.gridHelper.position.set(ST.sceneCenter[0],minY-diag*0.02,ST.sceneCenter[2]);
  scene.add(ST.gridHelper);
  ctrl.target.set(...ST.sceneCenter); ctrl.radius=diag*1.6; applyCamera();
  if(setup) setup();
  refreshDerived();
  allocSources();
  buildSources();
  softSolve(24);
  ST.bodies.forEach(updateArrow);
  syncMeshes();
  resetHistory();
  buildBodyUI();
  document.getElementById('bodiesBox').style.display='block';
  document.getElementById('traceBox').style.display='block';
  document.getElementById('simBox').style.display='block';
  setStatus(sourceName+': '+ST.bodies.length+' bodies, '+(pos.length/9)+' triangles');
  sendBodyShapesToWorkers();
  requestTrace(true);
}
export function clearAll(){
  pause();
  ST.simTime=0; timebar.textContent='';
  for(const b of ST.bodies){ if(b.mesh)scene.remove(b.mesh); if(b.arrow)scene.remove(b.arrow); }
  if(ST.lineGroup){scene.remove(ST.lineGroup); ST.lineGroup=null;}
  if(ST.gridHelper){scene.remove(ST.gridHelper); ST.gridHelper=null;}
  ST.bodies=[]; ST.SRC=null; ST.groups=[];
  ST.history=[]; ST.historyIndex=-1; ST.lastHistorySimTime=0;
}
// mass, inertia, permanent charges, flags — call on material/unit change
export function refreshDerived(){
  const u=SIM.u;
  ST.anySoft=false; ST.anyEddy=false;
  for(const b of ST.bodies){
    const mat=MATERIALS[b.material];
    let vol=0; for(const v of b.cellV) vol+=v;
    b.mass=Math.max(1e-9, mat.rho*vol*u*u*u);
    b.invMass=1/b.mass;
    let I=0;
    for(let i=0;i<b.nCell;i++){
      const r2=(b.cellP[i*3]**2+b.cellP[i*3+1]**2+b.cellP[i*3+2]**2)*u*u;
      I+=mat.rho*b.cellV[i]*u*u*u*(r2+ (Math.cbrt(b.cellV[i])*u)**2/6 );
    }
    b.I=Math.max(1e-12,I*0.8); b.invI=1/b.I;   // isotropic approximation
    b.isSoft = mat.cat==='soft'||mat.cat==='para'||mat.cat==='dia';
    b.isEddy = (mat.sigma||0)>CONDUCTIVE;
    if(b.isSoft) ST.anySoft=true;
    if(b.isEddy) ST.anyEddy=true;
    if(!b.isSoft) b.cellM.fill(0);
    if(!b.isEddy){ b.edD.fill(0); b.edInit=false; }
    for(let j=0;j<b.nEd;j++){
      const a=Math.cbrt(3*b.edV[j]/(4*Math.PI));
      b['edA3_'+j]=undefined;
    }
    b.edA3=new Float64Array(b.nEd); b.edTau=new Float64Array(b.nEd);
    for(let j=0;j<b.nEd;j++){
      const a=Math.cbrt(3*b.edV[j]/(4*Math.PI));       // STL units
      b.edA3[j]=2*Math.PI*a*a*a;                        // 2*pi*a^3 (unit-space)
      b.edTau[j]=MU0*(mat.sigma||0)*(a*u)*(a*u)/(Math.PI*Math.PI); // seconds
    }
    updatePermQ(b);
  }
  ST.groups.forEach(refreshGroupMassInertia);
  ST.anyFree = ST.bodies.some(b=>b.groupId==null&&b.motion.mode==='free') || ST.groups.some(g=>g.motion.mode==='free');
}
// combine member mass/inertia into the group's own rigid-body totals —
// call whenever a member's mass/I could have changed (material edit) or a
// group's membership changed
export function refreshGroupMassInertia(g){
  const u=SIM.u;
  let mass=0, I=0, maxReach=0;
  for(const bi of g.members){
    const b=ST.bodies[bi];
    mass+=b.mass;
    const r2=b.localPos[0]**2+b.localPos[1]**2+b.localPos[2]**2;
    I+=b.I+b.mass*r2*u*u;                 // parallel-axis theorem
    maxReach=Math.max(maxReach, Math.hypot(...b.localPos)+b.radius);
  }
  g.mass=Math.max(1e-9,mass); g.invMass=1/g.mass;
  g.I=Math.max(1e-12,I); g.invI=1/g.I;
  g.radius=Math.max(1e-6,maxReach);
}
export function updatePermQ(b){
  const mat=MATERIALS[b.material];
  if(mat.cat!=='perm'){ b.q.fill(0); b.fq.fill(0); return; }
  const ml=Math.hypot(...b.Mloc)||1;
  const mx=b.Mloc[0]/ml*b.Br, my=b.Mloc[1]/ml*b.Br, mz=b.Mloc[2]/ml*b.Br;
  for(let i=0;i<b.nCl;i++) b.q[i]=mx*b.clS[i*3]+my*b.clS[i*3+1]+mz*b.clS[i*3+2];
  for(let i=0;i<b.nFc;i++) b.fq[i]=mx*b.fcS[i*3]+my*b.fcS[i*3+1]+mz*b.fcS[i*3+2];
}

// ---------- flattened field sources ----------
export function allocSources(){
  let nc=0, nd=0;
  for(const b of ST.bodies){
    const mat=MATERIALS[b.material];
    if(mat.cat==='perm'){ b.c0=nc; nc+=b.nCl; } else b.c0=-1;
    b.d0=-1; b.e0=-1;
    if(b.isSoft){ b.d0=nd; nd+=b.nCell; }
    if(b.isEddy){ b.e0=nd; nd+=b.nEd; }
  }
  ST.SRC={
    nc, nd,
    cP:new Float64Array(nc*3), cQ:new Float64Array(nc), cB:new Int16Array(nc),
    dP:new Float64Array(nd*3), dD:new Float64Array(nd*3), dB:new Int16Array(nd),
    soft: Math.pow(ST.sceneDiag/120,2)
  };
  ST.bodies.forEach((b,bi)=>{
    if(b.c0>=0) ST.SRC.cB.fill(bi,b.c0,b.c0+b.nCl);
    if(b.d0>=0) ST.SRC.dB.fill(bi,b.d0,b.d0+b.nCell);
    if(b.e0>=0) ST.SRC.dB.fill(bi,b.e0,b.e0+b.nEd);
  });
}
export function rotOf(b){
  const q=b.quat, x=q.x,y=q.y,z=q.z,w=q.w;
  return [1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
          2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
          2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)];
}
export function xform(R,p,px,py,pz,out,o){ // world = R*local + p
  out[o]  =R[0]*px+R[1]*py+R[2]*pz+p[0];
  out[o+1]=R[3]*px+R[4]*py+R[5]*pz+p[1];
  out[o+2]=R[6]*px+R[7]*py+R[8]*pz+p[2];
}
export function buildSources(){
  for(const b of ST.bodies){
    const R=rotOf(b); b.R=R;
    if(b.c0>=0){
      for(let i=0;i<b.nCl;i++){
        xform(R,b.pos,b.clP[i*3],b.clP[i*3+1],b.clP[i*3+2],ST.SRC.cP,(b.c0+i)*3);
        ST.SRC.cQ[b.c0+i]=b.q[i];   // rotation-invariant
      }
    }
    if(b.d0>=0){
      for(let i=0;i<b.nCell;i++){
        xform(R,b.pos,b.cellP[i*3],b.cellP[i*3+1],b.cellP[i*3+2],ST.SRC.dP,(b.d0+i)*3);
        const gi=(b.d0+i)*3;
        ST.SRC.dD[gi]  =b.cellM[i*3]  *b.cellV[i];
        ST.SRC.dD[gi+1]=b.cellM[i*3+1]*b.cellV[i];
        ST.SRC.dD[gi+2]=b.cellM[i*3+2]*b.cellV[i];
      }
    }
    if(b.e0>=0){
      for(let j=0;j<b.nEd;j++){
        xform(R,b.pos,b.edP[j*3],b.edP[j*3+1],b.edP[j*3+2],ST.SRC.dP,(b.e0+j)*3);
        const gi=(b.e0+j)*3;
        ST.SRC.dD[gi]=b.edD[j*3]; ST.SRC.dD[gi+1]=b.edD[j*3+1]; ST.SRC.dD[gi+2]=b.edD[j*3+2];
      }
    }
  }
}
// B (tesla) at unit-space point; excl body index exB, excl dipole index exD
const _B=new Float64Array(3);
export function fieldCore(S,x,y,z,exB,exD){
  let bx=0,by=0,bz=0;
  const soft=S.soft;
  for(let i=0;i<S.nc;i++){
    if(S.cB[i]===exB) continue;
    const dx=x-S.cP[i*3],dy=y-S.cP[i*3+1],dz=z-S.cP[i*3+2];
    const r2=dx*dx+dy*dy+dz*dz+soft;
    const inv=S.cQ[i]/(r2*Math.sqrt(r2));
    bx+=dx*inv; by+=dy*inv; bz+=dz*inv;
  }
  for(let i=0;i<S.nd;i++){
    if(S.dB[i]===exB||i===exD) continue;
    const mx=S.dD[i*3],my=S.dD[i*3+1],mz=S.dD[i*3+2];
    if(mx===0&&my===0&&mz===0) continue;
    const dx=x-S.dP[i*3],dy=y-S.dP[i*3+1],dz=z-S.dP[i*3+2];
    const r2=dx*dx+dy*dy+dz*dz+soft;
    const r=Math.sqrt(r2), inv3=1/(r2*r);
    const mr=(mx*dx+my*dy+mz*dz)/r2;
    bx+=(3*mr*dx-mx)*inv3; by+=(3*mr*dy-my)*inv3; bz+=(3*mr*dz-mz)*inv3;
  }
  _B[0]=bx*INV4PI; _B[1]=by*INV4PI; _B[2]=bz*INV4PI;
  return Math.sqrt(_B[0]*_B[0]+_B[1]*_B[1]+_B[2]*_B[2]);
}
export const fieldAt=(x,y,z,exB=-1,exD=-1)=>fieldCore(ST.SRC,x,y,z,exB,exD);

// ---------- soft magnetization: Gauss-Seidel moment method ----------
export function softSolve(iters){
  if(!ST.anySoft) return;
  for(let it=0;it<iters;it++){
    for(const b of ST.bodies){
      if(!b.isSoft) continue;
      const mat=MATERIALS[b.material];
      const chi0 = mat.cat==='soft' ? (mat.muR-1) : mat.chi;
      const Ms = mat.Ms || Infinity;
      for(let i=0;i<b.nCell;i++){
        const gi=b.d0+i;
        fieldAt(ST.SRC.dP[gi*3],ST.SRC.dP[gi*3+1],ST.SRC.dP[gi*3+2],-1,gi);
        const bx=_B[0],by=_B[1],bz=_B[2];
        // |mu0 H| estimate with previous M
        const hx=bx-b.cellM[i*3]/3, hy=by-b.cellM[i*3+1]/3, hz=bz-b.cellM[i*3+2]/3;
        const hmag=Math.hypot(hx,hy,hz);
        const chi = isFinite(Ms)? chi0/(1+chi0*hmag/Ms) : chi0;
        const chieff = chi/(1+chi/3);
        const relax=0.65;
        let tx=chieff*bx, ty=chieff*by, tz=chieff*bz;
        let nx=b.cellM[i*3]+relax*(tx-b.cellM[i*3]);
        let ny=b.cellM[i*3+1]+relax*(ty-b.cellM[i*3+1]);
        let nz=b.cellM[i*3+2]+relax*(tz-b.cellM[i*3+2]);
        if(isFinite(Ms)){
          const mm=Math.hypot(nx,ny,nz);
          if(mm>Ms){const f=Ms/mm; nx*=f;ny*=f;nz*=f;}
        }
        b.cellM[i*3]=nx; b.cellM[i*3+1]=ny; b.cellM[i*3+2]=nz;
        ST.SRC.dD[gi*3]=nx*b.cellV[i]; ST.SRC.dD[gi*3+1]=ny*b.cellV[i]; ST.SRC.dD[gi*3+2]=nz*b.cellV[i];
      }
    }
  }
}
// ---------- eddy dipoles: exact exponential step ----------
export function eddyStep(dt){
  if(!ST.anyEddy) return;
  for(const b of ST.bodies){
    if(!b.isEddy) continue;
    for(let j=0;j<b.nEd;j++){
      const gi=b.e0+j;
      fieldAt(ST.SRC.dP[gi*3],ST.SRC.dP[gi*3+1],ST.SRC.dP[gi*3+2],-1,gi);
      if(!b.edInit){ b.edBprev[j*3]=_B[0]; b.edBprev[j*3+1]=_B[1]; b.edBprev[j*3+2]=_B[2]; continue; }
      const dbx=(_B[0]-b.edBprev[j*3])/dt, dby=(_B[1]-b.edBprev[j*3+1])/dt, dbz=(_B[2]-b.edBprev[j*3+2])/dt;
      b.edBprev[j*3]=_B[0]; b.edBprev[j*3+1]=_B[1]; b.edBprev[j*3+2]=_B[2];
      const tau=b.edTau[j], a3=b.edA3[j];
      const e=Math.exp(-dt/tau);
      // steady state for constant dB/dt: d* = -2 pi a^3 tau dB/dt
      const sx=-a3*tau*dbx, sy=-a3*tau*dby, sz=-a3*tau*dbz;
      b.edD[j*3]  =sx+(b.edD[j*3]  -sx)*e;
      b.edD[j*3+1]=sy+(b.edD[j*3+1]-sy)*e;
      b.edD[j*3+2]=sz+(b.edD[j*3+2]-sz)*e;
      ST.SRC.dD[gi*3]=b.edD[j*3]; ST.SRC.dD[gi*3+1]=b.edD[j*3+1]; ST.SRC.dD[gi*3+2]=b.edD[j*3+2];
    }
    b.edInit=true;
  }
}

// ---------- forces & rigid-body integration ----------
export function bodyForces(bi,b,F,T){
  const eps=ST.sceneDiag/200;
  const R=b.R||rotOf(b);
  // charge forces (permanent magnets)
  if(b.c0>=0){
    const tmp=[0,0,0];
    for(let i=0;i<b.nFc;i++){
      xform(R,b.pos,b.fcP[i*3],b.fcP[i*3+1],b.fcP[i*3+2],tmp,0);
      fieldAt(tmp[0],tmp[1],tmp[2],bi);
      const q=b.fq[i];
      F[0]+=q*_B[0]; F[1]+=q*_B[1]; F[2]+=q*_B[2];
      const rx=tmp[0]-b.pos[0],ry=tmp[1]-b.pos[1],rz=tmp[2]-b.pos[2];
      T[0]+=ry*q*_B[2]-rz*q*_B[1];
      T[1]+=rz*q*_B[0]-rx*q*_B[2];
      T[2]+=rx*q*_B[1]-ry*q*_B[0];
    }
  }
  // dipole forces: soft cells (decimated) + eddy cells, grad(d.B) by FD
  const dipSets=[];
  if(b.isSoft){
    // sample the coarse fsP grid, interpolate M as body-average (cheap)
    let ax=0,ay=0,az=0,vt=0;
    for(let i=0;i<b.nCell;i++){
      ax+=b.cellM[i*3]*b.cellV[i]; ay+=b.cellM[i*3+1]*b.cellV[i]; az+=b.cellM[i*3+2]*b.cellV[i]; vt+=b.cellV[i];
    }
    dipSets.push({p:b.fsP, d:(j,v)=>[ax/vt*v, ay/vt*v, az/vt*v], v:b.fsV, n:b.fsV.length});
  }
  if(b.isEddy) dipSets.push({p:b.edP, d:(j)=>[b.edD[j*3],b.edD[j*3+1],b.edD[j*3+2]], v:null, n:b.nEd});
  const tmp=[0,0,0];
  for(const set of dipSets){
    for(let j=0;j<set.n;j++){
      xform(R,b.pos,set.p[j*3],set.p[j*3+1],set.p[j*3+2],tmp,0);
      const dvec=set.d(j, set.v?set.v[j]:1);
      const dm=Math.hypot(...dvec); if(dm<1e-20) continue;
      // B at point (for torque d x B) and FD gradient of (d.B)
      fieldAt(tmp[0],tmp[1],tmp[2],bi);
      const B0=[_B[0],_B[1],_B[2]];
      const g=[0,0,0];
      for(let ax2=0;ax2<3;ax2++){
        const o=[...tmp]; o[ax2]+=eps;
        fieldAt(o[0],o[1],o[2],bi);
        const up=dvec[0]*_B[0]+dvec[1]*_B[1]+dvec[2]*_B[2];
        o[ax2]-=2*eps;
        fieldAt(o[0],o[1],o[2],bi);
        const dn=dvec[0]*_B[0]+dvec[1]*_B[1]+dvec[2]*_B[2];
        g[ax2]=(up-dn)/(2*eps);
      }
      F[0]+=g[0]; F[1]+=g[1]; F[2]+=g[2];
      const rx=tmp[0]-b.pos[0],ry=tmp[1]-b.pos[1],rz=tmp[2]-b.pos[2];
      T[0]+=ry*g[2]-rz*g[1] + (dvec[1]*B0[2]-dvec[2]*B0[1]);
      T[1]+=rz*g[0]-rx*g[2] + (dvec[2]*B0[0]-dvec[0]*B0[2]);
      T[2]+=rx*g[1]-ry*g[0] + (dvec[0]*B0[1]-dvec[1]*B0[0]);
    }
  }
}
export function inBodyLocal(b,wx,wy,wz){
  // world -> local (R^T (w - pos)) then fill-grid lookup
  const R=b.R||rotOf(b);
  const dx=wx-b.pos[0],dy=wy-b.pos[1],dz=wz-b.pos[2];
  const lx=R[0]*dx+R[3]*dy+R[6]*dz;
  const ly=R[1]*dx+R[4]*dy+R[7]*dz;
  const lz=R[2]*dx+R[5]*dy+R[8]*dz;
  const f=b.fill;
  const i=Math.floor((lx-f.min[0])/f.cellX), j=Math.floor((ly-f.min[1])/f.cellY), k=Math.floor((lz-f.min[2])/f.cellZ);
  if(i<0||j<0||k<0||i>=f.nx||j>=f.ny||k>=f.nz) return false;
  return f.occ[(k*f.ny+j)*f.nx+i]===1;
}
export function contacts(){
  const tmp=[0,0,0];
  for(let a=0;a<ST.bodies.length;a++) for(let c=a+1;c<ST.bodies.length;c++){
    const A=ST.bodies[a],B=ST.bodies[c];
    if(A.groupId!=null&&A.groupId===B.groupId) continue; // rigidly welded — never self-collide
    // geometry (surface samples, radius) stays per-body below, but the
    // actual dynamical response — mass, velocity, position — belongs to
    // whatever is really moving: the body itself, or its group if it's
    // welded into one (dyn() resolves that)
    const dynA=dyn(A), dynB=dyn(B);
    const freeA=dynA.motion.mode==='free', freeB=dynB.motion.mode==='free';
    if(!freeA&&!freeB) continue;
    const dx=B.pos[0]-A.pos[0],dy=B.pos[1]-A.pos[1],dz=B.pos[2]-A.pos[2];
    const dist=Math.hypot(dx,dy,dz), touchR=A.radius+B.radius;
    const stuck=A._stuckWith===B;
    // sticky band: once a resting contact has been established, keep it
    // held through a slightly wider radius than the exact touching distance
    // so a one-frame gap from the separation "push" below doesn't let the
    // strong close-range magnetic force/torque re-launch it before the next
    // narrow-phase check confirms contact again (that gap is what caused
    // the endless jitter/tumble instead of a clean, held snap)
    if(dist>touchR*(stuck?1.15:1)){
      if(stuck){ A._stuckWith=null; B._stuckWith=null; }
      continue;
    }
    // narrow phase: A's coarse surface points inside B (and vice versa)
    let n=[0,0,0], hits=0;
    const RA=A.R||rotOf(A);
    for(let i=0;i<A.nFc;i++){
      xform(RA,A.pos,A.fcP[i*3],A.fcP[i*3+1],A.fcP[i*3+2],tmp,0);
      if(inBodyLocal(B,tmp[0],tmp[1],tmp[2])){
        // outward normal of A's cluster in world
        const sx=A.fcS[i*3],sy=A.fcS[i*3+1],sz=A.fcS[i*3+2];
        n[0]+=RA[0]*sx+RA[1]*sy+RA[2]*sz;
        n[1]+=RA[3]*sx+RA[4]*sy+RA[5]*sz;
        n[2]+=RA[6]*sx+RA[7]*sy+RA[8]*sz;
        hits++;
      }
    }
    if(!hits){
      const RB=B.R||rotOf(B);
      for(let i=0;i<B.nFc;i++){
        xform(RB,B.pos,B.fcP[i*3],B.fcP[i*3+1],B.fcP[i*3+2],tmp,0);
        if(inBodyLocal(A,tmp[0],tmp[1],tmp[2])){
          const sx=B.fcS[i*3],sy=B.fcS[i*3+1],sz=B.fcS[i*3+2];
          n[0]-=RB[0]*sx+RB[1]*sy+RB[2]*sz;
          n[1]-=RB[3]*sx+RB[4]*sy+RB[5]*sz;
          n[2]-=RB[6]*sx+RB[7]*sy+RB[8]*sz;
          hits++;
        }
      }
    }
    if(!hits){
      if(stuck){
        // still within the sticky band even though this frame's exact
        // voxel-overlap sample missed — keep holding the resting contact
        if(freeA){ dynA.vel[0]=dynA.vel[1]=dynA.vel[2]=0; dynA.omega[0]=dynA.omega[1]=dynA.omega[2]=0; }
        if(freeB){ dynB.vel[0]=dynB.vel[1]=dynB.vel[2]=0; dynB.omega[0]=dynB.omega[1]=dynB.omega[2]=0; }
      }
      continue;
    }
    let nl=Math.hypot(...n);
    if(nl<1e-9){ n=[dx,dy,dz]; nl=Math.hypot(...n)||1; }
    n=[n[0]/nl,n[1]/nl,n[2]/nl];   // points roughly from A into B
    // for flush/face contact the handful of sample hits can yield a noisy,
    // off-axis normal; when it disagrees with the center-to-center line
    // (the only stable reference for two attracting bodies), fall back to
    // that line instead of letting a skewed normal inject spurious torque
    const cl=Math.hypot(dx,dy,dz);
    if(cl>1e-9){
      const cx=dx/cl,cy=dy/cl,cz=dz/cl;
      if(n[0]*cx+n[1]*cy+n[2]*cz<0.3) n=[cx,cy,cz];
    }
    // kill approaching relative velocity along n (inelastic impulse)
    const rel=(dynA.vel[0]-dynB.vel[0])*n[0]+(dynA.vel[1]-dynB.vel[1])*n[1]+(dynA.vel[2]-dynB.vel[2])*n[2];
    if(rel>0){
      const iA=freeA?dynA.invMass:0, iB=freeB?dynB.invMass:0;
      const jimp=rel/(iA+iB);
      if(freeA){dynA.vel[0]-=jimp*iA*n[0];dynA.vel[1]-=jimp*iA*n[1];dynA.vel[2]-=jimp*iA*n[2];}
      if(freeB){dynB.vel[0]+=jimp*iB*n[0];dynB.vel[1]+=jimp*iB*n[1];dynB.vel[2]+=jimp*iB*n[2];}
    }
    // small separation + stick damping (magnets snap and hold): once
    // touching, fully arrest motion each frame rather than only bleeding
    // off 30% — the magnetic attraction re-injects velocity every step, so
    // partial damping never converges and the bodies jitter/tumble forever
    const push=ST.sceneDiag*0.0006;
    if(freeA){dynA.pos[0]-=n[0]*push;dynA.pos[1]-=n[1]*push;dynA.pos[2]-=n[2]*push;
      dynA.vel[0]=dynA.vel[1]=dynA.vel[2]=0; dynA.omega[0]=dynA.omega[1]=dynA.omega[2]=0;}
    if(freeB){dynB.pos[0]+=n[0]*push;dynB.pos[1]+=n[1]*push;dynB.pos[2]+=n[2]*push;
      dynB.vel[0]=dynB.vel[1]=dynB.vel[2]=0; dynB.omega[0]=dynB.omega[1]=dynB.omega[2]=0;}
    A._stuckWith=B; B._stuckWith=A;
  }
}
const _dq=new THREE.Quaternion(), _ax=new THREE.Vector3();
// groups: a group is itself pose/motion state shaped just like a body
// (pos, quat, vel, omega, motion, mass, I, radius) — kinematicUpdate/
// integrateFree/integratePose below work on either a body or a group
// interchangeably. A grouped body has no independent motion of its own:
// each step its world pose is re-derived from its group's pose plus its
// fixed local offset (syncGroupMembers), which is what makes every member
// move and rotate together as a single rigid object.
export function groupOf(b){ return b.groupId==null?null:ST.groups.find(g=>g.id===b.groupId)||null; }
export function dyn(b){ return groupOf(b)||b; }
export function kinematicUpdate(e,t){
  const m=e.motion, tt=t-(e.tRef||0);
  if(m.mode==='spin'){
    const w=m.rpm*2*Math.PI/60;
    _ax.set(...m.axis).normalize();
    _dq.setFromAxisAngle(_ax, w*tt);
    e.quat.copy(_dq).multiply(e.quat0);
    e.omega=[_ax.x*w,_ax.y*w,_ax.z*w];
  }else if(m.mode==='osc'){
    const al=Math.hypot(...m.axis)||1, s=Math.sin(2*Math.PI*m.hz*tt);
    const c=Math.cos(2*Math.PI*m.hz*tt)*2*Math.PI*m.hz;
    for(let k=0;k<3;k++){
      e.pos[k]=e.pos0[k]+m.axis[k]/al*m.amp*s;
      e.vel[k]=m.axis[k]/al*m.amp*c;
    }
  }else if(m.mode==='slide'){
    for(let k=0;k<3;k++){ e.pos[k]=e.pos0[k]+m.v[k]*tt; e.vel[k]=m.v[k]; }
  }
}
// push a group's pose out to its members' own b.pos/b.quat (world = group
// pose + each member's fixed local offset) — everything else in the app
// (buildSources, contacts' narrow phase, bodyForces, rendering) reads
// per-body pos/quat directly and stays unaware groups exist at all
export function syncGroupMembers(g){
  const R=rotOf(g); g.R=R;
  for(const bi of g.members){
    const b=ST.bodies[bi];
    xform(R,g.pos,b.localPos[0],b.localPos[1],b.localPos[2],b.pos,0);
    b.quat.copy(g.quat).multiply(b.localQuat);
    b.vel=[...g.vel]; b.omega=[...g.omega];
  }
}
export function nearAnyOther(pos,radius,excludeBi){
  for(let i=0;i<ST.bodies.length;i++){
    if(excludeBi.has(i)) continue;
    const o=ST.bodies[i];
    const ddx=o.pos[0]-pos[0],ddy=o.pos[1]-pos[1],ddz=o.pos[2]-pos[2];
    if(Math.hypot(ddx,ddy,ddz)<(radius+o.radius)*1.4) return true;
  }
  return false;
}
export function integrateFree(e,F,T,h,fScale,tScale,u,nearOther){
  let Fx=F[0]*fScale, Fy=F[1]*fScale, Fz=F[2]*fScale;       // newtons
  if(SIM.gravity) Fy-=e.mass*SIM.g;
  const Tx=T[0]*tScale,Ty=T[1]*tScale,Tz=T[2]*tScale;       // N.m
  e.vel[0]+=Fx*e.invMass/u*h; e.vel[1]+=Fy*e.invMass/u*h; e.vel[2]+=Fz*e.invMass/u*h;
  e.omega[0]+=Tx*e.invI*h; e.omega[1]+=Ty*e.invI*h; e.omega[2]+=Tz*e.invI*h;
  // mild drag for numerical sanity; much stronger once another body is
  // close, since near-contact magnetic torque/force is huge and would
  // otherwise reload the just-zeroed contact velocity into a runaway
  // spin/jitter within a single free (non-contacting) step
  const dr=Math.exp((nearOther?-6:-0.4)*h), drOm=Math.exp((nearOther?-10:-0.4)*h);
  for(let k=0;k<3;k++){ e.vel[k]*=dr; e.omega[k]*=drOm; }
  // speed caps
  const vm=Math.hypot(...e.vel), vmax=ST.sceneDiag*3;
  if(vm>vmax) for(let k=0;k<3;k++) e.vel[k]*=vmax/vm;
  const om=Math.hypot(...e.omega), omax=nearOther?4:40;
  if(om>omax) for(let k=0;k<3;k++) e.omega[k]*=omax/om;
}
export function integratePose(e,h){
  for(let k=0;k<3;k++) e.pos[k]+=e.vel[k]*h;
  const om=Math.hypot(...e.omega);
  if(om>1e-9){
    _ax.set(e.omega[0]/om,e.omega[1]/om,e.omega[2]/om);
    _dq.setFromAxisAngle(_ax,om*h);
    e.quat.premultiply(_dq).normalize();
  }
}
export function step(h){
  ST.simTime+=h;
  const u=SIM.u, fScale=u*u/MU0, tScale=u*u*u/MU0;
  // kinematic poses: a standalone body drives itself; a grouped body is
  // driven by its group instead (propagated below), which is what makes
  // the whole group move/rotate as a single rigid object
  for(const b of ST.bodies){ if(b.groupId==null) kinematicUpdate(b,ST.simTime); }
  for(const g of ST.groups) kinematicUpdate(g,ST.simTime);
  for(const g of ST.groups) syncGroupMembers(g);

  buildSources();
  softSolve(2);          // warm-started relaxation
  eddyStep(h);

  // forces: standalone free bodies integrate directly; a free GROUP sums
  // every member's force/torque (each re-based from the member's own
  // center to the group's center via the standard lever-arm/parallel-axis
  // transfer) and integrates once as a single rigid body — a magnet pulling
  // on just one member of a welded assembly ends up moving all of it
  for(let bi=0;bi<ST.bodies.length;bi++){
    const b=ST.bodies[bi];
    if(b.groupId!=null||b.motion.mode!=='free') continue;
    const F=[0,0,0],T=[0,0,0];
    bodyForces(bi,b,F,T);
    integrateFree(b,F,T,h,fScale,tScale,u,nearAnyOther(b.pos,b.radius,new Set([bi])));
  }
  for(const g of ST.groups){
    if(g.motion.mode!=='free') continue;
    const F=[0,0,0],T=[0,0,0];
    for(const bi of g.members){
      const b=ST.bodies[bi];
      const Fm=[0,0,0],Tm=[0,0,0];
      bodyForces(bi,b,Fm,Tm);
      F[0]+=Fm[0]; F[1]+=Fm[1]; F[2]+=Fm[2];
      const rx=b.pos[0]-g.pos[0],ry=b.pos[1]-g.pos[1],rz=b.pos[2]-g.pos[2];
      T[0]+=Tm[0]+ry*Fm[2]-rz*Fm[1];
      T[1]+=Tm[1]+rz*Fm[0]-rx*Fm[2];
      T[2]+=Tm[2]+rx*Fm[1]-ry*Fm[0];
    }
    integrateFree(g,F,T,h,fScale,tScale,u,nearAnyOther(g.pos,g.radius,new Set(g.members)));
  }

  contacts();

  // integrate position: standalone free bodies, then free groups — then
  // push every group's (possibly contacts()-corrected) pose back out to
  // its members so rendering/next-frame physics see the final result
  for(const b of ST.bodies){ if(b.groupId==null&&b.motion.mode==='free') integratePose(b,h); }
  for(const g of ST.groups){ if(g.motion.mode==='free') integratePose(g,h); }
  for(const g of ST.groups) syncGroupMembers(g);
}
// current magnetic moment direction (world space) — used both for the
// visual arrow (rendering.js) and to weight field-line seed placement for
// soft/eddy bodies (tracer.js's gatherSeeds), so it lives here as the
// physics quantity it actually is rather than duplicated in either caller.
export function arrowVec(b){
  const mat=MATERIALS[b.material];
  if(mat.cat==='perm'){
    const R=b.R||rotOf(b), m=b.Mloc;
    return [R[0]*m[0]+R[1]*m[1]+R[2]*m[2],
            R[3]*m[0]+R[4]*m[1]+R[5]*m[2],
            R[6]*m[0]+R[7]*m[1]+R[8]*m[2]];
  }
  if(b.isSoft){
    let x=0,y=0,z=0;
    for(let i=0;i<b.nCell;i++){x+=b.cellM[i*3]*b.cellV[i];y+=b.cellM[i*3+1]*b.cellV[i];z+=b.cellM[i*3+2]*b.cellV[i];}
    return [x,y,z];
  }
  return [0,0,0];
}
