// Field-line tracing: seed placement, the CPU worker pool (fallback) and
// the primary GPU tracer (dedicated worker via OffscreenCanvas), and the
// trace-request/response orchestration that drives them both through the
// same message protocol so finishTrace/onTraceWorkerMessage don't care
// which backend actually produced a given response.
import {ST, MATERIALS, setStatus} from './state.js';
import {rotOf, xform, arrowVec} from './physics.js';
import {scene} from './scene.js';
import {maybeContinueLockstep, onLiveTraceLanded} from './rendering.js';

function gatherSeeds(){
  // per body: per-TRIANGLE surface charge (M.n*area) in world space, so seed
  // points can be drawn from anywhere on the real pole face — not just from
  // the coarse force/charge cluster centroids (those collapse a whole face
  // to a handful of points on low-poly meshes, making lines erupt from a
  // couple of spots instead of the object's actual surface).
  const out=[]; const A=[0,0,0],Bv=[0,0,0],C=[0,0,0];
  for(const b of ST.bodies){
    const mat=MATERIALS[b.material], R=b.R||rotOf(b);
    const list=[];
    let mx=0,my=0,mz=0;
    if(mat.cat==='perm'||mat.cat==='coil'){
      const ml=Math.hypot(...b.Mloc)||1;
      mx=b.Mloc[0]/ml*b.Br; my=b.Mloc[1]/ml*b.Br; mz=b.Mloc[2]/ml*b.Br;
    }else if(b.isSoft){
      const av=arrowVec(b); let vt=0; for(const v of b.cellV) vt+=v;
      if(vt>0){ mx=av[0]/vt; my=av[1]/vt; mz=av[2]/vt; }
      if(Math.hypot(mx,my,mz)<=1e-6){ mx=my=mz=0; }
    }
    if(mx||my||mz){
      const tris=b.tris;
      for(let i=0;i<tris.length;i+=9){
        const e1x=tris[i+3]-tris[i],e1y=tris[i+4]-tris[i+1],e1z=tris[i+5]-tris[i+2];
        const e2x=tris[i+6]-tris[i],e2y=tris[i+7]-tris[i+1],e2z=tris[i+8]-tris[i+2];
        const nx=(e1y*e2z-e1z*e2y)*.5, ny=(e1z*e2x-e1x*e2z)*.5, nz=(e1x*e2y-e1y*e2x)*.5;
        const area=Math.hypot(nx,ny,nz);
        if(area<1e-14) continue;
        const q=mx*nx+my*ny+mz*nz;             // = M.nhat * area (rotation-invariant)
        if(Math.abs(q)<1e-14) continue;
        xform(R,b.pos,tris[i],tris[i+1],tris[i+2],A,0);
        xform(R,b.pos,tris[i+3],tris[i+4],tris[i+5],Bv,0);
        xform(R,b.pos,tris[i+6],tris[i+7],tris[i+8],C,0);
        list.push({ax:A[0],ay:A[1],az:A[2],bx:Bv[0],by:Bv[1],bz:Bv[2],cx:C[0],cy:C[1],cz:C[2],
          nx:nx/area,ny:ny/area,nz:nz/area,q,idx:i});
      }
    }
    out.push(list);
  }
  // drop magnetically negligible triangles
  let gmax=0;
  for(const l of out) for(const s of l) gmax=Math.max(gmax,Math.abs(s.q));
  return out.map(l=>l.filter(s=>Math.abs(s.q)>gmax*1e-3));
}
// deterministic pseudo-random in [0,1) — used instead of Math.random() for
// seed placement so that retracing an unchanged (settled) body reproduces
// the exact same seed points instead of the field lines visibly wiggling
// on every "live lines" retrace even though nothing is actually moving
function hashRand(seed){
  const x=Math.sin(seed*12.9898)*43758.5453;
  return x-Math.floor(x);
}
function pickSeeds(list,sign,count){
  const cand=list.filter(s=>sign>0?s.q>0:s.q<0);
  if(!cand.length) return [];
  let total=0; for(const s of cand) total+=Math.abs(s.q);
  const out=[]; let acc=0,j=0;
  const targets=[]; for(let k=0;k<count;k++) targets.push((k+0.5)/count*total);
  for(const s of cand){
    acc+=Math.abs(s.q);
    let hits=0;
    while(j<targets.length&&targets[j]<=acc){hits++;j++;}
    for(let h=0;h<hits;h++){
      // uniform point on the source triangle (area-weighted barycentric sample)
      let r1=hashRand(s.idx*7.31+h*3.17+1.1), r2=hashRand(s.idx*7.31+h*3.17+5.9);
      if(r1+r2>1){ r1=1-r1; r2=1-r2; }
      const x=s.ax+r1*(s.bx-s.ax)+r2*(s.cx-s.ax);
      const y=s.ay+r1*(s.by-s.ay)+r2*(s.cy-s.ay);
      const z=s.az+r1*(s.bz-s.az)+r2*(s.cz-s.az);
      out.push({x,y,z,q:s.q,nx:s.nx,ny:s.ny,nz:s.nz});
    }
  }
  return out;
}
function snapshotSources(){
  return {nc:ST.SRC.nc,nd:ST.SRC.nd,soft:ST.SRC.soft,
    cP:ST.SRC.cP.slice(),cQ:ST.SRC.cQ.slice(),cB:ST.SRC.cB.slice(),
    dP:ST.SRC.dP.slice(),dD:ST.SRC.dD.slice(),dB:ST.SRC.dB.slice()};
}
// Termination used to test against a heuristic, dilated, shared grid built
// from surface-charge-cluster points (removed function buildTermGrid) — a
// tuned approximation and the root of most historical tracer bugs (multiple
// "poles", discontinuous arcs, early/late termination, punch-through),
// because the dilation margin was a guess, not the body's real shape.
// inBodyLocal's exact voxel-fill test (physics.js — already proven correct,
// since contacts() already uses it for collision detection) removes that
// whole class of bug: a line terminates exactly where the body's real
// surface is, no dilation/margin tuning involved.
//
// The fill grid (occ/nx/ny/nz/min/cellX/Y/Z) is static after load, so it's
// sent to each trace worker once (on load / body-count change) rather than
// rebuilt every trace cycle — only each body's current pos/rotation (which
// does change every cycle) needs to travel with each trace job.
function bodyShapesSnapshot(){
  return ST.bodies.map(b=>{
    const f=b.fill;
    return {occ:f.occ, nx:f.nx, ny:f.ny, nz:f.nz, min:f.min,
      cellX:f.cellX, cellY:f.cellY, cellZ:f.cellZ,
      // own-body-only "clear the seed's own voxel first" halo, scaled to
      // this body's actual voxel resolution rather than a global guess —
      // a seed offset only sceneDiag*0.006 off its own surface can still
      // land inside the *voxelized* approximation of its own body at step 0
      marginLocal:Math.hypot(f.cellX,f.cellY,f.cellZ)*1.5,
      radius:b.radius};
  });
}
export function sendBodyShapesToWorkers(){
  const shapes=bodyShapesSnapshot();
  traceWorkers.forEach(w=>w.postMessage({type:'shapes',shapes}));
  gpuTraceWorker.postMessage({type:'shapes',shapes});
}
function snapshotPoses(){
  return ST.bodies.map(b=>({pos:b.pos.slice(),R:(b.R||rotOf(b)).slice()}));
}
// traceLine/fieldCore's math is duplicated verbatim inside TRACE_WORKER_SRC
// below so it can run on a background thread — see the note there for why.
const TRACE_WORKER_SRC=`
const INV4PI=1/(4*Math.PI);
const _B=new Float64Array(3);
let sceneDiag=1, sceneCenter=[0,0,0];
function fieldCore(S,x,y,z,exB,exD){
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
// Static per-body occupancy grids (see bodyShapesSnapshot on the main
// thread) — sent once per load/body-count change via a {type:'shapes'}
// message, not with every trace job, since they never change pose-to-pose.
let BODY_SHAPES=[];
// Mirrors the main thread's inBodyLocal (used there by contacts() for
// collision detection) — exact "is this world point inside body k's real
// solid volume" test, against that body's CURRENT pose (poses[k], sent
// fresh with every trace job).
function inBodyLocalW(shape,pose,wx,wy,wz){
  const R=pose.R;
  const dx=wx-pose.pos[0],dy=wy-pose.pos[1],dz=wz-pose.pos[2];
  const lx=R[0]*dx+R[3]*dy+R[6]*dz;
  const ly=R[1]*dx+R[4]*dy+R[7]*dz;
  const lz=R[2]*dx+R[5]*dy+R[8]*dz;
  const i=Math.floor((lx-shape.min[0])/shape.cellX),
        j=Math.floor((ly-shape.min[1])/shape.cellY),
        k=Math.floor((lz-shape.min[2])/shape.cellZ);
  if(i<0||j<0||k<0||i>=shape.nx||j>=shape.ny||k>=shape.nz) return false;
  return shape.occ[(k*shape.ny+j)*shape.nx+i]===1;
}
// Termination needs a touch more tolerance than a bare point-in-volume test:
// with a finite integration step (h), a line can pass within a fraction of a
// voxel of the real surface — grazing it tangentially, or approaching a
// field-line saddle point — without its *sampled* points ever landing inside
// the exact occupied voxel. Checking the immediate 3x3x3 neighborhood (still
// sized to this body's own real voxel resolution, not a hand-tuned global
// margin) catches "at the surface, within numerical step size" without
// reintroducing the old dilated-grid's much larger, heuristic halo.
function nearBodyW(shape,pose,wx,wy,wz){
  const R=pose.R;
  const dx=wx-pose.pos[0],dy=wy-pose.pos[1],dz=wz-pose.pos[2];
  const lx=R[0]*dx+R[3]*dy+R[6]*dz;
  const ly=R[1]*dx+R[4]*dy+R[7]*dz;
  const lz=R[2]*dx+R[5]*dy+R[8]*dz;
  const ci=Math.floor((lx-shape.min[0])/shape.cellX),
        cj=Math.floor((ly-shape.min[1])/shape.cellY),
        ck=Math.floor((lz-shape.min[2])/shape.cellZ);
  for(let a=-1;a<=1;a++)for(let b=-1;b<=1;b++)for(let c=-1;c<=1;c++){
    const i=ci+a,j=cj+b,k=ck+c;
    if(i<0||j<0||k<0||i>=shape.nx||j>=shape.ny||k>=shape.nz) continue;
    if(shape.occ[(k*shape.ny+j)*shape.nx+i]===1) return true;
  }
  return false;
}
function traceLine(S,poses,seed,dir,q,bi){
  const h=q.h, maxSteps=q.steps, maxR=sceneDiag*2.1, off=sceneDiag*0.006;
  let px=seed.x+seed.nx*off, py=seed.y+seed.ny*off, pz=seed.z+seed.nz*off;
  const sx0=px, sy0=py, sz0=pz;
  const ownShape=BODY_SHAPES[bi], ownEscape=(ownShape?ownShape.marginLocal:0)+off;
  const pts=[px,py,pz], mags=[fieldCore(S,px,py,pz,-1,-1)];
  const k1=[0,0,0],k2=[0,0,0],k3=[0,0,0],k4=[0,0,0];
  const d=(x,y,z,out)=>{const m=fieldCore(S,x,y,z,-1,-1);const inv=dir/(m||1e-30);
    out[0]=_B[0]*inv;out[1]=_B[1]*inv;out[2]=_B[2]*inv;return m;};
  // any OTHER body's real surface is always an immediate termination; our
  // own launch body only counts once the line has cleared its own seed
  // voxel (ownEscape) — otherwise the seed's start point (a hair off its
  // own surface) would immediately look "inside" and cut the line to length 0
  const hitsAnyBody=(x,y,z)=>{
    for(let k=0;k<poses.length;k++){
      const shape=BODY_SHAPES[k], pose=poses[k];
      const rdx=x-pose.pos[0],rdy=y-pose.pos[1],rdz=z-pose.pos[2];
      if(rdx*rdx+rdy*rdy+rdz*rdz>shape.radius*shape.radius) continue; // cheap bounding-sphere pre-check
      if(!nearBodyW(shape,pose,x,y,z)) continue;
      if(k!==bi) return true;
      const dxs=x-sx0,dys=y-sy0,dzs=z-sz0;
      if(dxs*dxs+dys*dys+dzs*dzs>ownEscape*ownEscape) return true;
    }
    return false;
  };
  for(let s=0;s<maxSteps;s++){
    const m=d(px,py,pz,k1);
    if(q.rk4){
      d(px+k1[0]*h/2,py+k1[1]*h/2,pz+k1[2]*h/2,k2);
      d(px+k2[0]*h/2,py+k2[1]*h/2,pz+k2[2]*h/2,k3);
      d(px+k3[0]*h,py+k3[1]*h,pz+k3[2]*h,k4);
      px+=h/6*(k1[0]+2*k2[0]+2*k3[0]+k4[0]);
      py+=h/6*(k1[1]+2*k2[1]+2*k3[1]+k4[1]);
      pz+=h/6*(k1[2]+2*k2[2]+2*k3[2]+k4[2]);
    }else{ // RK2 midpoint
      d(px+k1[0]*h/2,py+k1[1]*h/2,pz+k1[2]*h/2,k2);
      px+=h*k2[0]; py+=h*k2[1]; pz+=h*k2[2];
    }
    pts.push(px,py,pz); mags.push(m);
    const rx=px-sceneCenter[0],ry=py-sceneCenter[1],rz=pz-sceneCenter[2];
    if(rx*rx+ry*ry+rz*rz>maxR*maxR) break;
    if(s>2&&hitsAnyBody(px,py,pz)) break;
  }
  return {pts,mags};
}
// colour (log|B| -> HSL) is computed on the main thread after all workers'
// chunks are back in, since it needs the *global* magnitude range across
// every chunk — so each worker only returns raw positions + magnitudes.
self.onmessage=function(e){
  if(e.data.type==='shapes'){ BODY_SHAPES=e.data.shapes; return; }
  const {reqId,S,poses,jobs,q}=e.data;
  try{
    sceneDiag=e.data.sceneDiag; sceneCenter=e.data.sceneCenter;
    const verts=[],mags=[];
    for(let i=0;i<jobs.length;i++){
      const r=traceLine(S,poses,jobs[i].s,jobs[i].dir,q,jobs[i].bi);
      const n=r.pts.length/3;
      for(let k=0;k<n-1;k++) for(const idx of[k,k+1]){
        verts.push(r.pts[idx*3],r.pts[idx*3+1],r.pts[idx*3+2]);
        mags.push(r.mags[idx]);
      }
    }
    const va=new Float32Array(verts), ma=new Float32Array(mags);
    self.postMessage({reqId,verts:va,mags:ma,jobCount:jobs.length},[va.buffer,ma.buffer]);
  }catch(err){
    self.postMessage({reqId,error:err.message});
  }
};
`;

// GPU field-line tracer: same physics/termination as TRACE_WORKER_SRC above
// (fieldCore, RK2/RK4 integration, inBodyLocal-style exact termination),
// but running the whole batch of lines for one trace cycle as parallel GPU
// threads (one WebGL2 fragment-shader invocation per line) instead of
// splitting jobs across a handful of CPU worker threads. Runs inside its own
// dedicated Worker (via OffscreenCanvas) so the blocking parts of WebGL
// (readPixels et al) never touch the main thread, exactly like the CPU
// workers above — this is an alternative back end for the exact same
// message protocol (shapes/{reqId,S,poses,jobs,q,...} in, {reqId,verts,mags,
// jobCount} out), so finishTrace/onTraceWorkerMessage need no changes at all.
//
// Validated (in a standalone prototype, before porting here) to match the
// CPU tracer's trajectories and termination steps to float32 precision, and
// measured ~2x faster than the 4-worker CPU pool for the "live" retrace tier
// that actually drives the lockstep cycle rate during motion — the tier that
// matters for this whole plan. It's measurably slower than the CPU pool for
// the infrequent, one-off full-quality settle-trace (every line in a GPU
// batch shares one step loop, so a single slow-to-terminate line forces the
// whole batch toward the full step budget, unlike independent CPU workers) —
// accepted as a reasonable trade for a single, simpler tracer implementation
// rather than a permanent two-tier GPU/CPU split.
// Used twice: here (outer/page scope) to substitute literal values into the
// GLSL text below via ${MAX_BODIES}/${OCC_DIM} at page-load time, and again
// (re-declared, separately) inside the worker source string itself for that
// worker's own runtime JS (array sizing etc.) once it's instantiated.
const MAX_BODIES=8, OCC_DIM=24;
const GPU_TRACE_WORKER_SRC=`
const MAX_BODIES=8, OCC_DIM=24;
let gl=null;
function tryInitGL(){
  try{
    const canvas=new OffscreenCanvas(4,4);
    const ctx=canvas.getContext('webgl2');
    if(!ctx) return null;
    if(!ctx.getExtension('EXT_color_buffer_float')) return null;
    return ctx;
  }catch(e){ return null; }
}
gl=tryInitGL();
const GPU_OK=!!gl;
self.postMessage({type:'gpu-probe', supported:GPU_OK,
  maxTextureSize: GPU_OK?gl.getParameter(gl.MAX_TEXTURE_SIZE):0,
  max3DTextureSize: GPU_OK?gl.getParameter(gl.MAX_3D_TEXTURE_SIZE):0});

function compileShader(type,src){
  const sh=gl.createShader(type);
  gl.shaderSource(sh,src); gl.compileShader(sh);
  if(!gl.getShaderParameter(sh,gl.COMPILE_STATUS)) throw new Error('shader: '+gl.getShaderInfoLog(sh));
  return sh;
}
function linkProgram(vsSrc,fsSrc){
  const vs=compileShader(gl.VERTEX_SHADER,vsSrc), fs=compileShader(gl.FRAGMENT_SHADER,fsSrc);
  const prog=gl.createProgram();
  gl.attachShader(prog,vs); gl.attachShader(prog,fs); gl.linkProgram(prog);
  if(!gl.getProgramParameter(prog,gl.LINK_STATUS)) throw new Error('link: '+gl.getProgramInfoLog(prog));
  return prog;
}
function makeTex(w,h,data){
  const tex=gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D,tex);
  gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA32F,w,h,0,gl.RGBA,gl.FLOAT,data);
  gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);
  return tex;
}
function make3DTex(w,h,d,data){
  const tex=gl.createTexture();
  gl.bindTexture(gl.TEXTURE_3D,tex);
  gl.texImage3D(gl.TEXTURE_3D,0,gl.R8,w,h,d,0,gl.RED,gl.UNSIGNED_BYTE,data);
  gl.texParameteri(gl.TEXTURE_3D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_3D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_3D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_3D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_3D,gl.TEXTURE_WRAP_R,gl.CLAMP_TO_EDGE);
  return tex;
}
function makeFBO(tex,tex2){
  const fbo=gl.createFramebuffer();
  gl.bindFramebuffer(gl.FRAMEBUFFER,fbo);
  gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,tex,0);
  if(tex2){
    gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT1,gl.TEXTURE_2D,tex2,0);
    gl.drawBuffers([gl.COLOR_ATTACHMENT0,gl.COLOR_ATTACHMENT1]);
  }
  return fbo;
}
const VS_QUAD=\`#version 300 es
in vec2 aPos;
void main(){ gl_Position=vec4(aPos,0.0,1.0); }
\`;
const FS_STEP=\`#version 300 es
precision highp float;
precision highp sampler2D;
precision highp sampler3D;
uniform sampler2D uPos;
uniform sampler2D uCharge;
uniform sampler2D uDipolePos;
uniform sampler2D uDipoleMom;
uniform sampler3D uOcc;
uniform sampler2D uSeedXYZDir;
uniform sampler2D uOwnBodyTex;
uniform int uNc, uNd, uNumBodies;
uniform float uSoft, uH, uOff, uMaxR;
uniform float uOwnEscapeByBody[${MAX_BODIES}];
uniform bool uRk4;
uniform vec3 uSceneCenter;
uniform vec3 uBodyPos[${MAX_BODIES}];
uniform mat3 uBodyRotT[${MAX_BODIES}];
uniform vec3 uBodyMin[${MAX_BODIES}];
uniform vec3 uBodyCell[${MAX_BODIES}];
uniform ivec3 uBodyDims[${MAX_BODIES}];
uniform float uBodyRadius[${MAX_BODIES}];
layout(location=0) out vec4 outState;
layout(location=1) out vec4 outMag;
const float INV4PI = 0.07957747154594767;
vec3 fieldCoreFull(vec3 p, out float mag){
  vec3 B = vec3(0.0);
  for(int i=0;i<8192;i++){
    if(i>=uNc) break;
    vec4 c = texelFetch(uCharge, ivec2(i,0), 0);
    vec3 d = p - c.xyz;
    float r2 = dot(d,d) + uSoft;
    float inv = c.w / (r2*sqrt(r2));
    B += d*inv;
  }
  for(int i=0;i<8192;i++){
    if(i>=uNd) break;
    vec3 dp = texelFetch(uDipolePos, ivec2(i,0), 0).xyz;
    vec3 dm = texelFetch(uDipoleMom, ivec2(i,0), 0).xyz;
    if(dm.x==0.0 && dm.y==0.0 && dm.z==0.0) continue;
    vec3 d = p - dp;
    float r2 = dot(d,d) + uSoft;
    float r = sqrt(r2);
    float inv3 = 1.0/(r2*r);
    float mr = dot(dm,d)/r2;
    B += (3.0*mr*d - dm)*inv3;
  }
  B *= INV4PI;
  mag = length(B);
  return B;
}
vec3 dirVec(vec3 p, float dirSign, out float mag){
  vec3 B=fieldCoreFull(p,mag);
  float inv = dirSign/max(mag,1e-30);
  return B*inv;
}
bool nearBody(int k, vec3 worldP){
  vec3 d = worldP - uBodyPos[k];
  vec3 local = uBodyRotT[k] * d;
  ivec3 dims = uBodyDims[k];
  vec3 fidx = (local - uBodyMin[k]) / uBodyCell[k];
  ivec3 ci = ivec3(floor(fidx));
  for(int a=-1;a<=1;a++){
    for(int b=-1;b<=1;b++){
      for(int c=-1;c<=1;c++){
        ivec3 idx = ci + ivec3(a,b,c);
        if(idx.x<0||idx.y<0||idx.z<0||idx.x>=dims.x||idx.y>=dims.y||idx.z>=dims.z) continue;
        float v = texelFetch(uOcc, ivec3(idx.x, idx.y, idx.z + k*${OCC_DIM}), 0).r;
        if(v > 0.5) return true;
      }
    }
  }
  return false;
}
bool hitsAnyBody(vec3 p, vec3 seedStart, int ownBody, float ownEscape){
  for(int k=0;k<${MAX_BODIES};k++){
    if(k>=uNumBodies) break;
    vec3 rd = p - uBodyPos[k];
    if(dot(rd,rd) > uBodyRadius[k]*uBodyRadius[k]) continue;
    if(!nearBody(k,p)) continue;
    if(k!=ownBody) return true;
    vec3 ds = p - seedStart;
    if(dot(ds,ds) > ownEscape*ownEscape) return true;
  }
  return false;
}
void main(){
  int lineIdx = int(gl_FragCoord.x);
  vec4 seedMeta = texelFetch(uSeedXYZDir, ivec2(lineIdx,0), 0);
  int ownBody = int(texelFetch(uOwnBodyTex, ivec2(lineIdx,0), 0).r + 0.5);
  float dirSign = seedMeta.w;
  vec3 seedStart = seedMeta.xyz;
  vec4 prev = texelFetch(uPos, ivec2(lineIdx,0), 0);
  if(prev.w <= 0.0){ outState = vec4(prev.xyz, -1.0); outMag = vec4(0.0); return; }
  vec3 p = prev.xyz;
  float m0, mtmp;
  vec3 k1=dirVec(p,dirSign,m0);
  vec3 np;
  if(uRk4){
    vec3 k2=dirVec(p+k1*uH*0.5,dirSign,mtmp);
    vec3 k3=dirVec(p+k2*uH*0.5,dirSign,mtmp);
    vec3 k4=dirVec(p+k3*uH,dirSign,mtmp);
    np = p + uH/6.0*(k1+2.0*k2+2.0*k3+k4);
  } else {
    vec3 k2=dirVec(p+k1*uH*0.5,dirSign,mtmp);
    np = p + uH*k2;
  }
  vec3 rc = np - uSceneCenter;
  bool term = dot(rc,rc) > uMaxR*uMaxR;
  if(!term) term = hitsAnyBody(np, seedStart, ownBody, uOwnEscapeByBody[ownBody]);
  outState = vec4(np, term ? 0.0 : 1.0);
  outMag = vec4(m0,0.0,0.0,0.0);
}
\`;
let prog=null, quadBuf=null;
if(GPU_OK){
  prog=linkProgram(VS_QUAD,FS_STEP);
  quadBuf=gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER,quadBuf);
  gl.bufferData(gl.ARRAY_BUFFER,new Float32Array([-1,-1, 1,-1, -1,1, 1,1]),gl.STATIC_DRAW);
}
let BODY_SHAPES=[], occTex=null;
function packOcc(){
  const data=new Uint8Array(OCC_DIM*OCC_DIM*OCC_DIM*MAX_BODIES);
  BODY_SHAPES.forEach((sh,k)=>{
    for(let z=0;z<sh.nz;z++)for(let y=0;y<sh.ny;y++)for(let x=0;x<sh.nx;x++){
      data[((k*OCC_DIM+z)*OCC_DIM+y)*OCC_DIM+x]=sh.occ[(z*sh.ny+y)*sh.nx+x]?255:0;
    }
  });
  if(occTex) gl.deleteTexture(occTex);
  occTex=make3DTex(OCC_DIM,OCC_DIM,OCC_DIM*MAX_BODIES,data);
}
// magnitude at a single point, CPU-side (no direction/exclusion, matching
// fieldCore(S,x,y,z,-1,-1)) — used only for each line's very first history
// row (the seed point itself), which precedes the GPU step loop and so never
// gets a magnitude from it; purely cosmetic (log|B| colour mapping).
function seedMagnitude(S,x,y,z){
  let bx=0,by=0,bz=0; const soft=S.soft, INV4PI=1/(4*Math.PI);
  for(let i=0;i<S.nc;i++){
    const dx=x-S.cP[i*3],dy=y-S.cP[i*3+1],dz=z-S.cP[i*3+2];
    const r2=dx*dx+dy*dy+dz*dz+soft;
    const inv=S.cQ[i]/(r2*Math.sqrt(r2));
    bx+=dx*inv; by+=dy*inv; bz+=dz*inv;
  }
  for(let i=0;i<S.nd;i++){
    const mx=S.dD[i*3],my=S.dD[i*3+1],mz=S.dD[i*3+2];
    if(mx===0&&my===0&&mz===0) continue;
    const dx=x-S.dP[i*3],dy=y-S.dP[i*3+1],dz=z-S.dP[i*3+2];
    const r2=dx*dx+dy*dy+dz*dz+soft;
    const r=Math.sqrt(r2), inv3=1/(r2*r);
    const mr=(mx*dx+my*dy+mz*dz)/r2;
    bx+=(3*mr*dx-mx)*inv3; by+=(3*mr*dy-my)*inv3; bz+=(3*mr*dz-mz)*inv3;
  }
  bx*=INV4PI; by*=INV4PI; bz*=INV4PI;
  return Math.sqrt(bx*bx+by*by+bz*bz);
}
function traceBatchGPU(S,poses,jobs,q,sceneDiag,sceneCenter){
  const numLines=jobs.length;
  const nc=S.nc||1, nd=S.nd||1;
  const chargeData=new Float32Array(nc*4);
  for(let i=0;i<S.nc;i++){ chargeData[i*4]=S.cP[i*3]; chargeData[i*4+1]=S.cP[i*3+1]; chargeData[i*4+2]=S.cP[i*3+2]; chargeData[i*4+3]=S.cQ[i]; }
  const dipPosData=new Float32Array(nd*4), dipMomData=new Float32Array(nd*4);
  for(let i=0;i<S.nd;i++){
    dipPosData[i*4]=S.dP[i*3]; dipPosData[i*4+1]=S.dP[i*3+1]; dipPosData[i*4+2]=S.dP[i*3+2];
    dipMomData[i*4]=S.dD[i*3]; dipMomData[i*4+1]=S.dD[i*3+1]; dipMomData[i*4+2]=S.dD[i*3+2];
  }
  const chargeTex=makeTex(nc,1,chargeData), dipPosTex=makeTex(nd,1,dipPosData), dipMomTex=makeTex(nd,1,dipMomData);

  const off=sceneDiag*0.006, maxR=sceneDiag*2.1;
  const seedXYZDirData=new Float32Array(numLines*4), ownBodyData=new Float32Array(numLines*4);
  const initStateData=new Float32Array(numLines*4), initMagData=new Float32Array(numLines*4);
  jobs.forEach((job,i)=>{
    const s=job.s;
    const sx=s.x+s.nx*off, sy=s.y+s.ny*off, sz=s.z+s.nz*off;
    seedXYZDirData[i*4]=sx; seedXYZDirData[i*4+1]=sy; seedXYZDirData[i*4+2]=sz; seedXYZDirData[i*4+3]=job.dir;
    ownBodyData[i*4]=job.bi;
    initStateData[i*4]=sx; initStateData[i*4+1]=sy; initStateData[i*4+2]=sz; initStateData[i*4+3]=1.0;
    initMagData[i*4]=seedMagnitude(S,sx,sy,sz);
  });
  const seedXYZDirTex=makeTex(numLines,1,seedXYZDirData), ownBodyTex=makeTex(numLines,1,ownBodyData);

  gl.useProgram(prog);
  const aPos=gl.getAttribLocation(prog,'aPos');
  gl.bindBuffer(gl.ARRAY_BUFFER,quadBuf);
  gl.enableVertexAttribArray(aPos);
  gl.vertexAttribPointer(aPos,2,gl.FLOAT,false,0,0);

  let stateA=makeTex(numLines,1,initStateData), magA=makeTex(numLines,1,initMagData);
  let stateB=makeTex(numLines,1,new Float32Array(numLines*4)), magB=makeTex(numLines,1,new Float32Array(numLines*4));
  let fboA=makeFBO(stateA,magA), fboB=makeFBO(stateB,magB);

  const maxSteps=q.steps;
  const historyTex=makeTex(numLines,maxSteps+1,null), magHistTex=makeTex(numLines,maxSteps+1,null);
  const historyFBO=makeFBO(historyTex,magHistTex);
  gl.bindFramebuffer(gl.READ_FRAMEBUFFER,fboA);
  gl.readBuffer(gl.COLOR_ATTACHMENT0);
  gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER,historyFBO);
  gl.drawBuffers([gl.COLOR_ATTACHMENT0,gl.NONE]);
  gl.blitFramebuffer(0,0,numLines,1, 0,0,numLines,1, gl.COLOR_BUFFER_BIT, gl.NEAREST);
  gl.drawBuffers([gl.COLOR_ATTACHMENT0,gl.COLOR_ATTACHMENT1]);

  gl.viewport(0,0,numLines,1);
  const U=n=>gl.getUniformLocation(prog,n);
  const uPos=U('uPos'),uCharge=U('uCharge'),uDipolePos=U('uDipolePos'),uDipoleMom=U('uDipoleMom'),uOcc=U('uOcc');
  const uSeedXYZDir=U('uSeedXYZDir'), uOwnBodyTex=U('uOwnBodyTex');
  const uNc=U('uNc'),uNd=U('uNd'),uNumBodies=U('uNumBodies');
  const uSoft=U('uSoft'),uH=U('uH'),uOff=U('uOff'),uMaxR=U('uMaxR');
  const uOwnEscapeByBody=U('uOwnEscapeByBody[0]');
  const uRk4=U('uRk4'), uSceneCenter=U('uSceneCenter');
  const uBodyPos=U('uBodyPos[0]'), uBodyRotT=U('uBodyRotT[0]'), uBodyMin=U('uBodyMin[0]');
  const uBodyCell=U('uBodyCell[0]'), uBodyDims=U('uBodyDims[0]'), uBodyRadius=U('uBodyRadius[0]');

  const nb=poses.length;
  const posArr=new Float32Array(nb*3), rotArr=new Float32Array(nb*9), minArr=new Float32Array(nb*3);
  const cellArr=new Float32Array(nb*3), dimsArr=new Int32Array(nb*3), radArr=new Float32Array(nb);
  const escArr=new Float32Array(MAX_BODIES);
  for(let k=0;k<nb;k++){
    posArr.set(poses[k].pos,k*3); rotArr.set(poses[k].R,k*9);
    minArr.set(BODY_SHAPES[k].min,k*3);
    cellArr.set([BODY_SHAPES[k].cellX,BODY_SHAPES[k].cellY,BODY_SHAPES[k].cellZ],k*3);
    dimsArr.set([BODY_SHAPES[k].nx,BODY_SHAPES[k].ny,BODY_SHAPES[k].nz],k*3);
    radArr[k]=BODY_SHAPES[k].radius;
    escArr[k]=BODY_SHAPES[k].marginLocal+off;
  }
  gl.activeTexture(gl.TEXTURE1); gl.bindTexture(gl.TEXTURE_2D,chargeTex); gl.uniform1i(uCharge,1);
  gl.activeTexture(gl.TEXTURE2); gl.bindTexture(gl.TEXTURE_2D,dipPosTex); gl.uniform1i(uDipolePos,2);
  gl.activeTexture(gl.TEXTURE3); gl.bindTexture(gl.TEXTURE_2D,dipMomTex); gl.uniform1i(uDipoleMom,3);
  gl.activeTexture(gl.TEXTURE4); gl.bindTexture(gl.TEXTURE_3D,occTex); gl.uniform1i(uOcc,4);
  gl.activeTexture(gl.TEXTURE5); gl.bindTexture(gl.TEXTURE_2D,seedXYZDirTex); gl.uniform1i(uSeedXYZDir,5);
  gl.activeTexture(gl.TEXTURE6); gl.bindTexture(gl.TEXTURE_2D,ownBodyTex); gl.uniform1i(uOwnBodyTex,6);
  gl.uniform1i(uNc,S.nc); gl.uniform1i(uNd,S.nd); gl.uniform1i(uNumBodies,nb);
  gl.uniform1f(uSoft,S.soft); gl.uniform1f(uH,q.h);
  gl.uniform1f(uOff,off); gl.uniform1f(uMaxR,maxR);
  gl.uniform1fv(uOwnEscapeByBody,escArr);
  gl.uniform1i(uRk4, q.rk4?1:0);
  gl.uniform3f(uSceneCenter,sceneCenter[0],sceneCenter[1],sceneCenter[2]);
  gl.uniform3fv(uBodyPos,posArr); gl.uniformMatrix3fv(uBodyRotT,false,rotArr);
  gl.uniform3fv(uBodyMin,minArr); gl.uniform3fv(uBodyCell,cellArr);
  gl.uniform3iv(uBodyDims,dimsArr); gl.uniform1fv(uBodyRadius,radArr);
  gl.uniform1i(uPos,0);

  const EARLY_EXIT_CHECK_EVERY=40;
  let actualSteps=maxSteps;
  const aliveCheckBuf=new Float32Array(numLines*4);
  for(let s=0;s<maxSteps;s++){
    gl.bindFramebuffer(gl.FRAMEBUFFER,fboB);
    gl.activeTexture(gl.TEXTURE0); gl.bindTexture(gl.TEXTURE_2D,stateA);
    gl.drawArrays(gl.TRIANGLE_STRIP,0,4);

    gl.bindFramebuffer(gl.READ_FRAMEBUFFER,fboB);
    gl.readBuffer(gl.COLOR_ATTACHMENT0);
    gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER,historyFBO);
    gl.drawBuffers([gl.COLOR_ATTACHMENT0,gl.NONE]);
    gl.blitFramebuffer(0,0,numLines,1, 0,s+1,numLines,s+2, gl.COLOR_BUFFER_BIT, gl.NEAREST);
    gl.bindFramebuffer(gl.READ_FRAMEBUFFER,fboB);
    gl.readBuffer(gl.COLOR_ATTACHMENT1);
    gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER,historyFBO);
    gl.drawBuffers([gl.NONE,gl.COLOR_ATTACHMENT1]);
    gl.blitFramebuffer(0,0,numLines,1, 0,s+1,numLines,s+2, gl.COLOR_BUFFER_BIT, gl.NEAREST);
    gl.drawBuffers([gl.COLOR_ATTACHMENT0,gl.COLOR_ATTACHMENT1]);

    const tA=stateA; stateA=stateB; stateB=tA;
    const tM=magA; magA=magB; magB=tM;
    const tF=fboA; fboA=fboB; fboB=tF;

    if((s+1)%EARLY_EXIT_CHECK_EVERY===0){
      gl.bindFramebuffer(gl.FRAMEBUFFER,fboA);
      gl.readBuffer(gl.COLOR_ATTACHMENT0);
      gl.readPixels(0,0,numLines,1,gl.RGBA,gl.FLOAT,aliveCheckBuf);
      let anyAlive=false;
      for(let li=0;li<numLines;li++){ if(aliveCheckBuf[li*4+3]>0.0){ anyAlive=true; break; } }
      if(!anyAlive){ actualSteps=s+1; break; }
    }
  }

  gl.bindFramebuffer(gl.FRAMEBUFFER,historyFBO);
  gl.readBuffer(gl.COLOR_ATTACHMENT0);
  const posOut=new Float32Array(numLines*(actualSteps+1)*4);
  gl.readPixels(0,0,numLines,actualSteps+1,gl.RGBA,gl.FLOAT,posOut);
  gl.readBuffer(gl.COLOR_ATTACHMENT1);
  const magOut=new Float32Array(numLines*(actualSteps+1)*4);
  gl.readPixels(0,0,numLines,actualSteps+1,gl.RGBA,gl.FLOAT,magOut);
  gl.bindFramebuffer(gl.FRAMEBUFFER,null);

  gl.deleteTexture(chargeTex); gl.deleteTexture(dipPosTex); gl.deleteTexture(dipMomTex);
  gl.deleteTexture(seedXYZDirTex); gl.deleteTexture(ownBodyTex);
  gl.deleteTexture(stateA); gl.deleteTexture(stateB); gl.deleteTexture(magA); gl.deleteTexture(magB);
  gl.deleteTexture(historyTex); gl.deleteTexture(magHistTex);
  gl.deleteFramebuffer(fboA); gl.deleteFramebuffer(fboB); gl.deleteFramebuffer(historyFBO);

  const verts=[], mags=[];
  for(let li=0;li<numLines;li++){
    const pts=[], ptMags=[];
    for(let row=0;row<=actualSteps;row++){
      const o=(row*numLines+li)*4;
      const w=posOut[o+3];
      if(w<0) break;
      pts.push(posOut[o],posOut[o+1],posOut[o+2]);
      ptMags.push(magOut[o]);
      if(w===0) break;
    }
    const n=pts.length/3;
    for(let k=0;k<n-1;k++) for(const idx of [k,k+1]){
      verts.push(pts[idx*3],pts[idx*3+1],pts[idx*3+2]);
      mags.push(ptMags[idx]);
    }
  }
  return {verts:new Float32Array(verts), mags:new Float32Array(mags)};
}
self.onmessage=function(e){
  if(e.data.type==='shapes'){ BODY_SHAPES=e.data.shapes; if(GPU_OK) packOcc(); return; }
  const {reqId,S,poses,jobs,q,sceneDiag,sceneCenter}=e.data;
  try{
    const {verts,mags}=traceBatchGPU(S,poses,jobs,q,sceneDiag,sceneCenter);
    self.postMessage({reqId,verts,mags,jobCount:jobs.length},[verts.buffer,mags.buffer]);
  }catch(err){
    self.postMessage({reqId,error:err.message});
  }
};
`;

// Field-line tracing is pure math (fieldCore/traceLine touch no THREE.js or
// DOM state), so it runs on a pool of background Workers instead of the
// main thread. It used to run synchronously inline — correct, but it
// blocked rendering for its whole duration (300-500ms+ per retrace, i.e.
// the render loop dropped to a few FPS during any active motion). A single
// worker fixed that blocking, but a retrace's *own* duration still sets how
// stale the displayed lines can be relative to a moving/spinning body —
// and that duration scales directly with how many lines are requested.
// Splitting one retrace's job list across several workers in parallel
// (using the machine's actual cores) cuts that duration roughly by the
// worker count, without reducing the line count, step budget, or accuracy
// at all — it's wall-clock parallelism, not a quality trade-off.
const TRACE_WORKER_COUNT=Math.max(1,Math.min(navigator.hardwareConcurrency||4,4));
const traceWorkers=Array.from({length:TRACE_WORKER_COUNT},()=>
  new Worker(URL.createObjectURL(new Blob([TRACE_WORKER_SRC],{type:'text/javascript'}))));
function finishTrace(parts,fullQuality){
  let totalVerts=0, totalJobs=0;
  for(const p of parts){ totalVerts+=p.verts.length; totalJobs+=p.jobCount; }
  const allVerts=new Float32Array(totalVerts), allMags=new Float32Array(totalVerts/3);
  let vo=0,mo=0;
  for(const p of parts){ allVerts.set(p.verts,vo); vo+=p.verts.length; allMags.set(p.mags,mo); mo+=p.mags.length; }
  let lmin=1e30,lmax=-1e30;
  for(let i=0;i<allMags.length;i++){
    const m=allMags[i]; if(m<=0) continue; const l=Math.log10(m);
    if(l<lmin)lmin=l; if(l>lmax)lmax=l;
  }
  if(!(lmax>lmin)){lmin=0;lmax=1;}
  const colors=new Float32Array(allMags.length*3), col=new THREE.Color();
  for(let i=0;i<allMags.length;i++){
    const t=Math.max(0,Math.min(1,(Math.log10(allMags[i]||1e-30)-lmin)/(lmax-lmin)));
    col.setHSL(0.62-0.62*t,0.95,0.5+0.15*t);
    colors[i*3]=col.r; colors[i*3+1]=col.g; colors[i*3+2]=col.b;
  }
  const old=ST.lineGroup;
  ST.lineGroup=new THREE.Group();
  const geo=new THREE.BufferGeometry();
  geo.setAttribute('position',new THREE.Float32BufferAttribute(allVerts,3));
  geo.setAttribute('color',new THREE.Float32BufferAttribute(colors,3));
  ST.lineGroup.add(new THREE.LineSegments(geo,new THREE.LineBasicMaterial({vertexColors:true,transparent:true,opacity:0.85})));
  scene.add(ST.lineGroup);
  if(old){scene.remove(old); old.traverse(o=>{o.geometry&&o.geometry.dispose();o.material&&o.material.dispose();});}
  setStatus(fullQuality?totalJobs+' field lines':'');
}
function onTraceWorkerMessage(e){
  const {reqId,verts,mags,jobCount,error}=e.data;
  if(reqId!==ST.pendingTraceReqId) return;   // superseded by a newer request — discard
  if(error){
    ST.pendingTraceReqId=null; ST.tracing=false;
    const btn=document.getElementById('trace'); if(btn) btn.disabled=false;
    if(ST.scrubTraceWanted){ ST.scrubTraceWanted=false; traceAll(false); return; }
    if(ST.pendingFullQuality) setStatus('trace failed: '+error);
    return;
  }
  ST.pendingParts.push({verts,mags,jobCount});
  if(--ST.pendingRemaining>0) return;        // still waiting on other workers' chunks
  ST.pendingTraceReqId=null; ST.tracing=false;
  const btn=document.getElementById('trace'); if(btn) btn.disabled=false;
  if(ST.scrubTraceWanted){
    // the user scrubbed to a newer position while this trace was still in
    // flight — it's already stale, so skip rendering it (avoids a flash of
    // mismatched lines) and go straight to tracing the current position
    ST.scrubTraceWanted=false;
    traceAll(false);
    return;
  }
  finishTrace(ST.pendingParts,ST.pendingFullQuality);
  onLiveTraceLanded();   // reveal the matching pose now that its lines are ready (no-op unless a live cycle is waiting on this trace)
  maybeContinueLockstep();
}
traceWorkers.forEach(w=>w.onmessage=onTraceWorkerMessage);
// Single dedicated GPU worker: unlike the CPU pool (splitting one trace's
// jobs across a handful of threads), the GPU traces the WHOLE job list as
// one massively-parallel batch, so only one worker/context is needed. Its
// support is unknown until the worker's own OffscreenCanvas+WebGL2 probe
// reports back (gpuTraceSupported stays null — "unknown, treat as
// unavailable" — until then); if unsupported, or a scene's source/body/line
// counts exceed what its fixed-size textures were sized for, traceAll()
// transparently falls back to the CPU pool above for that call.
const gpuTraceWorker=new Worker(URL.createObjectURL(new Blob([GPU_TRACE_WORKER_SRC],{type:'text/javascript'})));
gpuTraceWorker.onmessage=function(e){
  if(e.data.type==='gpu-probe'){
    ST.gpuTraceSupported=e.data.supported;
    ST.gpuMaxTextureSize=e.data.maxTextureSize;
    return;
  }
  onTraceWorkerMessage(e);
};
function canUseGPUTrace(S,jobs){
  return ST.gpuTraceSupported && ST.bodies.length<=MAX_BODIES
    && S.nc<=ST.gpuMaxTextureSize && S.nd<=ST.gpuMaxTextureSize && jobs.length<=ST.gpuMaxTextureSize;
}
export function traceAll(fullQuality){
  if(!ST.bodies.length) return;
  const S=snapshotSources();
  const poses=snapshotPoses();
  const userK=+document.getElementById('lines').value;
  // step budgets must give a line enough arc length to actually travel out
  // to maxR and curve back to the opposite pole (worst case ~2*pi*maxR of
  // path); too small a budget doesn't make lines "wrong", it just stops
  // them mid-flight in open space before they ever reach a sink or the
  // bounding sphere — i.e. they never get a proper termination at all.
  // K (line count) always matches the user's own "lines/pole" slider for
  // both tiers — a hidden cap here would make the displayed count disagree
  // with what's shown on screen, which is worse than the extra lag.
  const q=fullQuality
    ? {h:ST.sceneDiag/320, steps:4500, rk4:true, K:userK}
    : {h:ST.sceneDiag/170, steps:900, rk4:false, K:userK};
  const seedLists=gatherSeeds();
  const jobs=[];
  seedLists.forEach((list,bi)=>{
    for(const s of pickSeeds(list,+1,q.K)) jobs.push({s,dir:+1,bi});
    for(const s of pickSeeds(list,-1,q.K)) jobs.push({s,dir:-1,bi});
  });
  ST.tracing=true;
  const btn=document.getElementById('trace'); btn.disabled=true;
  if(fullQuality) setStatus('tracing…',true);
  const reqId=++ST.traceReqId;
  ST.pendingTraceReqId=reqId; ST.pendingFullQuality=fullQuality; ST.pendingParts=[];
  if(jobs.length===0){
    ST.pendingRemaining=0;
    finishTrace([],fullQuality);
    ST.pendingTraceReqId=null; ST.tracing=false; btn.disabled=false;
    onLiveTraceLanded();
    maybeContinueLockstep();
    return;
  }
  if(canUseGPUTrace(S,jobs)){
    ST.pendingRemaining=1;
    gpuTraceWorker.postMessage({reqId,S,poses,jobs,q,sceneDiag:ST.sceneDiag,sceneCenter:ST.sceneCenter});
    return;
  }
  const nW=traceWorkers.length, chunks=[];
  const chunkSize=Math.ceil(jobs.length/nW);
  for(let i=0;i<jobs.length;i+=chunkSize) chunks.push(jobs.slice(i,i+chunkSize));
  ST.pendingRemaining=chunks.length;
  chunks.forEach((chunk,i)=>traceWorkers[i].postMessage({reqId,S,poses,jobs:chunk,q,sceneDiag:ST.sceneDiag,sceneCenter:ST.sceneCenter}));
}
export function requestTrace(full){ traceAll(full); }
