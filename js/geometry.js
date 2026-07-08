// Pure geometry helpers: STL parsing, connected-component body splitting,
// and body-local (centered frame) mesh/voxel utilities. None of this touches
// shared simulation state — everything operates on triangle arrays passed
// in as parameters.

// ---------- STL parsing ----------
export function parseSTL(buffer){
  const dv=new DataView(buffer);
  const head=new TextDecoder().decode(new Uint8Array(buffer,0,Math.min(512,buffer.byteLength)));
  const looksAscii=/^\s*solid/.test(head)&&head.includes('facet');
  if(buffer.byteLength>=84){
    const n=dv.getUint32(80,true);
    if(84+n*50===buffer.byteLength) return parseBinary(dv,n);
  }
  if(looksAscii) return parseAscii(new TextDecoder().decode(buffer));
  return parseBinary(dv,dv.getUint32(80,true));
}
function parseBinary(dv,n){
  const pos=new Float32Array(n*9); let o=84;
  for(let i=0;i<n;i++){ o+=12;
    for(let k=0;k<9;k++){pos[i*9+k]=dv.getFloat32(o,true); o+=4;} o+=2; }
  return pos;
}
function parseAscii(text){
  const re=/vertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)/g;
  const arr=[]; let m;
  while((m=re.exec(text))) arr.push(+m[1],+m[2],+m[3]);
  const n=Math.floor(arr.length/9);
  return Float32Array.from(arr.slice(0,n*9));
}

// ---------- connected components ----------
export function splitBodies(pos){
  const nTri=pos.length/9;
  let mn=[1e30,1e30,1e30], mx=[-1e30,-1e30,-1e30];
  for(let i=0;i<pos.length;i+=3) for(let k=0;k<3;k++){
    mn[k]=Math.min(mn[k],pos[i+k]); mx[k]=Math.max(mx[k],pos[i+k]);
  }
  const diag=Math.hypot(mx[0]-mn[0],mx[1]-mn[1],mx[2]-mn[2])||1;
  const eps=diag*1e-5;
  const parent=new Int32Array(nTri); for(let i=0;i<nTri;i++)parent[i]=i;
  const find=a=>{while(parent[a]!==a){parent[a]=parent[parent[a]];a=parent[a];}return a;};
  const union=(a,b)=>{a=find(a);b=find(b);if(a!==b)parent[b]=a;};
  const map=new Map();
  for(let t=0;t<nTri;t++) for(let v=0;v<3;v++){
    const i=t*9+v*3;
    const key=Math.round((pos[i]-mn[0])/eps)+','+Math.round((pos[i+1]-mn[1])/eps)+','+Math.round((pos[i+2]-mn[2])/eps);
    const prev=map.get(key);
    if(prev===undefined) map.set(key,t); else union(prev,t);
  }
  const groups=new Map();
  for(let t=0;t<nTri;t++){const r=find(t); if(!groups.has(r))groups.set(r,[]); groups.get(r).push(t);}
  const out=[];
  for(const tris of groups.values()){
    const p=new Float32Array(tris.length*9);
    for(let j=0;j<tris.length;j++) p.set(pos.subarray(tris[j]*9,tris[j]*9+9),j*9);
    out.push(p);
  }
  out.sort((a,b)=>b.length-a.length);
  return {parts:out, diag};
}

// ---------- geometry helpers (all in body-local, centered frame) ----------
export function areaCentroid(tris){
  let c=[0,0,0], aSum=0;
  for(let i=0;i<tris.length;i+=9){
    const e1=[tris[i+3]-tris[i],tris[i+4]-tris[i+1],tris[i+5]-tris[i+2]];
    const e2=[tris[i+6]-tris[i],tris[i+7]-tris[i+1],tris[i+8]-tris[i+2]];
    const nx=e1[1]*e2[2]-e1[2]*e2[1], ny=e1[2]*e2[0]-e1[0]*e2[2], nz=e1[0]*e2[1]-e1[1]*e2[0];
    const a=Math.hypot(nx,ny,nz)*0.5;
    c[0]+=(tris[i]+tris[i+3]+tris[i+6])/3*a;
    c[1]+=(tris[i+1]+tris[i+4]+tris[i+7])/3*a;
    c[2]+=(tris[i+2]+tris[i+5]+tris[i+8])/3*a;
    aSum+=a;
  }
  if(aSum>0){c[0]/=aSum;c[1]/=aSum;c[2]/=aSum;}
  return c;
}
export function clusterTris(tris,cell){
  const cells=new Map();
  const addSample=(px,py,pz,area,nx,ny,nz)=>{
    const k=Math.floor(px/cell)+','+Math.floor(py/cell)+','+Math.floor(pz/cell);
    let c=cells.get(k);
    if(!c){c={x:0,y:0,z:0,w:0,sx:0,sy:0,sz:0}; cells.set(k,c);}
    c.x+=px*area;c.y+=py*area;c.z+=pz*area;c.w+=area;c.sx+=nx;c.sy+=ny;c.sz+=nz;
  };
  for(let i=0;i<tris.length;i+=9){
    const ax=tris[i],ay=tris[i+1],az=tris[i+2],bx=tris[i+3],by=tris[i+4],bz=tris[i+5],cx=tris[i+6],cy=tris[i+7],cz=tris[i+8];
    const e1x=bx-ax,e1y=by-ay,e1z=bz-az,e2x=cx-ax,e2y=cy-ay,e2z=cz-az;
    const nx=(e1y*e2z-e1z*e2y)*.5, ny=(e1z*e2x-e1x*e2z)*.5, nz=(e1x*e2y-e1y*e2x)*.5;
    const A=Math.hypot(nx,ny,nz); if(A<1e-14) continue;
    // subdivide triangles that are large relative to the cluster cell —
    // otherwise a coarsely-tessellated flat face (e.g. a box face built
    // from just 2 big triangles) collapses to one or two point charges,
    // which makes a single uniform pole face look like several separate
    // poles once you're close enough to see the discrete source blobs
    const e3x=cx-bx,e3y=cy-by,e3z=cz-bz;
    const maxEdge=Math.max(Math.hypot(e1x,e1y,e1z),Math.hypot(e2x,e2y,e2z),Math.hypot(e3x,e3y,e3z));
    const ns=Math.min(24,Math.max(1,Math.ceil(maxEdge/cell)));
    if(ns===1){
      addSample((ax+bx+cx)/3,(ay+by+cy)/3,(az+bz+cz)/3,A,nx,ny,nz);
      continue;
    }
    const subA=A/(ns*ns), snx=nx/(ns*ns), sny=ny/(ns*ns), snz=nz/(ns*ns);
    for(let row=0;row<ns;row++){
      for(let col=0;col<ns-row;col++){
        let s=(3*row+1)/(3*ns), t=(3*col+1)/(3*ns);
        addSample(ax+s*e1x+t*e2x, ay+s*e1y+t*e2y, az+s*e1z+t*e2z, subA,snx,sny,snz);
        if(row+col<ns-1){
          s=(3*row+2)/(3*ns); t=(3*col+2)/(3*ns);
          addSample(ax+s*e1x+t*e2x, ay+s*e1y+t*e2y, az+s*e1z+t*e2z, subA,snx,sny,snz);
        }
      }
    }
  }
  const n=cells.size, pos=new Float64Array(n*3), S=new Float64Array(n*3);
  let j=0;
  for(const c of cells.values()){
    pos[j*3]=c.x/c.w; pos[j*3+1]=c.y/c.w; pos[j*3+2]=c.z/c.w;
    S[j*3]=c.sx; S[j*3+1]=c.sy; S[j*3+2]=c.sz; j++;
  }
  return {pos,S,n};
}
// Moller-Trumbore, returns t or Infinity
function rayTri(ox,oy,oz,dx,dy,dz,t,i){
  const ax=t[i],ay=t[i+1],az=t[i+2];
  const e1x=t[i+3]-ax,e1y=t[i+4]-ay,e1z=t[i+5]-az;
  const e2x=t[i+6]-ax,e2y=t[i+7]-ay,e2z=t[i+8]-az;
  const px=dy*e2z-dz*e2y, py=dz*e2x-dx*e2z, pz=dx*e2y-dy*e2x;
  const det=e1x*px+e1y*py+e1z*pz;
  if(Math.abs(det)<1e-12) return Infinity;
  const inv=1/det, tx=ox-ax, ty=oy-ay, tz=oz-az;
  const u=(tx*px+ty*py+tz*pz)*inv; if(u<0||u>1) return Infinity;
  const qx=ty*e1z-tz*e1y, qy=tz*e1x-tx*e1z, qz=tx*e1y-ty*e1x;
  const v=(dx*qx+dy*qy+dz*qz)*inv; if(v<0||u+v>1) return Infinity;
  const tt=(e2x*qx+e2y*qy+e2z*qz)*inv;
  return tt>1e-9?tt:Infinity;
}
function insideVotes(tris,x,y,z){
  let votes=0;
  const dirs=[[1,0,0],[0,1,0],[0,0,1]];
  for(const d of dirs){
    let hits=0;
    for(let i=0;i<tris.length;i+=9)
      if(rayTri(x,y,z,d[0],d[1],d[2],tris,i)<Infinity) hits++;
    if(hits%2===1) votes++;
  }
  return votes>=2;
}
// interior fill grid: {occ,nx,ny,nz,min,cell,cells:{pos,vol}}
export function voxelFill(tris,targetCells){
  let mn=[1e30,1e30,1e30],mx=[-1e30,-1e30,-1e30];
  for(let i=0;i<tris.length;i+=3) for(let k=0;k<3;k++){
    mn[k]=Math.min(mn[k],tris[i+k]); mx[k]=Math.max(mx[k],tris[i+k]);
  }
  const ex=Math.max(mx[0]-mn[0],1e-9),ey=Math.max(mx[1]-mn[1],1e-9),ez=Math.max(mx[2]-mn[2],1e-9);
  let pitch=Math.cbrt(ex*ey*ez/targetCells);
  const nx=Math.max(1,Math.min(24,Math.round(ex/pitch)));
  const ny=Math.max(1,Math.min(24,Math.round(ey/pitch)));
  const nz=Math.max(1,Math.min(24,Math.round(ez/pitch)));
  const cx=ex/nx, cy=ey/ny, cz=ez/nz, vol=cx*cy*cz;
  const occ=new Uint8Array(nx*ny*nz);
  const cp=[],cv=[];
  for(let k=0;k<nz;k++)for(let j=0;j<ny;j++)for(let i=0;i<nx;i++){
    const x=mn[0]+(i+.5)*cx, y=mn[1]+(j+.5)*cy, z=mn[2]+(k+.5)*cz;
    if(insideVotes(tris,x,y,z)){ occ[(k*ny+j)*nx+i]=1; cp.push(x,y,z); cv.push(vol); }
  }
  if(cp.length===0){ // non-watertight fallback: single lump at centroid
    cp.push(mn[0]+ex/2,mn[1]+ey/2,mn[2]+ez/2); cv.push(ex*ey*ez*0.5);
    occ.fill(1);
  }
  return {occ,nx,ny,nz,min:mn,cellX:cx,cellY:cy,cellZ:cz,
          cellP:Float64Array.from(cp), cellV:Float64Array.from(cv)};
}
export function downsampleCells(cellP,cellV,groups){
  // merge fill cells into <=groups super-cells (volume-weighted)
  const n=cellV.length; if(n<=groups) return {p:cellP.slice(),v:cellV.slice()};
  let mn=[1e30,1e30,1e30],mx=[-1e30,-1e30,-1e30];
  for(let i=0;i<n;i++) for(let k=0;k<3;k++){
    mn[k]=Math.min(mn[k],cellP[i*3+k]); mx[k]=Math.max(mx[k],cellP[i*3+k]);
  }
  const g=Math.max(1,Math.round(Math.cbrt(groups)));
  const sx=(mx[0]-mn[0])/g+1e-9, sy=(mx[1]-mn[1])/g+1e-9, sz=(mx[2]-mn[2])/g+1e-9;
  const map=new Map();
  for(let i=0;i<n;i++){
    const k=Math.min(g-1,Math.floor((cellP[i*3]-mn[0])/sx))+','+
            Math.min(g-1,Math.floor((cellP[i*3+1]-mn[1])/sy))+','+
            Math.min(g-1,Math.floor((cellP[i*3+2]-mn[2])/sz));
    let c=map.get(k); if(!c){c={x:0,y:0,z:0,v:0};map.set(k,c);}
    const v=cellV[i];
    c.x+=cellP[i*3]*v; c.y+=cellP[i*3+1]*v; c.z+=cellP[i*3+2]*v; c.v+=v;
  }
  const p=[],v=[];
  for(const c of map.values()){p.push(c.x/c.v,c.y/c.v,c.z/c.v); v.push(c.v);}
  return {p:Float64Array.from(p), v:Float64Array.from(v)};
}
