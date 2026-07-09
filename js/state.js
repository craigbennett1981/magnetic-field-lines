// Shared mutable simulation state, plus a few static registries/config
// objects. Every other module imports ST and reads/writes its properties
// (e.g. ST.bodies, ST.playing = true) rather than holding its own copy or
// exporting/reassigning plain bindings — many of these fields (bodies,
// groups, sceneCenter, gridHelper, lineGroup, SRC, playing, simTime, ...)
// are reassigned wholesale from several different modules (body loading,
// play/pause, tracing, grouping), and ES module bindings imported with
// `import {x}` cannot be reassigned by the importer — only mutated if
// they're an object. A single shared object sidesteps that restriction
// entirely: reassigning a *property* of an imported object is always fine.
export const ST = {
  bodies: [],
  groups: [],           // rigid groups: members move/rotate together as one body
  nextGroupId: 1,
  sceneDiag: 1,
  sceneCenter: [0,0,0],
  SRC: null,             // flattened field sources (live)
  anySoft: false, anyEddy: false, anyFree: false,
  playing: false, simTime: 0,
  tracing: false,
  // trace request/response bookkeeping (see tracer.js)
  traceReqId: 0, pendingTraceReqId: null, pendingFullQuality: false,
  pendingParts: null, pendingRemaining: 0,
  gpuTraceSupported: false, gpuMaxTextureSize: 0,
  // main-loop / lockstep bookkeeping (see rendering.js)
  lastTS: 0, lastAdvanceReal: null,
  // three.js objects that get replaced wholesale on load/retrace (rendering.js)
  gridHelper: null, lineGroup: null,
  // timeline scrubbing (see rendering.js): one lightweight snapshot per
  // recorded point (pose + soft/eddy magnetization state, NOT field-line
  // geometry — those get re-traced on demand when scrubbing to a point) plus
  // an index into that history for "what's currently displayed". Playback
  // auto-stops once the scene settles or simTime reaches maxTime.
  maxTime: 10, history: [], historyIndex: -1, lastHistorySimTime: 0,
};

export const MU0 = 4*Math.PI*1e-7, INV4PI = 1/(4*Math.PI);

// ---------- materials: Br(T) | muR + Ms(T) | chi ; rho kg/m^3 ; sigma S/m ----
export const MATERIALS = {
  ndfeb_n35:   {name:'NdFeB N35',            cat:'perm', Br:1.17, rho:7500, sigma:6.7e5},
  ndfeb_n42:   {name:'NdFeB N42',            cat:'perm', Br:1.30, rho:7500, sigma:6.7e5},
  ndfeb_n52:   {name:'NdFeB N52',            cat:'perm', Br:1.45, rho:7500, sigma:6.7e5},
  smco:        {name:'SmCo (Sm₂Co₁₇)', cat:'perm', Br:1.05, rho:8400, sigma:1.2e6},
  alnico5:     {name:'Alnico 5',             cat:'perm', Br:1.28, rho:7300, sigma:2.0e6},
  ferrite_c8:  {name:'Hard ferrite (C8)',    cat:'perm', Br:0.39, rho:4900, sigma:1e-4},
  bonded:      {name:'Bonded / flexible',    cat:'perm', Br:0.25, rho:6000, sigma:1e-3},

  iron:        {name:'Iron (pure)',          cat:'soft', muR:5000,  Ms:2.15, rho:7870, sigma:1.0e7},
  steel1018:   {name:'Mild steel (1018)',    cat:'soft', muR:800,   Ms:2.00, rho:7850, sigma:6.0e6},
  sisteel:     {name:'Silicon steel',        cat:'soft', muR:4000,  Ms:1.97, rho:7650, sigma:2.0e6},
  ss430:       {name:'Stainless 430 (ferritic)', cat:'soft', muR:600, Ms:1.50, rho:7700, sigma:1.7e6},
  mumetal:     {name:'Mu-metal',             cat:'soft', muR:80000, Ms:0.75, rho:8700, sigma:1.7e6},
  permalloy:   {name:'Permalloy 80',         cat:'soft', muR:100000,Ms:1.00, rho:8700, sigma:2.0e6},
  ferrite_soft:{name:'Soft ferrite (MnZn)',  cat:'soft', muR:2500,  Ms:0.50, rho:4800, sigma:1},

  ss304:       {name:'Stainless 304 (austenitic)', cat:'para', chi:3e-3,   rho:8000,  sigma:1.4e6},
  aluminum:    {name:'Aluminium',            cat:'para', chi:2.2e-5, rho:2700,  sigma:3.5e7},
  titanium:    {name:'Titanium',             cat:'para', chi:1.8e-4, rho:4500,  sigma:2.4e6},
  platinum:    {name:'Platinum',             cat:'para', chi:2.6e-4, rho:21450, sigma:9.4e6},
  tungsten:    {name:'Tungsten',             cat:'para', chi:6.8e-5, rho:19300, sigma:1.8e7},

  copper:      {name:'Copper',               cat:'dia', chi:-9.6e-6,  rho:8960,  sigma:5.96e7},
  gold:        {name:'Gold',                 cat:'dia', chi:-3.4e-5,  rho:19300, sigma:4.1e7},
  silver:      {name:'Silver',               cat:'dia', chi:-2.4e-5,  rho:10490, sigma:6.3e7},
  bismuth:     {name:'Bismuth',              cat:'dia', chi:-1.66e-4, rho:9780,  sigma:7.7e5},
  graphite:    {name:'Pyrolytic graphite',   cat:'dia', chi:-4e-4,    rho:2200,  sigma:2e5},

  plastic:     {name:'Plastic / non-magnetic', cat:'none', rho:1200, sigma:0}
};
export const MAT_GROUPS = {perm:'Permanent magnets', soft:'Soft magnetic', para:'Paramagnetic', dia:'Diamagnetic', none:'Non-magnetic'};
export const CONDUCTIVE = 1e5;   // sigma above this gets an eddy model

export const SIM = {u:0.001, g:9.81, gravity:false, speed:1, h:1/120};
export const PALETTE=[0x56e6d2,0xe8a33d,0xd06bff,0x6bb2ff,0xff6b8a,0x9dff6b,0xffde59,0xff9d5c];

const statusEl=document.getElementById('status');
export const timebar=document.getElementById('timebar');
export function setStatus(m,busy){statusEl.textContent=m; statusEl.className=busy?'busy':'';}
