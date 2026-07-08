# magnetic-field-lines
Magnetic Field Lines Visualisation

A single-file, browser-based sandbox for visualizing magnetic field lines around STL geometry. Drop in an STL, assign materials, and trace the resulting field. Open `index.html` in a browser to run it — there's no build step or server required.

## Creating input files

The simulator loads geometry from **STL files** (binary or ASCII), dropped onto the panel on the left or picked via the file browser. A few things to know when preparing your own STL:

### Each disconnected solid becomes a separate body

On load, the mesh is automatically split into separate rigid bodies by connectivity — any set of triangles that share vertices forms one body, and each disconnected shell becomes its own independent object. This means you can model an entire scene (e.g. two magnets and a plate) as **one STL file containing several separate solids** — there's no need to import parts individually. Just make sure parts that should move independently (e.g. a magnet that should be free to fly and snap) are not welded/unioned into the same mesh as anything else.

### Solids should be watertight (closed, manifold)

Each body needs a well-defined inside and outside so the simulator can compute its volume, mass, and interior fill for the magnetization/eddy-current models. Small gaps or non-manifold edges usually still work, but a badly non-watertight mesh falls back to a single crude lump approximation at the centroid, which will look and behave wrong. Most CAD/mesh tools (Blender, FreeCAD, OpenSCAD, MeshLab, PrusaSlicer's repair tool, etc.) can export or repair watertight STLs.

### Triangle count can be low — the simulator subdivides internally

You don't need a high-poly mesh for good results. Simple primitives (boxes, cylinders) exported straight from CAD work fine — the app automatically subdivides large flat faces internally when building the charge/field model, so even a handful of triangles per face still produces a smooth, accurate field.

### Winding order matters

Triangle vertices should follow the standard STL convention (counter-clockwise when viewed from outside the solid). The simulator derives each triangle's outward normal from vertex order via the right-hand rule — it does **not** use any normal vectors stored in the file — so inverted/flipped triangles will show up as reversed magnetization or charge on that face.

### Set the correct STL units after loading

STL files have no embedded unit — the simulator assumes your file's numbers are in one of millimetres, centimetres, or metres, selectable from the **STL units** dropdown once a model is loaded (default: millimetres). This only affects physical quantities derived from real-world scale (mass, gravity, force) — pick whichever matches how you modeled the file, or physical behavior (weight, acceleration) will be off by orders of magnitude.

### Scale and placement

- Position parts that should interact (attract, repel, land on one another) within a reasonable distance of each other — the field solver and camera framing are both based on the overall scene's bounding-box diagonal, so wildly different scales between parts (e.g. a 1 mm part next to a 1 m part) will make one of them effectively invisible or numerically negligible.
- Leave a small gap between parts you want to start apart (e.g. a magnet you want to see "snap" into place); parts that are already touching/overlapping in the file are treated as being in contact from the start.

### Assigning materials

Material (permanent magnet grade, soft magnetic, paramagnetic, diamagnetic, or non-magnetic/plastic) and initial magnetization direction are **not** part of the STL — every body defaults to NdFeB N42 on import and is configured afterwards in the **Bodies** panel.

## Controlling motion

Like material, how a body moves is **not** part of the STL — it's set per body afterwards, via the **motion** dropdown on that body's card. Every body starts as `Static` on import. There are five modes:

| Mode | Behaviour |
|---|---|
| **Static** | Fixed in place. Still generates a field and exerts force/torque on other bodies, but never moves itself. Use it for anchors — the plate a magnet snaps onto, the iron core a rotor spins near, etc. |
| **Free (dynamics)** | Full rigid-body physics: magnetic force and torque from every other body, optional gravity, mass/inertia derived from the body's material density and volume, and collision response (impact + "stick" contact) against other bodies. This is the only mode where a body can accelerate, fall, snap, or bounce on its own. |
| **Spin** | Rotates at a constant rate about an axis you set (as an x/y/z direction — it's normalized automatically, so `0,1,0` and `0,2,0` behave the same), at a given **rpm**. Purely kinematic: the rotation follows the formula exactly regardless of any force acting on it. |
| **Oscillate** | Moves back and forth sinusoidally along an axis, with an **amplitude** (in the model's own units) and a **frequency** in Hz. Also kinematic. |
| **Slide** | Moves at a constant linear velocity (x/y/z, units per second). Also kinematic. |

A few things worth knowing:

- **Kinematic vs. dynamic**: Spin, Oscillate, and Slide *drive* a body's position directly from a formula of time — they ignore incoming magnetic force, so nothing can knock a spinning rotor off its axis. Only `Free` bodies are actually pushed around by the simulation. A typical scene pairs one `Static` or kinematic body (the thing being acted on — an iron plate, a conductor) with one `Free` body (the magnet doing the acting).
- **Switching modes mid-simulation** takes the body's *current* position and orientation as the new reference — e.g. switching a body to `Oscillate` makes it oscillate around wherever it happens to be at that moment, not its original load position, and switching to `Spin` spins it in place from its current orientation. Velocity and angular velocity are reset to zero on the switch.
- **Gravity** (checkbox in the Simulation panel) only affects `Free` bodies; kinematic and static bodies are unaffected.
- **Speed** (dropdown in the Simulation panel) scales simulated time relative to real time — slow it down (e.g. `0.05×`) to see fast eddy-current braking or a snap in slow motion, or speed it up for a slow drift.
- **Reset** puts every body back at its original loaded position/orientation and zeroes all velocity and induced magnetization — it does not change any body's material or motion mode.

## Grouping bodies

Multiple bodies can be welded together into one rigid assembly that moves and rotates as a single object — useful when an STL models several parts (e.g. two magnets bolted to a bracket) that should never move independently of each other.

To group bodies: check the selection box on 2 or more body cards in the **Bodies** panel, then click **Group selected**. The group gets one shared **motion** control (the same five modes as an individual body) in place of each member's own — magnetic force on any one member moves the whole assembly together, and collisions are resolved against the group's combined mass, not the individual part. Each member keeps its own material and magnetization controls, since those are still physically per-part. Click **Ungroup** to dissolve the assembly; members freeze in place as independent `Static` bodies from wherever they were at that instant.

## Demos

If you just want to see it working, use the **demo** buttons in the sidebar (procedurally generated geometry, no STL needed) — they cover a magnet snapping onto another magnet, a spinning magnet re-magnetizing an iron plate, a magnet dropping onto a conductive (eddy-current) plate, and a welded pair of magnets moving and rotating together as one group as it snaps onto a third magnet.
