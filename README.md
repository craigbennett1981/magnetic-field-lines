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

Material (permanent magnet grade, soft magnetic, paramagnetic, diamagnetic, or non-magnetic/plastic) and initial magnetization direction are **not** part of the STL — every body defaults to NdFeB N42 on import and is configured afterwards in the **Bodies** panel, including motion behavior (static, free/dynamic, spinning, oscillating, sliding).

## Demos

If you just want to see it working, use the three **demo** buttons in the sidebar (procedurally generated geometry, no STL needed) — they cover a magnet snapping onto another magnet, a spinning magnet re-magnetizing an iron plate, and a magnet dropping onto a conductive (eddy-current) plate.
