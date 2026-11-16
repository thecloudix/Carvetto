# ADR-0102: the scripting API surface (design, for owner review)

Status: proposed (T-0330 delivers the host, ADR-0101; T-0331 implements the bindings below)

## Context
The feature inventory says gadgets are Lua add-ons, and that the vendor's API is only documented in an
external SDK ("NOT FOUND" in the manual). This project does not reproduce any vendor SDK. The
compatibility question for the owner is therefore: **do we want API compatibility with existing
third-party gadget scripts?** This ADR assumes **no**: the API is our own, designed from what gadgets
must be able to do (inventory section 6: weave creator, drag knife, keyhole, job-sheet editor, DXF
batch). If compatibility is wanted later, a thin adapter module written from public tutorials can sit on
top without changing this core API; that is a product decision, not taken here.

## Design principles
1. **Small and explicit:** one global table `carvetto`, sub-tables per area, no other globals added
   (standard Lua libraries apart). `carvetto.api_version` is an integer, incremented on breaking change;
   a gadget manifest states the version it needs.
2. **Millimetres and radians**, like core. Angles in degrees only in form fields shown to users.
3. **Handles, not objects:** documents, objects, layers and toolpaths are referred to by integer ids that
   are stable within the document session. A stale id is a script error, never a crash.
4. **Everything that changes the document goes through the undo stack; one gadget run is one undo step**
   (T-0331). Read functions never change anything; a script that errors midway changes nothing (the step
   is rolled back).
5. **Values are plain:** numbers, strings, booleans and arrays. Points are two-element sequences
   (`{1.5, 2}`), because the boundary carries sequences (ADR-0101 item 9);
   contours are arrays of points, with an optional per-vertex bulge in a parallel array.
6. **Failure is an error with a message**, raised with the caller's line; `pcall` works.

## Proposed surface (version 1)
```
carvetto.api_version                                  -- 1
carvetto.log(...)                                     -- like print; shown in the gadget log panel

-- document (read)
carvetto.doc.units()                                  -- "mm" | "inch"
carvetto.doc.job_size()                               -- width, height, thickness (mm)
carvetto.doc.layers()                                 -- array of layer ids
carvetto.layer.name(id), .visible(id), .colour(id)
carvetto.doc.variable(name) / carvetto.doc.set_variable(name, value)   -- document variables

-- vectors
carvetto.selection.objects()                          -- ids of the selected objects
carvetto.object.kind(id)                              -- "contour" | "text" | "bitmap" | ...
carvetto.object.bounds(id)                            -- x0, y0, x1, y1
carvetto.object.contours(id)                          -- array of contours, each {points=..., closed=bool}
carvetto.object.create_contour(layer, points, closed) -- returns the new object id
carvetto.object.create_shape(layer, "rect"|"circle"|"ellipse"|..., params)
carvetto.object.move(id, dx, dy) / rotate / scale / delete(id)
carvetto.selection.set(ids)

-- toolpaths
carvetto.tool.find(name), carvetto.tool.list()
carvetto.toolpath.create(kind, objects, params)       -- kind: "profile" | "pocket" | "drill" | "engrave";
                                                      -- params as the operation's own settings names
carvetto.toolpath.create_custom(name, tool, moves)    -- moves: arrays {kind, x, y, z, feed}; for
                                                      -- gadgets that compute their own paths (drag knife)

-- user interface
carvetto.ui.form(spec)                                -- modal input form; returns a table of values or nil
                                                      -- spec: title, fields {name, label, kind="number"|
                                                      -- "text"|"choice"|"bool"|"layer"|"tool", default, ...}
carvetto.ui.message(text) / carvetto.ui.confirm(text)
```
Not exposed in version 1 (by design): file system, network, process execution, document open/save,
preferences. A gadget may ask for a file through `carvetto.ui.open_file()` / `save_file()` (user picks the
path; the host reads or writes it), which keeps file access explicit and revocable.

## Gadget packaging (T-0331)
A gadget is a folder holding `gadget.lua` and `manifest.json` (`name`, `version`, `api_version`, `menu`
path, optional `shortcut` such as `Ctrl+F5`, `description`). Discovery scans the user gadget folder at
startup and builds the Gadgets menu; Install New Gadget copies a folder or zip there after showing the
manifest; limits can be raised per gadget only by the user, in a confirmation dialog.

## Open questions for the owner
1. API compatibility with existing community scripts: yes (adapter later) or no (this design only)?
2. Should gadgets be able to read and write files without a user prompt (a DXF batch converter wants
   to)? Proposed: only inside a folder the user chose in the form.
3. Default budget (about half a second of VM time, 64 MiB heap): enough for a weave creator on a large
   grid? Proposed to measure with the stock gadgets (T-0332) and set per-gadget manifest hints.
