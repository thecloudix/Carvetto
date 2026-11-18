# Toolpaths

Toolpaths are created in the Toolpaths panel (**View > Panels** shows or hides it). Select the shapes first,
press Add and choose the operation.

## Operations

Profile, Pocket, Drilling, Engraving, V-Carving, Chamfer, Thread Milling, Inlay, Fluting and Prism cover 2D
and 2.5D work. The laser operations are described in the [laser guide](laser.md), the 3D ones in the
[relief guide](relief.md). Each operation opens a form for tool, depths, stepover and strategy; values
may use document variables and expressions.

## Tools, machines and materials

**Setup > Tools, Machines and Materials...** manages the tool database, machine definitions and
materials. A starter set of tools is built in. **Setup > Options...** holds general preferences, such as the
language and units.

## Tabs

Profile toolpaths can leave tabs: short bridges that keep the part attached while the rest is cut. Set
their width and height in the form; the "3D Tabs" entry in the Add menu places them on a finished toolpath.

## Preview and simulation

The Preview panel and **View > Simulation View** show the cut on a virtual block of material.

## Saving

**File > Save Toolpaths...** writes G-code through the post-processor of the machine, as one file or one
file per toolpath. **File > Create Job Sheet...** prints a setup sheet listing tools and operations.
