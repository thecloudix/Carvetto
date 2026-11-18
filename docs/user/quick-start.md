# Quick start

This walk-through makes a small sign: an outline with text, pocketed, cut out with holding tabs and
saved as G-code. Allow about fifteen minutes. Every menu name below is the one in the program.

## 1. Start a job

1. Choose **File > New Job...** and enter the size of your material (width, height, thickness) and the
   units you prefer. Confirm.
2. Check the result with **Setup > Job Setup...** if you need to change the material, the Z zero position
   or the datum later.

Save early and often with **File > Save** (or **File > Save As...** the first time). Jobs are saved as `.carv` files.

## 2. Bring in or draw the shape

- To start from artwork, choose **File > Import DXF...**. The drawing appears on the sheet.
  (**File > Import Bitmap...** traces a picture instead; see the [drawing guide](drawing.md).)
- Otherwise pick **Rectangle**, **Circle** or **Polyline** in the Tools toolbar and draw on the sheet.
  Use **Edit > Undo** if you slip.
- Use **View > Zoom to Material** to see the whole job.

## 3. Add text

1. Pick the **Text** tool and click on the sheet, or click on a curve to put the text along it.
2. Type, then press Ctrl+Enter to place it. Click a text later to change it.
3. **Edit > Convert Text to Curves** turns text into ordinary shapes if you want to edit its nodes.

## 4. Make the toolpaths

Open the Toolpaths panel (**View > Panels** lists every panel) and use its Add button.

1. Select the shape to pocket and add a **Pocket** toolpath. Choose a tool from the starter tools (or add
   your own in **Setup > Tools, Machines and Materials...**), set the depth and confirm.
2. Select the outline and add a **Profile** toolpath, cutting outside the line. Add tabs so the part stays
   attached to the sheet; see [toolpaths](toolpaths.md) for how tabs are placed.
3. Select the text and add a **V-Carving** toolpath with a V-bit and a flat clearance tool.

## 5. Preview

Use **View > Simulation View** or the Preview panel to watch the material being removed. Check the depth,
that the tabs are where you want them, and that nothing is cut that should not be.

## 6. Save the G-code

1. Choose the machine and its post-processor in **Setup > Tools, Machines and Materials...** if you have
   not done so yet (the shipped posts cover Grbl, LinuxCNC, Mach3, Mach4 and Marlin).
2. Choose **File > Save Toolpaths...**, pick the toolpaths to include, and save the output file.

Read the file and run an air cut before cutting real material. See [posts and machines](posts-and-machines.md).
