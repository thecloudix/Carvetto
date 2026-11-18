# Carvetto user guide

Carvetto is open-source software for designing parts and making the toolpaths that cut them on a CNC
router, mill or laser. This guide is bundled with the program: open it from **Help > User Guide**.

New here? Start with the [quick start](quick-start.md), which takes you from a blank job to a G-code file.

## Guide

- [Quick start](quick-start.md): draw, add text, pocket and profile with tabs, V-carve, preview, save G-code.
- [Drawing](drawing.md): the drawing tools, text, dimensions and importing artwork.
- [Editing](editing.md): selecting, node editing, transforms, copies and other vector operations.
- [Toolpaths](toolpaths.md): the toolpath operations, tools, tabs, preview and saving.
- [3D relief](relief.md): relief models, importing meshes, sculpting and relief toolpaths.
- [Rotary](rotary.md): jobs wrapped around a cylinder.
- [Laser](laser.md): laser cutting, filling and engraving.
- [Gadgets](gadgets.md): scripted extensions and the stock gadgets.
- [Posts and machines](posts-and-machines.md): output formats, machines and materials.
- [FAQ](faq.md): short answers to common questions.

## Reference for advanced users

- [Stock gadgets](../gadgets/stock-gadgets.md): what each shipped gadget does.
- [Post-processor guide](../../posts/README.md): how posts are written, tested and checked for machine safety.
- [Adapting a post for a laser](../laser-post-guide.md).
- [Design decisions (ADR index)](../adr/0001-language-and-ui-toolkit.md): every architectural decision has a
  numbered record in the `docs/adr` folder; the file for each topic is named after it. Useful starting points:
  [the .carv file format](../adr/0003-native-file-format.md),
  [the .pp post format](../adr/0004-pp-post-processor-compatibility.md),
  [post output safety](../adr/0020-post-output-safety.md),
  [the scripting API](../adr/0108-scripting-api-and-gadgets.md),
  [localization](../adr/0123-localization-infrastructure.md).

Menu names in this guide are written like **File > Import DXF...**; the ampersand shortcut markers are left out.
