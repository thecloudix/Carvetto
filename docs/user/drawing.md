# Drawing

The Tools toolbar holds the drawing tools: Select, Rectangle, Circle, Ellipse, Polygon, Star, Arc,
Polyline, Node Edit, Text and Dimension. Hover a tool to see its modifier keys (for example Shift keeps
lines at 45 degrees and Alt draws from the centre).

## Sheets and layers

A job can hold several sheets, and each sheet several layers. Pick them in the combo boxes at the end of
the Selectors toolbar. Toolpaths are made from the selected shapes, so layers are mainly for organising.

## Text

The Text tool places point text, text in a box, or text along a curve. Click existing text to edit it.
**Edit > Convert Text to Curves** converts it to vectors. Document variables (**Edit > Document Variables...**)
can be used in text and in toolpath expressions.

## Dimensions

**Edit > Dimensions** holds the dimension tool, its options, and edit/delete for the selected dimension.

## Importing

- **File > Import DXF...** reads 2D drawings.
- **File > Import Bitmap...** traces a picture into vectors.
- **File > Import Parts List...** nests panels from a CSV parts list through a mapping file.
- Export drawings with **File > Export Sheet as DXF...** or **File > Export Selection as DXF...**.

## Drawing in the 3D view

When the 3D view is shown, drawing tools can also be used on the model's zero plane; the shapes land on
the active sheet like any other.
