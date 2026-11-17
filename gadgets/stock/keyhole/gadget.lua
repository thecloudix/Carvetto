-- Keyhole Toolpath (stock gadget, ADR-0113).
-- For every selected vector (its bounding-box centre is the entry hole) the tool plunges to the depth,
-- slides along the slot direction for the slot length and lifts out. Optional preview vectors show the
-- entry hole (circle) and the slot (a rounded slot outline) on the chosen layer.
local cv = carvetto

local selection = cv.selection.objects()
if #selection == 0 then
  cv.ui.message("Select the vectors that mark the keyhole entry holes first.")
  return
end

local form = cv.ui.form({
  title = "Keyhole Toolpath",
  fields = {
    {name = "tool", label = "Tool", kind = "tool"},
    {name = "direction", label = "Slot direction (degrees, 0 = right, 90 = up)", kind = "number", default = 90,
     min = -360, max = 360},
    {name = "depth", label = "Depth (mm)", kind = "number", default = 4, min = 0.01, max = 200},
    {name = "length", label = "Slot length (mm)", kind = "number", default = 10, min = 0.01, max = 2000},
    {name = "feed", label = "Feed rate (mm/min)", kind = "number", default = 600, min = 1, max = 100000},
    {name = "plunge", label = "Plunge rate (mm/min)", kind = "number", default = 200, min = 1, max = 100000},
    {name = "safe_z", label = "Safe Z (mm)", kind = "number", default = 5, min = 0.1, max = 1000},
    {name = "preview", label = "Create preview vectors", kind = "bool", default = false},
    {name = "entry_diameter", label = "Entry hole diameter (mm)", kind = "number", default = 10, min = 0.1, max = 500},
    {name = "slot_diameter", label = "Slot diameter (mm)", kind = "number", default = 5, min = 0.1, max = 500},
    {name = "layer", label = "Preview layer", kind = "layer"},
    {name = "name", label = "Toolpath name", kind = "text", default = "Keyhole"},
  },
})
if form == nil then return end

local a = math.rad(form.direction)
local ux, uy = math.cos(a), math.sin(a)
local moves = {}
local centres = {}
for _, id in ipairs(selection) do
  local x0, y0, x1, y1 = cv.object.bounds(id)
  centres[#centres + 1] = {(x0 + x1) / 2, (y0 + y1) / 2}
end
for _, c in ipairs(centres) do
  moves[#moves + 1] = {kind = "rapid", x = c[1], y = c[2], z = form.safe_z}
  moves[#moves + 1] = {kind = "plunge", z = -form.depth, feed = form.plunge}
  moves[#moves + 1] = {kind = "feed", x = c[1] + ux * form.length, y = c[2] + uy * form.length, feed = form.feed}
  moves[#moves + 1] = {kind = "retract", z = form.safe_z}
end
cv.toolpath.create_custom(form.name, form.tool, moves)

if form.preview then
  local sr = form.slot_diameter / 2
  for _, c in ipairs(centres) do
    cv.object.create_shape(form.layer, "circle", {cx = c[1], cy = c[2], radius = form.entry_diameter / 2})
    -- slot outline: two parallel sides and two half-circle ends (bulge 1 = half circle, counter-clockwise)
    local nx, ny = -uy, ux
    local e = {c[1] + ux * form.length, c[2] + uy * form.length}
    cv.object.create_contour(form.layer, {
      {c[1] - nx * sr, c[2] - ny * sr},
      {e[1] - nx * sr, e[2] - ny * sr},
      {e[1] + nx * sr, e[2] + ny * sr},
      {c[1] + nx * sr, c[2] + ny * sr},
    }, true, {0, 1, 0, 1})
  end
end
cv.log("Keyhole: " .. #centres .. " keyholes")
