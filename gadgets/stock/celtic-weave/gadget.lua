-- Celtic Weave Creator (stock gadget, ADR-0113).
-- A grid of vertical and horizontal strands. At every crossing the pattern says which strand is cut:
--   V = cut vertical (the horizontal strand runs over), H = cut horizontal (the vertical runs over),
--   N = cut neither (both continuous). Apply creates the strand pieces (open vectors, layer
--   "Weave rails") and one cross-section profile (closed vector, layer "Weave cross section").
local cv = carvetto

local form = cv.ui.form({
  title = "Celtic Weave Creator",
  fields = {
    {name = "columns", label = "Vertical strands", kind = "number", default = 4, min = 1, max = 30},
    {name = "rows", label = "Horizontal strands", kind = "number", default = 4, min = 1, max = 30},
    {name = "spacing", label = "Strand spacing (mm)", kind = "number", default = 20, min = 1, max = 1000},
    {name = "strand_width", label = "Strand width (mm)", kind = "number", default = 6, min = 0.1, max = 500},
    {name = "pattern", label = "Intersections", kind = "choice",
     choices = {"Alternating", "All cut vertical", "All cut horizontal", "Cut neither", "Custom"},
     default = "Alternating"},
    {name = "custom", label = "Custom (rows top to bottom, V/H/N, '/' between rows)", kind = "text",
     default = ""},
    {name = "section", label = "Cross section", kind = "choice",
     choices = {"Round", "Flat", "Pointed"}, default = "Round"},
    {name = "section_height", label = "Cross section height (mm)", kind = "number", default = 3,
     min = 0.1, max = 500},
  },
})
if form == nil then return end

local cols = math.floor(form.columns)
local rows = math.floor(form.rows)
local s = form.spacing
local w = form.strand_width
if w >= s then
  cv.ui.message("The strand width must be smaller than the strand spacing.")
  return
end

-- state(i, j): i = 0..cols-1 left to right, j = 0..rows-1 bottom to top
local function parseCustom(text)
  local lines = {}
  for line in (text .. "/"):gmatch("([^/]*)/") do lines[#lines + 1] = line:gsub("%s", ""):upper() end
  if #lines ~= rows then return nil, "The custom pattern needs " .. rows .. " rows." end
  for r = 1, rows do
    if #lines[r] ~= cols then return nil, "Row " .. r .. " of the custom pattern needs " .. cols .. " letters." end
    if lines[r]:find("[^VHN]") then return nil, "Only V, H and N are allowed in the custom pattern." end
  end
  return lines
end

local custom
if form.pattern == "Custom" then
  local err
  custom, err = parseCustom(form.custom)
  if not custom then cv.ui.message(err); return end
end

local function state(i, j)
  local p = form.pattern
  if p == "All cut vertical" then return "V" end
  if p == "All cut horizontal" then return "H" end
  if p == "Cut neither" then return "N" end
  if p == "Custom" then
    local row = custom[rows - j]
    return row:sub(i + 1, i + 1)
  end
  return ((i + j) % 2 == 0) and "V" or "H"
end

local railLayer = cv.layer.create("Weave rails", "#2060c0")
local sectionLayer = cv.layer.create("Weave cross section", "#c06020")
local created = {}

-- Pieces of a strand from `a` to `b` with a gap of one strand width at every cut position.
local function pieces(a, b, cuts)
  table.sort(cuts)
  local out, start = {}, a
  for _, c in ipairs(cuts) do
    local stop = c - w / 2
    if stop - start > 1e-9 then out[#out + 1] = {start, stop} end
    start = c + w / 2
  end
  if b - start > 1e-9 then out[#out + 1] = {start, b} end
  return out
end

local ext = s / 2
local x1, y1 = (cols - 1) * s, (rows - 1) * s
for i = 0, cols - 1 do
  local cuts = {}
  for j = 0, rows - 1 do
    if state(i, j) == "V" then cuts[#cuts + 1] = j * s end
  end
  for _, p in ipairs(pieces(-ext, y1 + ext, cuts)) do
    created[#created + 1] = cv.object.create_contour(railLayer, {{i * s, p[1]}, {i * s, p[2]}}, false)
  end
end
for j = 0, rows - 1 do
  local cuts = {}
  for i = 0, cols - 1 do
    if state(i, j) == "H" then cuts[#cuts + 1] = i * s end
  end
  for _, p in ipairs(pieces(-ext, x1 + ext, cuts)) do
    created[#created + 1] = cv.object.create_contour(railLayer, {{p[1], j * s}, {p[2], j * s}}, false)
  end
end

-- Cross section profile, to the right of the grid, standing on its base line.
local cx = x1 + 2 * s
local h = form.section_height
local hw = w / 2
local section
if form.section == "Round" then
  section = cv.object.create_contour(sectionLayer, {{cx - hw, 0}, {cx + hw, 0}}, true, {-2 * h / w, 0})
elseif form.section == "Flat" then
  section = cv.object.create_contour(sectionLayer, {{cx - hw, 0}, {cx + hw, 0}, {cx + hw, h}, {cx - hw, h}}, true)
else
  section = cv.object.create_contour(sectionLayer, {{cx - hw, 0}, {cx + hw, 0}, {cx, h}}, true)
end
cv.selection.set(created)
cv.log("Celtic weave: " .. #created .. " strand pieces and one " .. form.section:lower() .. " cross section")
