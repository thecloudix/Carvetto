-- DXF Batch Processor (stock gadget, ADR-0113).
-- You choose a folder; every .dxf file in it is imported (millimetres, as the file says) onto its own
-- layer named after the file, and the parts are laid out left to right with a gap, each part's
-- lower-left corner on the row's base line. Optionally each part gets an outside profile toolpath.
-- Files that cannot be read are listed at the end; they never stop the others.
local cv = carvetto

local folder = cv.ui.choose_folder("Choose the folder of DXF files")
if folder == nil then return end

local files = cv.file.read_dxf_folder(folder)
if #files == 0 then
  cv.ui.message("The folder holds no DXF files.")
  return
end

local form = cv.ui.form({
  title = "DXF Batch Processor",
  fields = {
    {name = "gap", label = "Gap between parts (mm)", kind = "number", default = 10, min = 0, max = 1000},
    {name = "profile", label = "Create a profile toolpath per part", kind = "bool", default = false},
    {name = "tool", label = "Tool (profile)", kind = "tool"},
    {name = "cut_depth", label = "Cut depth (mm)", kind = "number", default = 3, min = 0.01, max = 200},
  },
})
if form == nil then return end

local function bounds(contours)
  local x0, y0, x1, y1 = math.huge, math.huge, -math.huge, -math.huge
  for _, c in ipairs(contours) do
    for _, p in ipairs(c.points) do
      x0, y0 = math.min(x0, p[1]), math.min(y0, p[2])
      x1, y1 = math.max(x1, p[1]), math.max(y1, p[2])
    end
  end
  return x0, y0, x1, y1
end

local cursor, imported, problems = 0, 0, {}
for _, f in ipairs(files) do
  if f.error then
    problems[#problems + 1] = f.file .. ": " .. f.error
  elseif #f.contours == 0 then
    problems[#problems + 1] = f.file .. ": no drawable geometry"
  else
    local x0, y0, x1, y1 = bounds(f.contours)
    local dx, dy = cursor - x0, -y0
    local layer = cv.layer.create(f.name)
    local ids = {}
    for _, c in ipairs(f.contours) do
      local pts = {}
      for i, p in ipairs(c.points) do pts[i] = {p[1] + dx, p[2] + dy} end
      ids[#ids + 1] = cv.object.create_contour(layer, pts, c.closed, c.bulges)
    end
    if form.profile then
      cv.toolpath.create("profile", ids, {tool = form.tool, cutDepth = form.cut_depth, side = "outside",
                                          name = f.name})
    end
    cursor = cursor + (x1 - x0) + form.gap
    imported = imported + 1
  end
end
if #problems > 0 then
  cv.ui.message(imported .. " files imported. Not imported:\n" .. table.concat(problems, "\n"))
end
cv.log("DXF batch: " .. imported .. " of " .. #files .. " files imported")
