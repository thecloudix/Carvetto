-- Drag Knife Toolpath (stock gadget, ADR-0113).
-- The blade trails the pivot (the tool centre the machine moves) by the blade offset. The pivot path
-- therefore runs `offset` ahead of the blade tip along the cutting direction. A corner sharper than the
-- tolerance angle is swivelled: the pivot overshoots to P + offset * incoming direction, the blade is
-- lifted to the swivel depth, the pivot turns on an arc of radius `offset` around the corner point P to
-- P + offset * outgoing direction, and the blade goes back to cutting depth. Gentler corners are
-- followed directly. Every contour starts with a lead-in of one blade offset so the blade is aligned
-- when the cut proper begins, and each depth pass repeats the contour.
local cv = carvetto

local selection = cv.selection.objects()
if #selection == 0 then
  cv.ui.message("Select the vectors to cut first.")
  return
end

local form = cv.ui.form({
  title = "Drag Knife Toolpath",
  fields = {
    {name = "tool", label = "Tool", kind = "tool"},
    {name = "offset", label = "Blade offset (mm)", kind = "number", default = 0.5, min = 0.01, max = 50},
    {name = "tolerance", label = "Tolerance angle (degrees)", kind = "number", default = 15, min = 0, max = 179},
    {name = "cut_depth", label = "Cut depth (mm)", kind = "number", default = 1, min = 0.01, max = 100},
    {name = "swivel_depth", label = "Swivel depth (mm)", kind = "number", default = 0.2, min = 0, max = 100},
    {name = "passes", label = "Passes", kind = "number", default = 1, min = 1, max = 50},
    {name = "use_start", label = "Use vector start points", kind = "bool", default = true},
    {name = "feed", label = "Feed rate (mm/min)", kind = "number", default = 1000, min = 1, max = 100000},
    {name = "plunge", label = "Plunge rate (mm/min)", kind = "number", default = 300, min = 1, max = 100000},
    {name = "safe_z", label = "Safe Z (mm)", kind = "number", default = 5, min = 0.1, max = 1000},
    {name = "name", label = "Toolpath name", kind = "text", default = "Drag knife"},
  },
})
if form == nil then return end
if form.swivel_depth > form.cut_depth then
  cv.ui.message("The swivel depth cannot be deeper than the cut depth.")
  return
end

local off = form.offset
local tol = math.rad(form.tolerance)
local passes = math.floor(form.passes)

-- Contour -> list of points (arcs flattened in steps of at most 5 degrees), no repeated points.
local function flatten(c)
  local pts = {}
  local function add(x, y)
    local last = pts[#pts]
    if not last or math.abs(last[1] - x) > 1e-9 or math.abs(last[2] - y) > 1e-9 then pts[#pts + 1] = {x, y} end
  end
  local n = #c.points
  local last = c.closed and n or n - 1
  for i = 1, last do
    local a, b = c.points[i], c.points[i % n + 1]
    add(a[1], a[2])
    local bulge = c.bulges[i] or 0
    if bulge ~= 0 then
      local theta = 4 * math.atan(bulge)
      local chord = math.sqrt((b[1] - a[1]) ^ 2 + (b[2] - a[2]) ^ 2)
      if chord > 1e-12 then
        local r = chord / (2 * math.sin(math.abs(theta) / 2))
        local mx, my = (a[1] + b[1]) / 2, (a[2] + b[2]) / 2
        local nx, ny = -(b[2] - a[2]) / chord, (b[1] - a[1]) / chord
        local d = (chord / 2) / math.tan(theta / 2)  -- signed: left of a->b for counter-clockwise arcs
        local cx, cy = mx + nx * d, my + ny * d
        local a0 = math.atan(a[2] - cy, a[1] - cx)
        local steps = math.max(1, math.ceil(math.abs(theta) / math.rad(5)))
        for k = 1, steps - 1 do
          local ang = a0 + theta * k / steps
          add(cx + r * math.cos(ang), cy + r * math.sin(ang))
        end
      end
    end
  end
  if not c.closed then add(c.points[n][1], c.points[n][2]) end
  if c.closed and #pts > 1 then
    local f, l = pts[1], pts[#pts]
    if math.abs(f[1] - l[1]) < 1e-9 and math.abs(f[2] - l[2]) < 1e-9 then pts[#pts] = nil end
  end
  return pts
end

local contours = {}
for _, id in ipairs(selection) do
  if cv.object.kind(id) == "contour" then
    for _, c in ipairs(cv.object.contours(id)) do
      local pts = flatten(c)
      if #pts >= 2 then contours[#contours + 1] = {pts = pts, closed = c.closed} end
    end
  end
end
if #contours == 0 then
  cv.ui.message("The selection holds no vectors with a length.")
  return
end

local function dist2(p, x, y) return (p[1] - x) ^ 2 + (p[2] - y) ^ 2 end

-- Without "use vector start points": nearest neighbour order, open contours reversed when their end
-- is nearer, closed contours started at their nearest vertex.
if not form.use_start then
  local ordered, cx, cy = {}, 0, 0
  while #contours > 0 do
    local best, bestD, bestRev, bestRot = 1, math.huge, false, 1
    for k, c in ipairs(contours) do
      local n = #c.pts
      if c.closed then
        for i = 1, n do
          local d = dist2(c.pts[i], cx, cy)
          if d < bestD then best, bestD, bestRev, bestRot = k, d, false, i end
        end
      else
        local d1, d2 = dist2(c.pts[1], cx, cy), dist2(c.pts[n], cx, cy)
        if d1 < bestD then best, bestD, bestRev, bestRot = k, d1, false, 1 end
        if d2 < bestD then best, bestD, bestRev, bestRot = k, d2, true, 1 end
      end
    end
    local c = table.remove(contours, best)
    local n = #c.pts
    local pts = {}
    if c.closed then
      for i = 0, n - 1 do pts[#pts + 1] = c.pts[(bestRot - 1 + i) % n + 1] end
    elseif bestRev then
      for i = n, 1, -1 do pts[#pts + 1] = c.pts[i] end
    else
      pts = c.pts
    end
    ordered[#ordered + 1] = {pts = pts, closed = c.closed}
    local last = c.closed and pts[1] or pts[#pts]
    cx, cy = last[1], last[2]
  end
  contours = ordered
end

local moves = {}
local function rapid(x, y, z) moves[#moves + 1] = {kind = "rapid", x = x, y = y, z = z} end
local function feedTo(x, y) moves[#moves + 1] = {kind = "feed", x = x, y = y, feed = form.feed} end
local function plungeTo(z) moves[#moves + 1] = {kind = "plunge", z = z, feed = form.plunge} end
local function retractTo(z) moves[#moves + 1] = {kind = "retract", z = z} end

local function runContour(c, zCut)
  local pts, n = c.pts, #c.pts
  local zSwivel = -math.min(form.swivel_depth, -zCut)
  -- unit directions of the segments; segment k runs from point k to point k % n + 1
  local segs = c.closed and n or n - 1
  local dirs = {}
  for k = 1, segs do
    local a, b = pts[k], pts[k % n + 1]
    local len = math.sqrt(dist2(a, b[1], b[2]))
    dirs[k] = {(b[1] - a[1]) / len, (b[2] - a[2]) / len}
  end
  local function turn(din, dout)
    return math.atan(din[1] * dout[2] - din[2] * dout[1], din[1] * dout[1] + din[2] * dout[2])
  end
  local function swivel(p, din, dout)
    local t = turn(din, dout)
    retractTo(zSwivel)
    local to = {p[1] + off * dout[1], p[2] + off * dout[2]}
    moves[#moves + 1] = {kind = t >= 0 and "arc_ccw" or "arc_cw", x = to[1], y = to[2], z = zSwivel,
                         cx = p[1], cy = p[2], feed = form.feed}
    plungeTo(zCut)
  end
  -- lead-in
  local s, d0 = pts[1], dirs[1]
  rapid(s[1], s[2], form.safe_z)
  plungeTo(zSwivel)
  feedTo(s[1] + off * d0[1], s[2] + off * d0[2])
  plungeTo(zCut)
  -- corners 2..segs (and, for a closed contour, the start corner at the end)
  local last = c.closed and n + 1 or n
  for k = 2, last do
    local din = dirs[k - 1]
    local p = pts[(k - 1) % n + 1]
    local dout = dirs[(k - 1) % segs + 1]
    local atEnd = (k == n and not c.closed)
    feedTo(p[1] + off * din[1], p[2] + off * din[2])
    if not atEnd and math.abs(turn(din, dout)) > tol then swivel(p, din, dout) end
  end
  retractTo(form.safe_z)
end

for _, c in ipairs(contours) do
  for pass = 1, passes do
    runContour(c, -form.cut_depth * pass / passes)
  end
end

local id = cv.toolpath.create_custom(form.name, form.tool, moves)
cv.log("Drag knife: " .. #contours .. " contours, " .. passes .. " passes, " .. #moves .. " moves")
