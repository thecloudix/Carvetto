-- Job Setup Sheet Editor (stock gadget, ADR-0113).
-- The choices are stored in the job as document variables named jobsheet_*; the job sheet reads them
-- (header and footer colour as 0xRRGGBB numbers, sections as 1 = shown / 0 = hidden). "Restore default"
-- sets every variable to its default, which prints the sheet exactly as it was before.
local cv = carvetto

local names = {
  "jobsheet_header_rgb", "jobsheet_footer_rgb", "jobsheet_show_layout", "jobsheet_show_notes",
  "jobsheet_show_material", "jobsheet_show_summary", "jobsheet_show_toolpaths",
}

local function toHex(n)
  if n == nil then return "" end
  return string.format("#%06X", math.floor(n) % 0x1000000)
end
local function fromHex(text)
  text = text:gsub("%s", "")
  if text == "" then return nil end
  local digits = text:match("^#?(%x%x%x%x%x%x)$")
  if not digits then return false end
  return tonumber(digits, 16)
end
local function shown(name)
  local v = cv.doc.variable(name)
  return v == nil or v ~= 0
end

local form = cv.ui.form({
  title = "Job Setup Sheet Editor",
  fields = {
    {name = "header", label = "Header colour (#RRGGBB, empty = none)", kind = "text",
     default = toHex(cv.doc.variable("jobsheet_header_rgb"))},
    {name = "footer", label = "Footer colour (#RRGGBB, empty = none)", kind = "text",
     default = toHex(cv.doc.variable("jobsheet_footer_rgb"))},
    {name = "layout", label = "Include Job Layout", kind = "bool", default = shown("jobsheet_show_layout")},
    {name = "notes", label = "Include Job Notes", kind = "bool", default = shown("jobsheet_show_notes")},
    {name = "material", label = "Include Material Setup", kind = "bool", default = shown("jobsheet_show_material")},
    {name = "summary", label = "Include Toolpaths Summary", kind = "bool", default = shown("jobsheet_show_summary")},
    {name = "toolpaths", label = "Include Individual Toolpath Summaries", kind = "bool",
     default = shown("jobsheet_show_toolpaths")},
    {name = "restore", label = "Restore default (ignores the choices above)", kind = "bool", default = false},
  },
})
if form == nil then return end

-- the variables a run needs to change; unchanged ones are not written
local function set(name, value)
  if cv.doc.variable(name) ~= value then cv.doc.set_variable(name, value) end
end

if form.restore then
  -- "default" is "no variable": a variable cannot be removed from a script, so each takes its
  -- default value (colours 0 = none, sections 1 = shown).
  set("jobsheet_header_rgb", 0)
  set("jobsheet_footer_rgb", 0)
  for _, n in ipairs({"jobsheet_show_layout", "jobsheet_show_notes", "jobsheet_show_material",
                      "jobsheet_show_summary", "jobsheet_show_toolpaths"}) do
    set(n, 1)
  end
  cv.log("Job sheet: defaults restored")
  return
end

local header, footer = fromHex(form.header), fromHex(form.footer)
if header == false or footer == false then
  cv.ui.message("A colour is six hex digits such as #1A3C5E.")
  return
end
-- 0 means "no colour"; black is written as 0x000001 (one blue step off, invisible)
set("jobsheet_header_rgb", header == nil and 0 or (header == 0 and 1 or header))
set("jobsheet_footer_rgb", footer == nil and 0 or (footer == 0 and 1 or footer))
set("jobsheet_show_layout", form.layout and 1 or 0)
set("jobsheet_show_notes", form.notes and 1 or 0)
set("jobsheet_show_material", form.material and 1 or 0)
set("jobsheet_show_summary", form.summary and 1 or 0)
set("jobsheet_show_toolpaths", form.toolpaths and 1 or 0)
cv.log("Job sheet settings stored")
