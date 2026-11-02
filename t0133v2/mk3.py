import zipfile, sys, json
src, out, name, tool = sys.argv[1:5]
b = zipfile.ZipFile(src)
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    for n in b.namelist():
        d = b.read(n)
        if n.startswith("toolpaths/"):
            j = json.loads(d); j["name"] = name; j["tool"]["name"] = tool; d = json.dumps(j, ensure_ascii=False).encode()
        z.writestr(n, d)
