import zipfile, sys, json
src, out, field, raw = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
b = zipfile.ZipFile(src)
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    for n in b.namelist():
        d = b.read(n)
        if n.startswith("toolpaths/"):
            j = json.loads(d)
            if field.startswith("tool."): j["tool"][field[5:]] = "@@V@@"
            elif field == "dwell": j["moves"].insert(1, ["dwell",0,0,5,"@@V@@"])
            elif field == "feedov":
                for i,m in enumerate(j["moves"]):
                    if m[0] in ("feed","plunge") and len(m)==4: j["moves"][i]=m+[{"feed":"@@V@@"}]; break
            d = json.dumps(j).replace('"@@V@@"', raw).encode()
        z.writestr(n, d)
