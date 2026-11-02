import zipfile, sys, json
src, outdir = sys.argv[1], sys.argv[2]
base = zipfile.ZipFile(src)
good = base.read("toolpaths/0002.json").decode()
names = [n for n in base.namelist()]
def make(name, member_bytes, member="toolpaths/0001.json"):
    with zipfile.ZipFile(f"{outdir}/{name}.carv", "w", zipfile.ZIP_DEFLATED) as z:
        for n in names:
            if n.startswith("toolpaths/"): continue
            z.writestr(n, base.read(n))
        z.writestr(member, member_bytes)
g = json.loads(good)
def var(**kw):
    d = dict(g); d.update(kw); return json.dumps(d)
make("deep_params", good.replace('"params":{', '"params":{"x":' + "["*100000 + "]"*100000 + ',', 1))
make("deep_params_1k", good.replace('"params":{', '"params":{"x":' + "["*1000 + "]"*1000 + ',', 1))
make("deep_moves", good.replace('"moves":[', '"moves":[' + "["*100000 + "]"*100000 + ',', 1))
make("huge_num", var(safe_z=1e308))
make("huge_num2", good.replace('"safe_z":5', '"safe_z":1e400'))
make("huge_coords", var(moves=[["rapid",0,0,5],["plunge",0,0,-1e308],["feed",1e308,-1e308,-1e308],["arc_cw",1,1,1,1e308,-1e308],["retract",0,0,1e308]]))
make("wrong_types", var(moves=[["rapid","0",0,5]]))
make("wrong_types2", var(tool="x"))
make("wrong_types3", var(moves={"a":1}))
make("toolnum", var(tool=dict(g["tool"], tool_number=-5)))
make("toolnum_float", var(tool=dict(g["tool"], tool_number=5.5)))
make("neg_feed", var(tool=dict(g["tool"], feed_rate=-100, diameter=-3)))
make("dwell_neg", var(moves=[["rapid",0,0,5],["dwell",0,0,5,-1e300]]))
make("bad_kind_num", var(moves=[[5,0,0,5]]))
make("empty_move", var(moves=[[]]))
make("feedovr_rapid", var(moves=[["rapid",0,0,5,{"feed":100}]]))
make("not_object", "[1,2,3]")
make("not_json", "{oops")
make("empty_moves", var(moves=[]))
make("arc_zero_radius", var(moves=[["rapid",0,0,5],["plunge",0,0,-1],["arc_cw",10,0,-1,0,0]]))
make("many_moves", var(moves=[["feed",i*0.001,0,-1] for i in range(300000)]))
make("long_string", var(name="A"*10_000_000))
make("ctrl_name", var(name="x)\nM3 S1\nG0 Z-50 (", tool=dict(g["tool"], name="T\nG0 Z-99")))
make("non_toolpath_member", good, member="toolpaths/1.json")
