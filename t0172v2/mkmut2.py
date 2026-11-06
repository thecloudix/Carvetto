#!/usr/bin/env python3
# Round-2 mutants of the branch (2665a1a): writes mutants/<X>/{offset.cpp,box_tree.hpp,box_tree_test.cpp} (+ .diff).
import os, sys, difflib
W='/home/chris/wt-T0172'; H='/home/chris/harness/t0172v2/mutants'
src={'offset.cpp':open(W+'/core/src/geometry/offset.cpp').read(),
     'box_tree.hpp':open(W+'/core/src/geometry/box_tree.hpp').read(),
     'box_tree_test.cpp':open(W+'/core/tests/geometry/box_tree_test.cpp').read()}
M={
 # BoxTree
 'BP':('box_tree.hpp','        for (const std::size_t id : pending_) {\n            ++work_;','        for (const std::size_t id : std::vector<std::size_t>{}) {\n            ++work_;'),  # pending list never scanned
 'BR':('box_tree.hpp','if (node.maxId < lo || node.minId > hi ||','if (node.maxId <= lo || node.minId > hi ||'),  # id-range off by one
 'BL':('box_tree.hpp','                        if (id >= lo && id <= hi && t.boxes[k].intersects(query) && f(id)) {','                        if (t.boxes[k].intersects(query) && f(id)) {'),  # no per-item id filter in leaves
 'BC':('box_tree.hpp','                    stack[top++] = node.left + 1;\n',''),  # right child never visited (CQ analog)
 'BM':('box_tree.hpp','trees_.back().ids.size() <= ids.size()','trees_.back().ids.size() < ids.size()'),  # merge only smaller (more trees)
 'BA':('box_tree.hpp','            const auto a = static_cast<std::size_t>((parentAxis + k) % 4);','            const auto a = static_cast<std::size_t>((0 + k - 1) % 4);'),  # always first differing axis (x min)
 'BN':('box_tree.hpp','static bool holdable(const Box2& b) { return b.min.x <= b.max.x && b.min.y <= b.max.y; }','static bool holdable(const Box2& b) { return !b.isEmpty(); }'),  # NaN boxes held
 'BU':('box_tree.hpp','                        if (id >= lo && id <= hi && t.boxes[k].intersects(query) && f(id)) {\n                            return true;','                        if (id >= lo && id <= hi && t.boxes[k].intersects(query) && f(id)) {\n                            (void)0;'),  # queryUntil does not stop in trees
 'BS':('box_tree.hpp','        const std::size_t mid = lo + (hi - lo) / 2;','        const std::size_t mid = lo + 1;'),  # unbalanced split (depth n): stack overflow / perf
 # selfCrossingCuts / meetReach
 'XI':('offset.cpp','        boxes.push_back(b.inflated(meetReach(s, b)));','        boxes.push_back(b);'),  # no growth
 'X7':('offset.cpp','        reach += 1e-9 + 1e-7 * arc->radius;','        reach += 1e-9;'),  # no 1e-7 r for arcs
 'X9':('offset.cpp','        reach += 1e-9 + 1e-7 * arc->radius;','        reach += 1e-7 * arc->radius;'),  # no 1e-9 for arcs
 'XL':('offset.cpp','        reach += 1e-7 * len;\n','        reach += 0.0 * len;\n'),  # no line growth
 'XN':('offset.cpp','tree.queryUntil(boxes[i], from, i - 2,','tree.queryUntil(boxes[i], from, i - 1,'),  # neighbour included
 'XF':('offset.cpp','            from = i;\n            i = from + 1;  // the loop goes on at from + 2','            i = from + 1;  // the loop goes on at from + 2'),  # part start not moved
 'XC':('offset.cpp','        dropTinyEnds(run, tol);\n        if (!run.empty()) {\n            runs.push_back(std::move(run));','        if (!run.empty()) {\n            runs.push_back(std::move(run));'),
 'SL':('offset.cpp','    if (partCrossesItself(strand)) {\n        // Not a plain needle','    if (strand.size() > 512 || partCrossesItself(strand)) {\n        // Not a plain needle'),
 'S5':('offset.cpp','                for (std::vector<Span>& run : nonCrossingRuns(std::move(part), tol)) {\n                    add(st.key, std::move(run));\n                }',
        '                if (part.size() > 512) { for (Span& sp : part) { if (!isDegenerate(sp, tol)) { add(st.key, std::vector<Span>{sp}); } } } else\n                for (std::vector<Span>& run : nonCrossingRuns(std::move(part), tol)) {\n                    add(st.key, std::move(run));\n                }'),
 # putBackStrays window / cover
 'WR':('offset.cpp','        window = Box2{};\n        for (const Span& sp : part) {','        for (const Span& sp : part) {'),  # window not reset on add
 'WB':('offset.cpp','            if (coverGrid.box(i).intersects(around)) {','            if (true) {'),  # window candidates not filtered (still exact distances; equivalent)
 'WA':('offset.cpp','            const double reachAhead = kWindow * tol.distance + 2.0 * cap;','            const double reachAhead = 0.0;'),
 'WN':('offset.cpp','            coverGrid.add(bounds(sp));','            coverGrid.add(Box2{});'),  # added strays never found
 'OM':('offset.cpp','                     [](const Stray& a, const Stray& b) { return a.key < b.key; });\n    std::vector<Contour> merged;','                     [](const Stray& a, const Stray& b) { return a.key > b.key; });\n    std::vector<Contour> merged;'),
 'M3':('offset.cpp','            const bool isolated = from == k && to == e;','            const bool isolated = true;'),
 'W2':('offset.cpp','            const std::size_t from = k > 0 ? k - 1 : k;\n            const std::size_t to = e + 1 < samples.size() ? e + 1 : e;','            const std::size_t from = k;\n            const std::size_t to = e;'),
 'S':('offset.cpp','    const std::vector<std::size_t> cuts = selfCrossingCuts(part);\n','    std::vector<std::size_t> cuts = selfCrossingCuts(part);\n    if (part.size() > 512 && cuts.empty()) { for (std::size_t q = 1; q < part.size(); ++q) cuts.push_back(q); }\n'),
 'S4':('offset.cpp','    const std::vector<std::size_t> cuts = selfCrossingCuts(part);\n','    std::vector<std::size_t> cuts = selfCrossingCuts(part);\n    if (part.size() > 4 && cuts.empty()) { for (std::size_t q = 1; q < part.size(); ++q) cuts.push_back(q); }\n'),
 'P':('offset.cpp','    std::vector<Span> strand = total(other) > total(one) ? std::move(other) : std::move(one);','    std::vector<Span> strand = total(other) > total(one) ? std::move(one) : std::move(other);'),
 'O':('offset.cpp','        while (r < result.size() && !(a.key < resultKeys[r])) {','        while (r < result.size()) {'),
 'E2':('offset.cpp','                            if (exactOver(p)) {','                            if (true || exactOver(p)) {'),
 'EJ':('offset.cpp','    if (ahead(arc) && geometry::distance(arc.startPoint(), j) <= 1e-12 * scale) {','    if (ahead(arc)) {'),
}
sel=sys.argv[1:] or list(M)
for k in sel:
    f,a,b=M[k]
    c=src[f].count(a)
    if c!=1: print('MUTANT',k,'pattern count',c); continue
    d=H+'/'+k; os.makedirs(d,exist_ok=True)
    for name,txt in src.items():
        out=txt.replace(a,b) if name==f else txt
        open(d+'/'+name,'w').write(out)
    open(H+'/'+k+'.diff','w').write(''.join(difflib.unified_diff(src[f].splitlines(1),src[f].replace(a,b).splitlines(1),f,f)))
    print('wrote',k)
