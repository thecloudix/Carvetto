#!/usr/bin/env python3
# T-0172 verifier: mutants of the branch offset.cpp -> mutants/offset_<X>.cpp (each replacement must hit exactly once)
import sys, pathlib
src = pathlib.Path('/home/chris/wt-T0172/core/src/geometry/offset.cpp').read_text()
M = {
 'M3': [('const bool isolated = from == k && to == e;', 'const bool isolated = true;')],
 'W2': [('const std::size_t from = k > 0 ? k - 1 : k;\n            const std::size_t to = e + 1 < samples.size() ? e + 1 : e;',
         'const std::size_t from = k;\n            const std::size_t to = e;')],
 # S: the T-0128 fallback restored (> 512 spans: one chain per span, sub-tol spans dropped), both in step 7 and in the loop strand
 'S': [('const std::vector<std::size_t> cuts = selfCrossingCuts(part);',
        'std::vector<std::size_t> cuts = selfCrossingCuts(part);\n                if (part.size() > 512 && cuts.empty()) { for (std::size_t q = 1; q < part.size(); ++q) cuts.push_back(q); }'),
       ('    if (partCrossesItself(strand)) {', '    if (strand.size() > 512 || partCrossesItself(strand)) {')],
 'S4': [('const std::vector<std::size_t> cuts = selfCrossingCuts(part);',
        'std::vector<std::size_t> cuts = selfCrossingCuts(part);\n                if (part.size() > 4 && cuts.empty()) { for (std::size_t q = 1; q < part.size(); ++q) cuts.push_back(q); }')],
 'SL': [('    if (partCrossesItself(strand)) {', '    if (strand.size() > 4 || partCrossesItself(strand)) {')],
 'P': [('std::vector<Span> strand = total(other) > total(one) ? std::move(other) : std::move(one);',
        'std::vector<Span> strand = total(other) > total(one) ? std::move(one) : std::move(other);')],
 'O': [('while (r < result.size() && !(a.key < resultKeys[r])) {', 'while (r < result.size()) {')],
 'O2': [('[](const Stray& a, const Stray& b) { return a.key < b.key; });', '[](const Stray& a, const Stray& b) { return a.key > b.key; });')],
 'E2': [('if (exactOver(p)) {', 'if (true || exactOver(p)) {')],
 'EJ': [('if (ahead(arc) && geometry::distance(arc.startPoint(), j) <= 1e-12 * scale) {', 'if (ahead(arc)) {')],
 # new-code mutants (informative)
 'XI': [('boxes.push_back(b.inflated(4.0 * kParamEps * length(s) + 1e-12 * scale));', 'boxes.push_back(b); (void)scale;')],
 'XF': [('            from = i;\n        }\n        grid.add(boxes[i]);', '        }\n        grid.add(boxes[i]);')],
 'XC': [('                    const auto begin = part.begin();', '                    const auto begin = part.begin(); if (c == 0) { first = 0; }'),
        ('                        dropTinyEnds(run, tol);\n                        if (!run.empty()) {', '                        if (!run.empty()) {')],
 'WR': [('    auto add = [&](double key, std::vector<Span>&& part) {\n        window = Box2{};', '    auto add = [&](double key, std::vector<Span>&& part) {')],
 'WK': [('            if (coverGrid.box(i).intersects(around)) {', '            if (true) {')],
 'WA': [('const double reachAhead = kWindow * tol.distance + 2.0 * cap;', 'const double reachAhead = kWindow * tol.distance;')],
 'CG': [('if (child != kNoChild && !b.contains(cellBox(levels_[li], x, y))) {', 'if (child != kNoChild) {')],
 'CQ': [('                const std::size_t child = childOf(l, c);\n                if (child != kNoChild && visit(child, query, f)) {',
         '                const std::size_t child = childOf(l, c);\n                if (child != kNoChild && x == x0 && visit(child, query, f)) {')],
 'BS': [('        for (const std::size_t i : boundaryHits) {', '        std::reverse(boundaryHits.begin(), boundaryHits.end());\n        for (const std::size_t i : boundaryHits) {')],
}
names = sys.argv[1:] or list(M)
for n in names:
    s = src
    for a, b in M[n]:
        c = s.count(a)
        if c != 1: sys.exit(f'{n}: pattern found {c} times: {a[:60]}')
        s = s.replace(a, b)
    pathlib.Path(f'/home/chris/harness/t0172v1/mutants/offset_{n}.cpp').write_text(s)
    print(n, 'ok')
