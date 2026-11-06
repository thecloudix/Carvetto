#!/usr/bin/env python3
# timerpatch.py ORIG OUT: profile.cpp with phase timers in the T-0140 whole-scene block (stderr "T0140TIME ...").
import sys
s = open(sys.argv[1]).read()
def rep(a, b):
    global s
    assert s.count(a) == 1, (a[:50], s.count(a)); s = s.replace(a, b)
rep("        const double reach = rt + 2e-3;\n        Boundaries all;\n        {\n", "        const double reach = rt + 2e-3;\n        Boundaries all;\n        auto T0 = std::chrono::steady_clock::now(); double tB=0, tU=0, tI=0, tT=0, tA=0; std::size_t nHoles=0, nPieces=0;\n        {\n")
rep("            all.c = geometry::unionContours(bands, offTol_, geometry::FillRule::NonZero);\n            for (const Contour& b : all.c) {", "            auto T1 = std::chrono::steady_clock::now(); tB = std::chrono::duration<double>(T1 - T0).count();\n            all.c = geometry::unionContours(bands, offTol_, geometry::FillRule::NonZero);\n            tU = std::chrono::duration<double>(std::chrono::steady_clock::now() - T1).count();\n            for (const Contour& b : all.c) {")
rep("            // The piece: the hole less its islands (bands with their own holes inside it).\n", "            auto Ta = std::chrono::steady_clock::now(); ++nHoles;\n")
rep("            if (pieceArea < 0.04) continue;\n            const std::vector<Contour> thick", "            auto Tb = std::chrono::steady_clock::now(); tI += std::chrono::duration<double>(Tb - Ta).count();\n            if (pieceArea < 0.04) continue;\n            const std::vector<Contour> thick")
rep("            if (thick.empty()) continue;\n            // Attributed", "            auto Tc = std::chrono::steady_clock::now(); tT += std::chrono::duration<double>(Tc - Tb).count();\n            if (thick.empty()) continue;\n            ++nPieces;\n            // Attributed")
rep("            Piece piece;\n            piece.bounds = all.box[self];", "            tA += std::chrono::duration<double>(std::chrono::steady_clock::now() - Tc).count();\n            Piece piece;\n            piece.bounds = all.box[self];")
rep("            found.push_back(piece);\n        }\n    }\n", "            found.push_back(piece);\n        }\n        std::fprintf(stderr, \"T0140TIME total %.3f bands %.3f union %.3f islands %.3f thick %.3f attrib %.3f contours %zu holes %zu pieces %zu\\n\", std::chrono::duration<double>(std::chrono::steady_clock::now() - T0).count(), tB, tU, tI, tT, tA, all.c.size(), nHoles, nPieces);\n    }\n")
s = s.replace('#include <numbers>\n', '#include <numbers>\n#include <chrono>\n#include <cstdio>\n', 1)
open(sys.argv[2], 'w').write(s)
