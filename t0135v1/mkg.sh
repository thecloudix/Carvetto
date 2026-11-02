D=/home/chris/wt-T0135/build-verify/drv
for v in new base; do if [ $v = new ]; then W=/home/chris/wt-T0135; B=$W/build-verify/rel; o=$D/bin/gfit; else W=/home/chris/wt-T0135/build-verify/base-src; B=/home/chris/wt-T0135/build-verify/base-rel; o=$D/bin/gfit_base; fi
c++ -std=c++20 -O2 -w -I$W/core/include gfit.cpp -o $o $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1; done
