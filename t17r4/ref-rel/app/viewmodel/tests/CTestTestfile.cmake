# CMake generated Testfile for 
# Source directory: /home/chris/wt-T0017/build-verify2/h/ref-src/app/viewmodel/tests
# Build directory: /home/chris/wt-T0017/build-verify2/h/ref-rel/app/viewmodel/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
include("/home/chris/wt-T0017/build-verify2/h/ref-rel/app/viewmodel/tests/carvetto_viewmodel_tests-b12d07c_include.cmake")
add_test([=[viewmodel.qt_free]=] "/usr/bin/cmake" "-DSOURCE_ROOT=/home/chris/wt-T0017/build-verify2/h/ref-src" "-DLINK_FILE=/home/chris/wt-T0017/build-verify2/h/ref-rel/app/viewmodel/tests/viewmodel_link_libraries.txt" "-P" "/home/chris/wt-T0017/build-verify2/h/ref-src/app/viewmodel/tests/qt_free_check.cmake")
set_tests_properties([=[viewmodel.qt_free]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/chris/wt-T0017/build-verify2/h/ref-src/app/viewmodel/tests/CMakeLists.txt;33;add_test;/home/chris/wt-T0017/build-verify2/h/ref-src/app/viewmodel/tests/CMakeLists.txt;0;")
