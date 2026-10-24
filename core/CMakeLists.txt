add_library(carvetto_core STATIC
  src/geometry/vec2.cpp
  src/geometry/box2.cpp
  src/geometry/transform2.cpp
  src/geometry/span.cpp
  src/geometry/polyline.cpp
  src/geometry/contour.cpp
)
target_include_directories(carvetto_core PUBLIC include)
target_link_libraries(carvetto_core PRIVATE carvetto_warnings)
add_library(carvetto::core ALIAS carvetto_core)
