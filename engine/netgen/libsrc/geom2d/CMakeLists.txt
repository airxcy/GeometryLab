target_sources(nglib PRIVATE
    csg2d.cpp
    genmesh2d.cpp
    geometry2d.cpp
    python_geom2d.cpp
)

install(FILES
  geometry2d.hpp spline2d.hpp
  vsgeom2d.hpp csg2d.hpp
  DESTINATION ${NG_INSTALL_DIR_INCLUDE}/geom2d
)
