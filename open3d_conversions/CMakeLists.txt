cmake_minimum_required(VERSION 3.5.0)
project(open3d_conversions)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

# find all open3D stuff
message("Searching open3D in /usr/local/lib/cmake/")
find_package(Open3D HINTS /usr/local/lib/cmake/)
list(APPEND Open3D_LIBRARIES dl)

find_package(OpenMP)
if (OpenMP_CXX_FOUND)
    message("Found OpenMP ${OpenMP_CXX_FOUND}  ${OpenMP_VERSION} ${OpenMP_CXX_VERSION_MAJOR} ${Open3D_VERSION} OpenMP::OpenMP_CXX")
    get_cmake_property(_variableNames VARIABLES)
    list (SORT _variableNames)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
endif ()

# Open3D
if (Open3D_FOUND)
    message("Found Open3D ${Open3D_VERSION}")
    # link_directories must be before add_executable
    link_directories(${Open3D_LIBRARY_DIRS})
else ()
    message("Open3D not found")
endif ()


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES open3d_conversions
 CATKIN_DEPENDS roscpp sensor_msgs
 DEPENDS EIGEN3 Open3D
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
)

# C++ library
add_library(open3d_conversions src/open3d_conversions.cpp)
add_dependencies(open3d_conversions ${open3d_conversions_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open3d_conversions ${catkin_LIBRARIES} ${Open3D_LIBRARIES} OpenMP::OpenMP_CXX)

# Install
install(TARGETS open3d_conversions
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/open3d_conversions/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_open3d_conversions test/test_open3d_conversions.cpp)
  target_link_libraries(test_open3d_conversions open3d_conversions ${catkin_LIBRARIES} ${Open3D_LIBRARIES}  OpenMP::OpenMP_CXX)
endif()
