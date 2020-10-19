cmake_minimum_required(VERSION 2.8.3)
project(mpl_test_node)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations -fopenmp -DGLIBCXX_USE_CXX11_ABI=ON")


execute_process(COMMAND lsb_release -cs
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
)

SET(COMPILE_ELLIPSOID_PLANNER_NODE TRUE)
#if(${LSB_RELEASE_ID_SHORT} MATCHES "bionic")
#  SET(COMPILE_ELLIPSOID_PLANNER_NODE FALSE)
#  message("-- Systemm version: ${LSB_RELEASE_ID_SHORT}, disable the compile of [ellipsoid_planner_node]")
#endif()
#message("-- COMPILE_ELLIPSOID_PLANNER_NODE: ${COMPILE_ELLIPSOID_PLANNER_NODE}")

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(motion_primitive_library REQUIRED)
find_package(decomp_util REQUIRED)
find_package(open3d_conversions REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${MOTION_PRIMITIVE_LIBRARY_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS})

# find all open3D stuff
message(STATUS "foo include dir: ${Open3D_INCLUDE_DIRS}")
find_package(Open3D HINTS /usr/local/lib/cmake/)
list(APPEND Open3D_LIBRARIES dl)
#
find_package(OpenMP)
if (OpenMP_CXX_FOUND)
    message("Found OpenMP ${OpenMP_CXX_FOUND}  ${OpenMP_VERSION} ${OpenMP_CXX_VERSION_MAJOR} ${Open3D_VERSION}")
    get_cmake_property(_variableNames VARIABLES)
    list (SORT _variableNames)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
endif ()
#
# Open3D
if (Open3D_FOUND)
    message("Found Open3D ${Open3D_VERSION}")
    # link_directories must be before add_executable
    link_directories(${Open3D_LIBRARY_DIRS})
else ()
    message("Open3D not found")
endif ()

catkin_simple()

cs_add_executable(map_planner_node src/map_planner_node.cpp)
target_link_libraries(map_planner_node ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})

cs_add_executable(map_replanner_node src/map_replanner_node.cpp)
target_link_libraries(map_replanner_node ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})

cs_add_executable(distance_map_planner_node src/distance_map_planner_node.cpp)
target_link_libraries(distance_map_planner_node ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})

if(COMPILE_ELLIPSOID_PLANNER_NODE)
cs_add_executable(ellipsoid_planner_node src/ellipsoid_planner_node.cpp
        src/tinyply.cpp)
target_link_libraries(ellipsoid_planner_node ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES} ${Open3D_LIBRARIES} OpenMP::OpenMP_CXX
        stdc++fs)
endif()

cs_add_executable(poly_map_planner_node src/poly_map_planner_node.cpp)
target_link_libraries(poly_map_planner_node ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})

cs_add_executable(poly_map_replanner_node src/poly_map_replanner_node.cpp)
target_link_libraries(poly_map_replanner_node ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})

cs_add_executable(traj_solver_node src/traj_solver_node.cpp)
target_link_libraries(traj_solver_node ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})

cs_add_executable(multi_robot_node src/multi_robot_node.cpp)
target_link_libraries(multi_robot_node ${MOTION_PRIMITIVE_LIBRARY_LIBRARIES})

cs_install()

cs_export()
