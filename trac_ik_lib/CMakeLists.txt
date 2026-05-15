cmake_minimum_required(VERSION 3.8)
project(trac_ik_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
if(NOT TARGET NLopt::nlopt_cxx)
  find_package(nlopt_cxx REQUIRED)
endif()

# Force legacy FindBoost so ${Boost_LIBRARIES} resolves to raw .so paths
# rather than the Boost::date_time imported target.  Imported targets in
# INTERFACE_LINK_LIBRARIES cause ament's export file to be processed before
# find_dependency(Boost) runs, breaking consumers (CMake 3.28+, Boost 1.83+).
set(Boost_NO_BOOST_CMAKE TRUE)
find_package(Boost REQUIRED COMPONENTS date_time)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_library(trac_ik
  src/kdl_tl.cpp
  src/nlopt_ik.cpp
  src/trac_ik.cpp
)

# Required so the static library can be linked into the SWIG shared module
set_target_properties(trac_ik PROPERTIES POSITION_INDEPENDENT_CODE ON)

ament_target_dependencies(trac_ik
  kdl_parser
  urdf
)

target_link_libraries(trac_ik
  NLopt::nlopt_cxx
  ${Boost_LIBRARIES}
)

target_include_directories(trac_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS trac_ik
  EXPORT export_trac_ik
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_trac_ik HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(kdl_parser urdf NLopt)

ament_package()
