cmake_minimum_required(VERSION 3.8)
project(trac_ik_python)

add_compile_options(-Wno-deprecated)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(trac_ik_lib REQUIRED)

find_package(SWIG REQUIRED)
find_package(Python3 COMPONENTS Development Interpreter REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

include(${SWIG_USE_FILE})

include_directories(
  ${trac_ik_lib_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

set(CMAKE_SWIG_FLAGS "-c++")

SET_SOURCE_FILES_PROPERTIES(
  swig/trac_ik_wrap.i PROPERTIES CPLUSPLUS ON
)

SWIG_ADD_LIBRARY(trac_ik_wrap
  LANGUAGE python
  SOURCES swig/trac_ik_wrap.i
)

SWIG_LINK_LIBRARIES(trac_ik_wrap
  ${Python3_LIBRARIES}
  trac_ik_lib::trac_ik
)

install(TARGETS _trac_ik_wrap
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/trac_ik_wrap.py"
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_package()
