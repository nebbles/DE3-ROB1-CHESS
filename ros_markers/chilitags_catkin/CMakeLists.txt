cmake_minimum_required(VERSION 2.8.3)
project(chilitags_catkin)

find_package(catkin REQUIRED)

include(ExternalProject)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
ExternalProject_Add(chilitags_src
  GIT_REPOSITORY https://github.com/chili-epfl/chilitags.git
  CONFIGURE_COMMAND cmake -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX} -DCMAKE_BUILD_TYPE=Release ../chilitags_src 
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND make install
)

catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  LIBRARIES ${PROJECT_NAME}
  CFG_EXTRAS chilitags-extras.cmake
)

add_library(${PROJECT_NAME}
  src/lib_wrap.cc
)
add_dependencies(${PROJECT_NAME} chilitags_src)
target_link_libraries(${PROJECT_NAME}
  ${CATKIN_DEVEL_PREFIX}/lib/libchilitags${CMAKE_SHARED_LIBRARY_SUFFIX}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
