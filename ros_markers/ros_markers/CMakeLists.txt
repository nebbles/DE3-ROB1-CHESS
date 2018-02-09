cmake_minimum_required(VERSION 2.8.3)
project(ros_markers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        tf 
        cv_bridge 
        image_transport 
        image_geometry 
        nodelet
        chilitags_catkin)

## System dependencies
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_markers
  CATKIN_DEPENDS 
    tf
    nodelet
  DEPENDS OpenCV
  LIBRARIES 
)

##########################################
##           Options                   ###
##########################################

option (WITH_KNOWLEDGE "Add support for exporting to a KB-API knowledge base" OFF)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)

set(LIBS 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

if (WITH_KNOWLEDGE)

    add_definitions(-DWITH_KNOWLEDGE)
    pkg_check_modules(LIBKB REQUIRED liboro)
    include_directories(${LIBKB_INCLUDE_DIRS})
    set (LIBS ${LIBS} ${LIBKB_LDFLAGS})

endif()


add_executable(detect src/chilitagsdetector.cpp src/detect.cpp)
target_link_libraries(detect ${LIBS})

add_library(markers_nodelet src/chilitagsdetector.cpp src/nodelet.cpp)
add_dependencies(markers_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(markers_nodelet ${LIBS})


#############
## Install ##
#############

install(TARGETS detect markers_nodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    launch/detect.launch
    launch/detect_gscam.launch
    launch/gscam_nodelet_pipeline.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
    config/markers_configuration_sample.yml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES
    nodelet_markers.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

