cmake_minimum_required(VERSION 2.4.6)
project(ipa_lawnmower_tracker)

set(catkin_RUN_PACKAGES 
        roscpp
        sensor_msgs
        tf
        image_transport
        cv_bridge
        visualization_msgs
        cob_object_detection_msgs
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
	cmake_modules
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Creates a bunch of environment variables that may be used later in the script
## e.g. catkin_INCLUDE_DIRS

find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(OpenCV REQUIRED)
#find_package(TinyXML REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread)
#find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS
	ros/include
LIBRARIES
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
DEPENDS
	OpenCV
#	TinyXML
	Boost
#	PCL
	Eigen
)


###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
	ros/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
#	${TinyXML_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
#	${PCL_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(lawnmower_tracker
	ros/src/lawnmower_tracker_node.cpp
	ros/src/lawnmower_tracker.cpp
)
add_dependencies(lawnmower_tracker ${catkin_EXPORTED_TARGETS})
target_link_libraries(lawnmower_tracker
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
#	${TinyXML_LIBRARIES}
	${Boost_LIBRARIES}
#	${PCL_LIBRARIES}
	${Eigen_LIBRARIES}
)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS lawnmower_tracker
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY common/files
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
	PATTERN ".svn" EXCLUDE
)

