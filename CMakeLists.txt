cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
# rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

# octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

# c++ 0x
#if("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang") 
#	set(CMAKE_COMPILER_IS_CLANGXX)
#endif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang")
if(CMAKE_COMPILER_IS_GNUCXX)
	execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
	if (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		add_definitions(-std=gnu++0x)
	else (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		message(SEND_ERROR "You need GCC version 4.4 or greater to compile this library")
	endif (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
else(CMAKE_COMPILER_IS_GNUCXX)
		add_definitions(-std=gnu++0x)
	#	message(SEND_ERROR "You need partial C++0x support for N2351, N2672, N1984. Currently this build toolchain supports only GCC >= 4.4. If your compiler supports these specifications, please send us a patch.")
endif(CMAKE_COMPILER_IS_GNUCXX)
# well, this breaks libpointmatcher on non-unix platform,
# please contribute the define for your favorite compiler
# It must support (compiler support in parenthesis):
# - N2351: shared pointers in standard
# - N2672: Initializer lists (gcc)
# - N1984: auto-typed variables (gcc, clang)


# look for yaml-cpp library
find_path(yaml-cpp_INCLUDE_DIRS yaml-cpp/yaml.h
	/usr/local/include
	/usr/include
)
find_library(yaml-cpp_LIBRARIES yaml-cpp PATHS
	/usr/local/lib
	/usr/lib
NO_DEFAULT_PATH)
if (yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)
	include_directories(${yaml-cpp_INCLUDE_DIRS})
	set(EXTRA_LIBS ${EXTRA_LIBS} ${yaml-cpp_LIBRARIES})
	add_definitions(-DHAVE_YAML_CPP)
endif (yaml-cpp_INCLUDE_DIRS AND yaml-cpp_LIBRARIES)

rosbuild_add_boost_directories()

rosbuild_add_executable(goal_interact_marker src/goalInteractMarker.cpp)

rosbuild_add_executable(ugv_nav_static src/ugv_nav_static.cpp
	src/TensorMap.cpp src/tensorUtils.cpp src/GenericSearch.cpp
	src/DStarLite.cpp src/LocalPlanner.cpp src/trp_params.cpp src/MyTimer.cpp)
# filesystem and system are needed from pointmatcher
rosbuild_link_boost(ugv_nav_static filesystem system)
# EXTRA_LIBS and nabo needed for pointmatcher
target_link_libraries(ugv_nav_static ${EXTRA_LIBS} pointmatcher nabo)

#rosbuild_add_executable(trp_batch src/trp_batch.cpp
#	src/TensorMap.cpp src/tensorUtils.cpp src/GenericSearch.cpp
#	src/DStarLite.cpp src/PathExecution.cpp src/trp_params.cpp src/MyTimer.cpp)
#rosbuild_link_boost(trp_batch filesystem system)
# EXTRA_LIBS and nabo needed for pointmatcher
#target_link_libraries(trp_batch ${EXTRA_LIBS} pointmatcher nabo)

#rosbuild_add_executable(trpb_viz src/trpb_viz.cpp src/tensorUtils.cpp)
# filesystem and system are needed from pointmatcher
#rosbuild_link_boost(trpb_viz filesystem system)
# EXTRA_LIBS and nabo needed for pointmatcher
#target_link_libraries(trpb_viz ${EXTRA_LIBS})# pointmatcher)

#rosbuild_add_executable(input_viz src/input_viz.cpp src/tensorUtils.cpp)
#target_link_libraries(input_viz ${EXTRA_LIBS})

rosbuild_add_executable(dist_filter_eval src/dist_filter.cpp
		src/tensorUtils.cpp)
target_link_libraries(dist_filter_eval ${EXTRA_LIBS})



#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
