cmake_minimum_required(VERSION 2.8)

# Eliminate a warning when building in Windows that relates
# to static linking of Qt executables to qtmain.lib.
# This policy was introduced in CMake version 2.8.11.
# CMake version 2.8.11.2 warns when the policy is not set
# and uses OLD behavior.
if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()


#requires PCL
find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} include)



        #As we are going to re-use several components of the main qCC project
        #we define its sources path relatively to the plugin's dir
        set(CloudCompare_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../qCC)

        #include the whole CC directory (easier)
        include_directories( ${CloudCompare_SOURCE_DIR} )

option( INSTALL_QDATAFRAMEWORK_PLUGIN "Check to install QDATAFRAMEWORK plugin" OFF )

if (INSTALL_QDATAFRAMEWORK_PLUGIN)

#CloudCompare 'DUMMY' plugin
project( QDATAFRAMEWORK_PLUGIN )

#load necessary libraries (see qPCV for an example)
#add_subdirectory (LIB1)

#if the plugin is an 'OpenGL filter', uncomment the line below
#set( CC_OPENGL_FILTER ON BOOL)
include( ../CMakePluginTpl.cmake )

#set dependencies to necessary libraries (see qPCV for an example)
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries( ${PROJECT_NAME} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

endif()
