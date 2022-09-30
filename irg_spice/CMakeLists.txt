cmake_minimum_required(VERSION 3.0.2)
project(irg_spice)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

###########
## Fetch ##
###########

set(BUILD_FOLDER "${CMAKE_BINARY_DIR}")

# Warning: The MD5 will break whenever they update the tarball!
set(PACKAGE_URL http://naif.jpl.nasa.gov/pub/naif/toolkit/C/PC_Linux_GCC_64bit/packages/cspice.tar.Z)
set(PACKAGE_MD5 d7d737e6d70218d333e12795e9a15f7c)
set(DOWNLOAD_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/tarball)
set(DOWNLOAD_PATH "${DOWNLOAD_DIR}/cspice.tar.Z")
message("Downloading file: ${DOWNLOAD_PATH}")

# If we don't already have the correct download file, re-download it.
if(EXISTS ${DOWNLOAD_PATH})
  file(MD5 ${DOWNLOAD_PATH} existingMd5)
else()
  set(existingMd5 "NA")
endif()

if(NOT ${existingMd5} STREQUAL ${PACKAGE_MD5})
  file(MAKE_DIRECTORY ${DOWNLOAD_DIR})
  message("Downloading file: ${PACKAGE_URL}")
  file(DOWNLOAD ${PACKAGE_URL} ${DOWNLOAD_PATH} SHOW_PROGRESS EXPECTED_MD5=${PACKAGE_MD5})
else()
  message("Already have the cspice tarball.")
endif()

# Copy the downloaded tarball to the build folder so we don't pollute the /src folder
set(BUILD_TARBALL "${BUILD_FOLDER}/cspice.tar.Z")
file(COPY ${DOWNLOAD_PATH} DESTINATION ${BUILD_FOLDER})

# Unpack the tarball
execute_process(COMMAND tar -xf ${BUILD_TARBALL} WORKING_DIRECTORY ${BUILD_FOLDER})
set(UNPACK_FOLDER "${BUILD_FOLDER}/cspice")

message("Finished unpacking the file.")

###########
## Build ##
###########

# Rebuild everything (not strictly necessary but should be easy)
execute_process(COMMAND ./makeall.csh WORKING_DIRECTORY ${UNPACK_FOLDER})

# Get the list of built files
file(GLOB libFiles  "${UNPACK_FOLDER}/lib/*.a"    )
file(GLOB incFiles  "${UNPACK_FOLDER}/include/*.h")
file(GLOB dataFiles "${UNPACK_FOLDER}/data/*"     )

# Copy the files to the devel folder
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/cspice)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/data)
file(COPY ${libFiles}  DESTINATION ${CATKIN_DEVEL_PREFIX}/lib)
file(COPY ${incFiles}  DESTINATION ${CATKIN_DEVEL_PREFIX}/include/cspice)
file(COPY ${dataFiles} DESTINATION ${CATKIN_DEVEL_PREFIX}/data)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/cspice.a ${CATKIN_DEVEL_PREFIX}/lib/csupport.a
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############
## Install ##
#############

install(FILES ${libFiles}  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES ${incFiles}  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES ${dataFiles} DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION})
