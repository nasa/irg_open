cmake_minimum_required(VERSION 3.5)
project(irg_spice)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)

###########
## Fetch ##
###########

# TODO: Should be a real variable!
set(BUILD_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/../../../build/irg_spice") 


# Warning: The MD5 will break whenever they update the tarball!
set(PACKAGE_URL http://naif.jpl.nasa.gov/pub/naif/toolkit/C/PC_Linux_GCC_64bit/packages/cspice.tar.Z)
set(PACKAGE_MD5 b776166eea01b562466f978754c9d3c7)
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


#############
## Install ##
#############

install(FILES ${incFiles}  DESTINATION include/${PROJECT_NAME})
install(FILES ${dataFiles} DESTINATION share)
install(FILES ${libFiles}  DESTINATION lib)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(lib/cspice.a lib/csupport.a)


ament_package()

