cmake_minimum_required(VERSION 3.1)
project(legged_state_estimator CXX)
set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

#############
## Options ##
#############
option(OPTIMIZE_FOR_NATIVE "Enable -march=native" ON)

##################################
## Build legged_state_estimator ##
##################################
if(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "WARNING: CMAKE_BUILD_TYPE is NOT set to Release, which can decrease performance significantly.")
endif()
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
# find Eigen
find_package(Eigen3 REQUIRED)
# find pinocchio
find_package(pinocchio REQUIRED)
# find inekf
find_package(inekf REQUIRED)
# build 
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES src/*.cpp)
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS include/${PROJECT_NAME}/*.h*)
add_library(
  ${PROJECT_NAME} 
  SHARED
  ${${PROJECT_NAME}_SOURCES} 
  ${${PROJECT_NAME}_HEADERS}
)
target_link_libraries(
  ${PROJECT_NAME} 
  PUBLIC
  ${PINOCCHIO_LIBRARIES}
  inekf::inekf
)
target_include_directories(
  ${PROJECT_NAME} 
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${inekf_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if (OPTIMIZE_FOR_NATIVE)
  target_compile_options(
    ${PROJECT_NAME} 
    PUBLIC
    -march=native
  )
endif()

##############
## Bindings ##
##############
add_subdirectory(bindings/python)

#############
## Install ##
#############
include(GNUInstallDirs)
# Install lib files
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-config
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)
# Install header files
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
# Install config file 
set(CONFIG_PACKAGE_LOCATION "share/${PROJECT_NAME}/cmake")
install(
  EXPORT ${PROJECT_NAME}-config
  FILE ${PROJECT_NAME}-config.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CONFIG_PACKAGE_LOCATION}
)
# Install alias
add_library(
  ${PROJECT_NAME}::${PROJECT_NAME} 
  ALIAS ${PROJECT_NAME}
)