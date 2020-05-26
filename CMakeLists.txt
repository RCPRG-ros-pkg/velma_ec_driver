###############################################################################
# CMakeLists.txt
#
# Defines project structure for velma_ec_driver
###############################################################################

cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)
project(velma_ec_driver LANGUAGES CXX)

###############################################################################
# Project dependencies
###############################################################################

find_package(catkin REQUIRED COMPONENTS
    shm_comm
    cifx_driver
)


catkin_package(
    CATKIN_DEPENDS
        shm_comm
        cifx_driver
)

###############################################################################
# Main executable definition
###############################################################################

add_executable(velma_ec_driver
    src/velma_ec_driver.cpp
)

target_include_directories(velma_ec_driver
    PRIVATE
        ${catkin_INCLUDE_DIRS}
)

target_link_libraries(velma_ec_driver
    PRIVATE
        ${catkin_LIBRARIES}
        pthread
)

set_target_properties(velma_ec_driver
    PROPERTIES
        CXX_STANDARD 14
        CXX_EXTENSIONS OFF
        CXX_STANDARD_REQUIRED ON
)

target_compile_options(velma_ec_driver
    PRIVATE
        -Wall -Wextra -pedantic
)

###############################################################################
# Package configuration
###############################################################################

catkin_package(
    CATKIN_DEPENDS
        shm_comm
        cifx_driver
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(TARGETS velma_ec_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

