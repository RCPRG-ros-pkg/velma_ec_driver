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

###############################################################################
# Package configuration
###############################################################################

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
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

target_compile_options(velma_ec_driver
    PRIVATE
        -Wall -Wextra -pedantic
)

###############################################################################
# Installation configuration
###############################################################################

install(TARGETS velma_ec_driver
    RUNTIME
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
