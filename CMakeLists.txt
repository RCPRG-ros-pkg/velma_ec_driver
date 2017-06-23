cmake_minimum_required(VERSION 2.8)
project(velma_ec_driver)

find_package(catkin REQUIRED COMPONENTS
  shm_comm
  cifx_driver
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS shm_comm cifx_driver
)

add_executable(velma_ec_driver src/velma_ec_driver.cpp)

target_link_libraries(velma_ec_driver ${catkin_LIBRARIES})


