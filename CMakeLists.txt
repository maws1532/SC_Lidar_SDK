cmake_minimum_required(VERSION 2.8.3)
project(delta_2b_lidar)

set(DELTA_2b_LIDAR_SDK_PATH "./src/")


#SET(CMAKE_SYSTEM_NAME Linux)
#SET(CMAKE_C_COMPILER /home/yuy/Downloads/Shark/qfeeltools/prebuilts/gcc/linux-x86/aarch64/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
#SET(CMAKE_CXX_COMPILER /home/yuy/Downloads/Shark/qfeeltools/prebuilts/gcc/linux-x86/aarch64/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
SET(CMAKE_CXX_FLAGS "-std=c++11")

FILE(GLOB DELTA_2b_LIDAR_SDK_SRC 
  "${DELTA_2b_LIDAR_SDK_PATH}/src/*.cpp"
)

include_directories(
  ${DELTA_2b_LIDAR_SDK_PATH}/include
  ${DELTA_2b_LIDAR_SDK_PATH}/src
)

add_executable(delta_2b_lidar_node app/node.cpp ${DELTA_2b_LIDAR_SDK_SRC})
target_link_libraries(delta_2b_lidar_node -lrt -lpthread)


