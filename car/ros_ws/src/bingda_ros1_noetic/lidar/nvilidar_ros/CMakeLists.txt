cmake_minimum_required(VERSION 2.8.3)
project(nvilidar_ros)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#add_definitions(-std=c++11) # Use C++11

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
)

#add_subdirectory(sdk)

set(SDK_PATH "./sdk/")

FILE(GLOB SDK_SRC 
  "${SDK_PATH}/src/impl/*.h"
  "${SDK_PATH}/src/impl/include/*.h"
  "${SDK_PATH}/src/impl/include/serial/*.h"
  "${SDK_PATH}/src/nvilidar/*.h"
  "${SDK_PATH}/src/impl/src/serial/*.cpp"
  "${SDK_PATH}/src/impl/src/socket/*.cpp"
  "${SDK_PATH}/src/nvilidar/*.cpp"
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/sdk/src/nvilidar
  ${PROJECT_SOURCE_DIR}/sdk/src/impl/include
)

add_executable(nvilidar_node src/nvilidar_node.cpp  ${SDK_SRC})
add_executable(nvilidar_client src/nvilidar_client.cpp)

target_link_libraries(nvilidar_node
   ${catkin_LIBRARIES} 
 )
target_link_libraries(nvilidar_client
   ${catkin_LIBRARIES} 
 )

install(TARGETS nvilidar_node nvilidar_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch startup sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


