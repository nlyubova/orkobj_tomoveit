cmake_minimum_required(VERSION 2.8.3)
SET(PROJECTNAME convertmsg_orktomoveit)

project(${PROJECTNAME})

find_package(Boost QUIET COMPONENTS program_options signals)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECTNAME} src/main.cpp)

 target_link_libraries(${PROJECTNAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
