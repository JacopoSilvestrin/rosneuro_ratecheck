cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_ratecheck)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 dynamic_reconfigure
			 topic_tools)
find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

generate_dynamic_reconfigure_options(
	
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	rosneuro_data
	rosneuro_msgs
	topic_tools
  DEPENDS
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} 
		    include
		    
)

add_definitions()


add_library(${PROJECT_NAME}
	src/RateCheck.cpp
	src/DelayCheck.cpp
)

target_link_libraries(${PROJECT_NAME} 
		     ${catkin_LIBRARIES} 
)

add_dependencies(${PROJECT_NAME} 
	         ${catkin_EXPORTED_TARGETS}
)


#################
## Executables ##
#################

add_executable(test_rate src/test_rate.cpp)
target_link_libraries(test_rate rosneuro_ratecheck)
add_dependencies(test_rate ${PROJECT_NAME}_gencfg)

add_executable(test_delay src/test_delay.cpp)
target_link_libraries(test_delay rosneuro_ratecheck)
add_dependencies(test_delay ${PROJECT_NAME}_gencfg)



