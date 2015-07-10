# %Tag(FULLTEXT)%

cmake_minimum_required(VERSION 2.8.3)
project(aubionode)

## Find catkin and any catkin packages

#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp rospy
    std_msgs
    genmsg
    cmake_modules
    dynamic_reconfigure
)
find_package(Eigen REQUIRED)

# Generate the reconfigure options
generate_dynamic_reconfigure_options(cfg/audioConstants.cfg)

#find_package(SFML 2 REQUIRED system audio)
#if(SFML_FOUND)
 #   include_directories(${SFML_INCLUDE_DIR})
  #  target_link_libraries(${EXECUTABLE_NAME} ${SFML_LIBRARIES})
#endif()

## Declare ROS messages, services and actions
add_message_files(DIRECTORY msg FILES Commands.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)


# export the dependencies of this package for who ever depends on us
catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS
    geometry_msgs
    roscpp rospy
    std_msgs
    dynamic_reconfigure
    DEPENDS eigen
)

##Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Build talker and listener
include_directories( ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS})

#building the aubio lib with a script
add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/aubionode/src/aubio-lib/build/src/libaubio.so
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/aubionode/src/aubio-lib
                COMMAND ./build-aubiolib.sh
                )

add_custom_target(aubio-lib
                  DEPENDS ${CMAKE_SOURCE_DIR}/aubionode/src/aubio-lib/build/src/libaubio.so
                )

#building the RtAudio lib with a script
add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/aubionode/src/rtaudio/librtaudio.so
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/aubionode/src/rtaudio
                COMMAND ./build-rtaudio.sh
                )

add_custom_target(rtaudio
                  DEPENDS ${CMAKE_SOURCE_DIR}/aubionode/src/rtaudio/librtaudio.so
                )

add_executable(aubionode src/main.cpp)

add_dependencies(aubionode std_msgs geometry_msgs aubio-lib rtaudio)

target_link_libraries(aubionode ${catkin_LIBRARIES}) #link to ROS libs
target_link_libraries(aubionode ${CMAKE_SOURCE_DIR}/aubionode/src/aubio-lib/build/src/libaubio.so) #link to aubio-lib
target_link_libraries(aubionode ${CMAKE_SOURCE_DIR}/aubionode/src/rtaudio/librtaudio.so) #link to rtaudio

#add_executable(state_estimator_SICK src/state_estimator_SICK.cpp)
#target_link_libraries(state_estimator_SICK ${catkin_LIBRARIES})
#add_dependencies(state_estimator_SICK sensors_gencpp std_msgs geometry_msgs)

#add_executable(desired_state src/desired_state.cpp)
#target_link_libraries(desired_state ${catkin_LIBRARIES})
## {state...} added to resolve issue with state.msg --> state.h in devel not being found
#add_dependencies(desired_state  sensor_gencpp  geometry_msgs std_msgs ${state_EXPORTED_TARGETS})

#%EndTag(FULLTEXT)%
