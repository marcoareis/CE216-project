cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system iostreams filesystem)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
      pkg_check_modules(GAZEBO gazebo)
  endif()

  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

  find_package(Protobuf REQUIRED)
  
  include_directories(${GAZEBO_INCLUDE_DIRS})
  link_directories(${GAZEBO_LIBRARY_DIRS})

  add_library(model_push SHARED model_push.cc)
  add_library(contact SHARED contactPlugin.cc)
  add_executable(listener listener.cc)
  add_executable(plotPos plotPos.cc)
  target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)
  target_link_libraries(listener ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)
  target_link_libraries(contact ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
  target_link_libraries(plotPos ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)
