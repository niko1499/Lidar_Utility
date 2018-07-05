add_library(my_nodelet MyNodeletClass.cc )
target_link_libraries(my_nodelet
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})


add_executable(MyNodeletClass MyNodeletClass.cc)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(my_nodelet ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(my_nodelet
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES}
    )
