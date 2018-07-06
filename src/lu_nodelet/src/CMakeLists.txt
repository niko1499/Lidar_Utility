add_library(nodelet_class MyNodeletClass.cc )
target_link_libraries(nodelet_class
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_executable(lu_node lu_node.cc)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(nodelet_class ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(
	lu_node

    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES}
    )
