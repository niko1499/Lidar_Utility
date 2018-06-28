add_executable(cloud_node_pandar cloud_node.cc convert.cc driver.cc)
add_dependencies(cloud_node_pandar ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cloud_node_pandar pandar_rawdata
					  pandar_input
					  pcap
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS cloud_node_pandar
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(cloud_nodelet_pandar cloud_nodelet.cc convert.cc driver.cc)
add_dependencies(cloud_nodelet_pandar ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cloud_nodelet_pandar
					  pandar_rawdata
					  pandar_input
					  pcap
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS cloud_nodelet_pandar
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(ringcolors_node_pandar ringcolors_node.cc colors.cc)
target_link_libraries(ringcolors_node_pandar
					  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS ringcolors_node_pandar
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(ringcolors_nodelet_pandar ringcolors_nodelet.cc colors.cc)
target_link_libraries(ringcolors_nodelet_pandar
					  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS ringcolors_nodelet_pandar
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(transform_node_pandar transform_node.cc transform.cc)
add_dependencies(transform_node_pandar ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_node_pandar pandar_rawdata
					  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS transform_node_pandar
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(transform_nodelet_pandar transform_nodelet.cc transform.cc)
add_dependencies(transform_nodelet_pandar ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_nodelet_pandar pandar_rawdata
					  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS transform_nodelet_pandar
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
