

add_library(passThroughFilter passThroughFilter.cc )
target_link_libraries(passThroughFilter
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_library(outlierRemovalFilter outlierRemovalFilter.cc )
target_link_libraries(outlierRemovalFilter
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_library(planeFilter planeFilter.cc )
target_link_libraries(planeFilter
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_library(differenceOfNormals differenceOfNormals.cc )
target_link_libraries(differenceOfNormals
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})


#add_executable(lu_node lu_node.cc)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(

	${catkin_EXPORTED_TARGETS})
endif()

#target_link_libraries(
#   	passThroughFilter
#    outlierRemovalFilter
#    ${catkin_LIBRARIES}
#    ${libpcap_LIBRARIES}
#    )


## Mark the nodelet library for installations
#install(TARGETS passThroughFilter
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES nodelet.xml nodelet.launch
#DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
