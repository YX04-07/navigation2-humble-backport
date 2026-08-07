if(NOT TARGET tf2_sensor_msgs::tf2_sensor_msgs)
  add_library(tf2_sensor_msgs::tf2_sensor_msgs INTERFACE IMPORTED)
  set_target_properties(tf2_sensor_msgs::tf2_sensor_msgs PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${tf2_sensor_msgs_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${tf2_sensor_msgs_LIBRARIES}"
  )
endif()