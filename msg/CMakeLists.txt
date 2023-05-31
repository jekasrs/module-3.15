find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
  DEPENDENCIES geometry_msgs
)

