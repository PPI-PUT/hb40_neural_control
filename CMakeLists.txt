cmake_minimum_required(VERSION 3.14)
project(hb40_neural_controller)

find_package(ament_cmake_auto REQUIRED)
find_package(Torch REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED) # remove after factor msgs 
ament_auto_find_build_dependencies()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)
# set(msg "msg/hb40_neural_controller.msg")
# ament_auto_generate_messages(${msg} )

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/hb40_neural_controller/hb40_neural_controller.hpp
  include/hb40_neural_controller/hb40_neural_controller_constants.hpp
  include/hb40_neural_controller/visibility_control.hpp
  src/hb40_neural_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${TORCH_LIBRARIES})

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/hb40_neural_controller/hb40_neural_controller_node.hpp
  src/hb40_neural_controller_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "hb40_neural_controller::Hb40NeuralControllerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)



ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
