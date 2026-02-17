module koda::ROSGenerator

import koda::AST;
import koda::Parser;
import koda::CST2AST;

import IO;
import List;
import String;
import vis::Text;

alias Env = map[str, value];
alias Result = tuple[Env, value];

// This represents a single dezyne call
data CallTpl
  = ctpl(str name, list[Argument] arguments, str ret)
  | empty_ctpl()
  ;

// These represent a capability, for example, for action, topic, or service
data CapabilityDef = capDef(
  str name, str ttype, str msg,
  CallTpl trigger,
  CallTpl onReturn,
  CallTpl onError,
  CallTpl onAbort
);

data CapabilityData
  = capData(str includes, str members, str methods, str parameters, str constructor, str startUp,
            list[str] deps, list[str] packageDeps, list[str] configParam)
  | empty_data()
  ;

loc BASE_DIR = |project://koda/generated/|;
loc INCLUDE_DIR = |project://koda/generated/include|;
loc SRC_OUTPUT_DIR = |project://koda/generated/source|;
str CLASS_NAME = "Supervisor";
str PARAMETER_DIR = "config";
str PARAMETER_FILE = "params.yaml";

// =============================================================
// Support files
public void generateCMakeLists(str packageName, list[str] deps)
{
  str output = trim("
cmake_minimum_required(VERSION 3.8)
project(<toLowerCase(packageName)>)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -g\")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES \"Clang\")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Added
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
<for (dep <- deps){>
find_package(<dep> REQUIRED)<}>

file(GLOB SOURCES \"${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc\")
add_executable(${PROJECT_NAME}
  src/main.cpp
  ${SOURCES}
)

ament_target_dependencies(${PROJECT_NAME}<for (dep <- deps){>
  <dep><}>
)

# Add include directories so .hh files can be found
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/dzn
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
");

  loc filename = BASE_DIR + "CMakeLists.txt";
  touch(filename);
  writeFile(filename, output);
}

public void generatePackage(str packageName, list[str] deps)
{
    str output = trim("
\<?xml version=\"1.0\"?\>
\<?xml-model href=\"http://download.ros.org/schema/package_format3.xsd\" schematypens=\"http://www.w3.org/2001/XMLSchema\"?\>
\<package format=\"3\"\>
  \<name\><toLowerCase(packageName)>\</name\>
  \<version\>0.0.0\</version\>
  \<description\>TODO\</description\>
  \<maintainer email=\"todo@todo.com\"\>TODO\</maintainer\>
  \<license\>Apache-2.0\</license\>

  \<buildtool_depend\>ament_cmake\</buildtool_depend\>

  \<test_depend\>ament_lint_auto\</test_depend\>
  \<test_depend\>ament_lint_common\</test_depend\>

  \<depend\>rclcpp\</depend\>
  \<depend\>rclcpp_action\</depend\>
<for (dep <- deps){>
  \<depend\><dep>\</depend\><}>

  \<exec_depend\>ros2launch\</exec_depend\>

  \<export\>
    \<build_type\>ament_cmake\</build_type\>
  \</export\>
\</package\>

");

  loc filename = BASE_DIR + "package.xml";
  touch(filename);
  writeFile(filename, output);
}

public void generateLaunch(str packageName)
{
    str output = trim("
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = Path(get_package_share_directory(\'<toLowerCase(packageName)>\'))
    params = str(pkg_share / \'<PARAMETER_DIR>\' / \'<PARAMETER_FILE>\')
    return LaunchDescription([
        Node(
            package=\'<toLowerCase(packageName)>\',
            executable=\'<toLowerCase(packageName)>\',
            name=\'<uncapitalize(CLASS_NAME)>\',
            output=\'screen\',
            parameters=[params]
        )
    ])
");

  loc filename = BASE_DIR + "/launch/default.launch.py";
  touch(filename);
  writeFile(filename, output);
}

public void generateParameters(list[str] parameters)
{
    str output = trim("
<uncapitalize(CLASS_NAME)>:
  ros__parameters:<for (param <- parameters){>
    <param><}>
");

  loc filename = BASE_DIR + "/<PARAMETER_DIR>/<PARAMETER_FILE>";
  touch(filename);
  writeFile(filename, output);
}

// =============================================================
// Capability nodes
public CapabilityData generateInitialPose(str key, CapabilityDef cap)
{
  str includes = "";
  str members = "";
  str methods = "";
  str parameters = "";
  str constructor = "";
  str startUp = "";
  list[str] deps = [];
  list[str] packageDeps = [];
  list[str] configParam = [];

  // ======================================================================================================
  // Include dependencies
  deps += "geometry_msgs";

  // ======================================================================================================
  // Package xml dependencies
  packageDeps += "geometry_msgs";

  // ======================================================================================================
  // Includes - These are the necessary headers
  includes += "// InitialPose ========================================================================== \n";
  includes += "#include \<geometry_msgs/msg/pose_with_covariance_stamped.hpp\>\n";

  // ======================================================================================================
  // Members - These are the members needed for this capability
  members += "  // InitialPose ========================================================================== \n";
  members += "  std::atomic\<bool\> got_amcl_pose_{false};\n";
  members += "  rclcpp::Publisher\<geometry_msgs::msg::PoseWithCovarianceStamped\>::SharedPtr initialpose_pub_;\n";
  members += "  rclcpp::Subscription\<geometry_msgs::msg::PoseWithCovarianceStamped\>::SharedPtr amcl_pose_sub_;\n\n";

  // ======================================================================================================
  // Constructor - These are the actions necessary for the correct construction of this capability
  constructor += "  // InitialPose ========================================================================== \n";
  constructor += "  initialpose_pub_ = create_publisher\<geometry_msgs::msg::PoseWithCovarianceStamped\>(\"/initialpose\", 10);\n";
  constructor += "  amcl_pose_sub_ = create_subscription\<geometry_msgs::msg::PoseWithCovarianceStamped\>(\"/amcl_pose\", 10,\n";
  constructor += "   [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {\n";
  constructor += "      got_amcl_pose_.store(true);\n";
  constructor += "      auto tmp = amcl_pose_sub_;\n";
  constructor += "      amcl_pose_sub_.reset();\n";
  constructor += " }, options);\n\n";

  // ======================================================================================================
  // Parameters - These are the paramaters needed for this capability
  parameters += "  // InitialPose ========================================================================== \n";
  parameters += "  declare_parameter\<bool\>(\"from_start\", true);\n";
  parameters += "  declare_parameter\<std::string\>(\"map_frame\", \"map\");\n";
  parameters += "  declare_parameter\<double\>(\"initial_x\", 0.0);\n";
  parameters += "  declare_parameter\<double\>(\"initial_y\", 0.0);\n";
  parameters += "  declare_parameter\<double\>(\"initial_yaw\", 0.0);\n\n";

  configParam += "from_start: true";
  configParam += "map_frame: map";
  configParam += "initial_x: -0.0252";
  configParam += "initial_y: -0.0276";
  configParam += "initial_yaw: 0.0";

  // ======================================================================================================
  // Start - Here we add any start up actions needed by this capability

  // ======================================================================================================
  // Methods - Here we add any methods that this capability might add to the supervisor
  if (ctpl(str _, list[Argument] _, str ret) := cap.trigger)
  {
    methods += "<ret> <CLASS_NAME>::<key>Trigger()\n";
    methods += "{\n";
    methods += "  const auto map_frame = get_parameter(\"map_frame\").as_string();\n";
    methods += "  const double x = get_parameter(\"initial_x\").as_double();\n";
    methods += "  const double y = get_parameter(\"initial_y\").as_double();\n";
    methods += "  const double yaw = get_parameter(\"initial_yaw\").as_double();\n\n";

    methods += "  geometry_msgs::msg::PoseWithCovarianceStamped msg;\n";
    methods += "  msg.header.stamp = now();\n";
    methods += "  msg.header.frame_id = map_frame;\n";
    methods += "  msg.pose.pose.position.x = x;\n";
    methods += "  msg.pose.pose.position.y = y;\n";
    methods += "  msg.pose.pose.position.z = 0.0;\n\n";

    methods += "  tf2::Quaternion q;\n";
    methods += "  q.setRPY(0, 0, yaw);\n";
    methods += "  msg.pose.pose.orientation = tf2::toMsg(q);\n\n";

    methods += "  const double var_xy = 0.25 * 0.25;\n";
    methods += "  const double var_yaw = (10.0 * M_PI / 180.0) * (10.0 * M_PI / 180.0);\n";
    methods += "  for (double& c : msg.pose.covariance)\n";
    methods += "    c = 0.0;\n";
    methods += "  msg.pose.covariance[0] = var_xy;\n";
    methods += "  msg.pose.covariance[7] = var_xy;\n";
    methods += "  msg.pose.covariance[35] = var_yaw;\n\n";

    methods += "  RCLCPP_INFO(get_logger(), \"Publishing initial pose at (%.2f, %.2f, yaw=%.2f rad) in %s.\", x, y, yaw, map_frame.c_str());\n";
    methods += "  for (int i = 0; i \< 10; ++i) {\n";
    methods += "    msg.header.stamp = now();\n";
    methods += "    initialpose_pub_-\>publish(msg);\n";
    methods += "    rclcpp::sleep_for(std::chrono::milliseconds(100));\n";
    methods += "  }\n\n";

    methods += "  if (task_thread_.joinable())\n";
    methods += "    task_thread_.join();\n";

    methods += "  task_thread_ = std::thread([this] {\n";
    methods += "    auto deadline = now() + rclcpp::Duration::from_seconds(2.0);\n";
    methods += "    while (!got_amcl_pose_.load() && now() \< deadline) {\n";
    methods += "      rclcpp::sleep_for(std::chrono::milliseconds(50));\n";
    methods += "    }\n\n";

    methods += "    if (got_amcl_pose_.load()) {\n";
    methods += "      RCLCPP_INFO(get_logger(), \"AMCL pose received after initialpose publish.\");\n";
    methods += "      rclcpp::sleep_for(std::chrono::milliseconds(2000));\n";
    methods += "      if (<key>_position_set)\n";
    methods += "        <key>_position_set();\n";
    methods += "    } else {\n";
    methods += "      RCLCPP_WARN(get_logger(), \"No /amcl_pose observed yet; continuing anyway.\");\n";
    methods += "      if (<key>_failed)\n";
    methods += "        <key>_failed();\n";
    methods += "    }\n";
    methods += "  });\n";
    methods += "}\n";
  }

  return capData(includes, members, methods, parameters, constructor, startUp,deps, packageDeps, configParam);
}

public CapabilityData generateDrive(str key, CapabilityDef cap)
{
  str includes = "";
  str members = "";
  str methods = "";
  str parameters = "";
  str constructor = "";
  str startUp = "";
  list[str] deps = [];
  list[str] packageDeps = [];
  list[str] configParam = [];

  // ======================================================================================================
  // Include dependencies
  deps += "nav2_msgs";
  deps += "control_msgs";
  deps += "trajectory_msgs";

  // ======================================================================================================
  // Package xml dependencies
  packageDeps += "nav2_msgs";
  packageDeps += "control_msgs";
  packageDeps += "trajectory_msgs";

  // ======================================================================================================
  // Includes - These are the necessary headers
  includes += "// Drive ========================================================================== \n";
  includes += "#include \<nav2_msgs/action/navigate_to_pose.hpp\>\n";
  includes += "#include \<nav_msgs/msg/odometry.hpp\>\n";

  // ======================================================================================================
  // Members - These are the members needed for this capability
  members += "  // Drive ========================================================================== \n";
  members += "  rclcpp_action::Client\<nav2_msgs::action::NavigateToPose\>::SharedPtr nav_client_;\n\n";

  // ======================================================================================================
  // Constructor - These are the actions necessary for the correct construction of this capability
  // TODO: Callback groups should be passed to the capability template
  constructor += "  // Drive ========================================================================== \n";
  constructor += "  nav_client_ = rclcpp_action::create_client\<nav2_msgs::action::NavigateToPose\>(this, \"navigate_to_pose\", cbg_);\n\n";

  // ======================================================================================================
  // Parameters - These are the paramaters needed for this capability
  parameters += "  // Drive ========================================================================== \n";

  // ======================================================================================================
  // Start - Here we add any start up actions needed by this capability
  startUp += "  if (!nav_client_-\>wait_for_action_server(5s)) {\n";
  startUp += "    RCLCPP_ERROR(get_logger(), \"Nav2 \'navigate_to_pose\' action not available.\");\n";
  startUp += "    rclcpp::shutdown();\n";
  startUp += "    return;\n";
  startUp += "  }\n\n";

  // ======================================================================================================
  // Methods - Here we add any methods that this capability might add to the supervisor
  if (ctpl(str _, list[Argument] _, str ret) := cap.trigger)
  {
    methods += "  // Drive ========================================================================== \n";
    methods += "<ret> <CLASS_NAME>::<key>Trigger(float x, float y, float yaw)\n";
    methods += "{\n";
    methods += "  auto frame_id = get_parameter(\"map_frame\").as_string();\n";
    methods += "  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();\n";
    methods += "  goal_msg.pose.header.stamp = now();\n";
    methods += "  goal_msg.pose.header.frame_id = frame_id;\n";
    methods += "  goal_msg.pose.pose.position.x = x;\n";
    methods += "  goal_msg.pose.pose.position.y = y;\n\n";

    methods += "  tf2::Quaternion q;\n";
    methods += "  q.setRPY(0, 0, yaw);\n";
    methods += "  goal_msg.pose.pose.orientation = tf2::toMsg(q);\n\n";

    methods += "  rclcpp_action::Client\<nav2_msgs::action::NavigateToPose\>::SendGoalOptions opts;\n";
    methods += "  opts.goal_response_callback =\n";
    methods += "      [this](auto gh) {\n";
    methods += "        RCLCPP_INFO(this-\>get_logger(), gh ? \"Goal ACCEPTED\" : \"Goal REJECTED\");\n";
    methods += "      };\n";
    methods += "  opts.result_callback =\n";
    methods += "      [this, x, y, yaw](const auto& wr) {\n";
    methods += "        RCLCPP_INFO(this-\>get_logger(), \"Result code=%d\", (int)wr.code);\n";
    methods += "        if (wr.code != rclcpp_action::ResultCode::SUCCEEDED) {\n";
    methods += "          if (<key>_path_blocked)\n";
    methods += "            <key>_path_blocked();\n";
    methods += "        } else {\n";
    methods += "          if (<key>_in_position)\n";
    methods += "            <key>_in_position(x, y, yaw);\n";
    methods += "        }\n";
    methods += "      };\n\n";
    methods += "  nav_client_-\>async_send_goal(goal_msg, opts);\n";
    methods += "}\n\n";
  }

  return capData(includes, members, methods, parameters, constructor, startUp, deps, packageDeps, configParam);
}

public CapabilityData generateVision(str key, CapabilityDef cap)
{
  str includes = "";
  str members = "";
  str methods = "";
  str parameters = "";
  str constructor = "";
  str startUp = "";
  list[str] deps = [];
  list[str] packageDeps = [];
  list[str] configParam = [];

  // ======================================================================================================
  // Include dependencies
  deps += "cv_bridge";
  deps += "OpenCV";
  deps += "sensor_msgs";

  // ======================================================================================================
  // Package xml dependencies
  packageDeps += "cv_bridge";
  packageDeps += "opencv";
  packageDeps += "sensor_msgs";

  // ======================================================================================================
  // Includes - These are the necessary headers
  includes += "// ArucoVision ========================================================================== \n";
  includes += "#include \<opencv2/aruco.hpp\>\n";
  includes += "#include \<opencv2/opencv.hpp\>\n";
  includes += "#include \<sensor_msgs/msg/image.hpp\>\n";
  includes += "#include \<std_msgs/msg/color_rgba.hpp\>\n";
  includes += "#include \<cv_bridge/cv_bridge.h\>\n";

  // ======================================================================================================
  // Members - These are the members needed for this capability
  members += "  // ArucoVision ========================================================================== \n";
  members += "  rclcpp::Subscription\<sensor_msgs::msg::Image\>::SharedPtr image_sub_;\n";
  members += "  cv::Ptr\<cv::aruco::Dictionary\> aruco_dict_;\n";
  members += "  cv::Ptr\<cv::aruco::DetectorParameters\> detector_params_;\n";
  members += "  std::mutex img_mtx_;\n";
  members += "  std::condition_variable img_cv_;\n";
  members += "  sensor_msgs::msg::Image::SharedPtr last_img_;\n";
  members += "  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg);\n\n";

  // ======================================================================================================
  // Constructor - These are the actions necessary for the correct construction of this capability
  constructor += "  // ArucoVision ========================================================================== \n";
  constructor += "  aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);\n";
  constructor += "  detector_params_ = cv::aruco::DetectorParameters::create();\n";
  constructor += "  image_sub_ = create_subscription\<sensor_msgs::msg::Image\>(get_parameter(\"camera_topic\").as_string(), rclcpp::SensorDataQoS(), std::bind(&Supervisor::imageCb, this, std::placeholders::_1), options);\n\n";

  // ======================================================================================================
  // Parameters - These are the paramaters needed for this capability
  parameters += "  // ArucoVision ========================================================================== \n";
  parameters += "  declare_parameter\<std::string\>(\"camera_topic\", \"/pi_camera/image_raw\");\n";
  parameters += "  declare_parameter\<std::string\>(\"camera_frame\", \"camera_link\");\n";
  parameters += "  declare_parameter\<int\>(\"aruco_id\", 23);\n";
  parameters += "  declare_parameter\<double\>(\"aruco_size_m\", 0.08);\n";
  parameters += "  declare_parameter\<double\>(\"arrival_tolerance_m\", 0.30);\n";
  parameters += "  declare_parameter\<double\>(\"aruco_timeout_s\", 15.0);\n\n";

  configParam += "aruco_size_m: 0.08";
  configParam += "arrival_tolerance_m: 0.30";
  configParam += "aruco_timeout_s: 15.0";

  // ======================================================================================================
  // Start - Here we add any start up actions needed by this capability

  // ======================================================================================================
  // Methods - Here we add any methods that this capability might add to the supervisor
  if (ctpl(str _, list[Argument] _, str ret) := cap.trigger)
  {
    methods += "// ArucoVision ========================================================================== \n";
    methods += "<ret> <CLASS_NAME>::<key>Trigger()\n";
    methods += "{\n";
    methods += "  const int target_id = get_parameter(\"aruco_id\").as_int();\n";
    methods += "  const double size_m = get_parameter(\"aruco_size_m\").as_double();\n";
    methods += "  const double tol_m = get_parameter(\"arrival_tolerance_m\").as_double();\n";
    methods += "  const double timeout_s = get_parameter(\"aruco_timeout_s\").as_double();\n";
    methods += "  RCLCPP_INFO(get_logger(), \"Checking ArUco id=%d within %.2f size and %.2fm tolerance...\", target_id, size_m, tol_m);\n\n";

    methods += "  if (task_thread_.joinable())\n";
    methods += "    task_thread_.join();\n\n";

    methods += "  task_thread_ = std::thread([this] {\n";
    methods += "    while (true) {\n";
    methods += "      {\n";
    methods += "        std::unique_lock\<std::mutex\> lock(abort_mtx_);\n";
    methods += "        if (aborted_)\n";
    methods += "          break;\n";
    methods += "      }\n\n";

    methods += "      sensor_msgs::msg::Image::SharedPtr img;\n";
    methods += "      {\n";
    methods += "        std::unique_lock\<std::mutex\> lk(img_mtx_);\n";
    methods += "        if (!img_cv_.wait_for(lk, 500ms, [&] { return last_img_ != nullptr; }))\n";
    methods += "          continue;\n";
    methods += "        img = last_img_;\n";
    methods += "        last_img_.reset();  // consume\n";
    methods += "      }\n\n";

    methods += "      cv::Mat frame;\n";
    methods += "      try {\n";
    methods += "        frame = cv_bridge::toCvCopy(img, \"bgr8\")-\>image;\n";
    methods += "      } catch (const std::exception& e) {\n";
    methods += "        RCLCPP_WARN(get_logger(), \"Failed to cv_bridge::toCvCopy\");\n";
    methods += "        continue;\n";
    methods += "      }\n";

    methods += "      if (frame.empty()) {\n ";
    methods += "        RCLCPP_WARN(get_logger(), \"Frame is empty\");\n ";
    methods += "        continue;\n ";
    methods += "      }\n\n";

    methods += "      std::vector\<int\> ids;\n";
    methods += "      std::vector\<std::vector\<cv::Point2f\>\> corners;\n";
    methods += "      cv::aruco::detectMarkers(frame, aruco_dict_, corners, ids, detector_params_);\n";
    methods += "      if (ids.empty()) {\n";
    methods += "        RCLCPP_WARN(get_logger(), \"Failed to detect markers, saving image\");\n";
    methods += "        cv::imwrite(\"/tmp/cam.png\", frame);\n";
    methods += "        continue;\n";
    methods += "      } else {\n";
    methods += "        RCLCPP_INFO(get_logger(), \"Found %ld ids\", ids.size());\n";
    methods += "        if (<key>_found)\n";
    methods += "          <key>_found();\n";
    methods += "        return;\n";
    methods += "      }\n";
    methods += "    }\n";
    methods += "  });\n";
    methods += "}\n";
  }

  methods += "void Supervisor::imageCb(const sensor_msgs::msg::Image::SharedPtr msg)\n";
  methods += "{\n";
  methods += "  {\n";
  methods += "    std::unique_lock\<std::mutex\> lk(img_mtx_);\n";
  methods += "    last_img_ = msg;\n";
  methods += "  }\n";
  methods += "  img_cv_.notify_all();\n";
  methods += "}\n\n";

  return capData(includes, members, methods, parameters, constructor, startUp, deps, packageDeps, configParam);
}

public CapabilityData generateGrip(str key, CapabilityDef cap)
{
  str includes = "";
  str members = "";
  str methods = "";
  str parameters = "";
  str constructor = "";
  str startUp = "";
  list[str] deps = [];
  list[str] packageDeps = [];
  list[str] configParam = [];

  // ======================================================================================================
  // Include dependencies
  deps += "moveit_ros_planning_interface";
  deps += "tf2_geometry_msgs";

  // ======================================================================================================
  // Package xml dependencies
  packageDeps += "moveit_ros_planning_interface";
  packageDeps += "tf2_geometry_msgs";

  // ======================================================================================================
  // Includes - These are the necessary headers
  includes += "// Grip ========================================================================== \n";
  includes += "#include \<moveit/move_group_interface/move_group_interface.h\>\n";
  includes += "#include \<tf2_geometry_msgs/tf2_geometry_msgs.hpp\>\n";

  // ======================================================================================================
  // Members - These are the members needed for this capability
  members += "  // Grip ========================================================================== \n";
  members += "  std::shared_ptr\<moveit::planning_interface::MoveGroupInterface\> arm_mgi_;\n";
  members += "  bool moveGripperHome();\n\n";

  // ======================================================================================================
  // Constructor - These are the actions necessary for the correct construction of this capability

  // ======================================================================================================
  // Parameters - These are the paramaters needed for this capability
  parameters += "  // Grip ========================================================================== \n";
  parameters += "  declare_parameter\<std::vector\<double\>\>(\"target_offset_xyz\", {0.18, 0.0, 0.2});\n";
  parameters += "  declare_parameter\<std::vector\<double\>\>(\"target_offset_rpy\", {0.0, 0.0, 0.03});\n";
  parameters += "  declare_parameter\<std::vector\<double\>\>(\"drive_pose\", {-0.068, 0.0, 0.26});\n";
  parameters += "  declare_parameter\<std::string\>(\"arm_group\", \"arm\");\n";
  parameters += "  declare_parameter\<std::string\>(\"eef_link\", \"end_effector_link\");\n\n";

  configParam += "arm_group: \"arm\"";
  configParam += "eef_link: \"end_effector_link\"";
  configParam += "drive_pose: [-0.068, 0.0, 0.26]";
  configParam += "target_offset_xyz: [0.18, 0.0, 0.2]";
  configParam += "target_offset_rpy: [0.0, 0.0, 0.03]";

  // ======================================================================================================
  // Start - Here we add any start up actions needed by this capability
  startUp += "  if (arm_mgi_ == nullptr){ \n";
  startUp += "    arm_mgi_ = std::make_shared\<moveit::planning_interface::MoveGroupInterface\>(shared_from_this(), get_parameter(\"arm_group\").as_string());\n";
  startUp += "    arm_mgi_-\>setEndEffectorLink(get_parameter(\"eef_link\").as_string());\n";
  startUp += "  }\n\n";

  startUp += "  if (!moveGripperHome()) {\n";
  startUp += "    RCLCPP_ERROR(get_logger(), \"Failed to set arm to drive position.\");\n";
  startUp += "    rclcpp::shutdown();\n";
  startUp += "    return;\n";
  startUp += "  }\n\n";

  // ======================================================================================================
  // Methods - Here we add any methods that this capability might add to the supervisor
  if (ctpl(str _, list[Argument] _, str ret) := cap.trigger)
  {
    methods += "void <CLASS_NAME>::<key>Trigger(int grip)\n";
    methods += "{\n";
    methods += "  if (task_thread_.joinable())\n";
    methods += "    task_thread_.join();\n\n";

    methods += "  task_thread_ = std::thread([this] {\n";
    methods += "    auto offs_xyz = get_parameter(\"target_offset_xyz\").as_double_array();\n\n";

    methods += "    const auto planning_frame = arm_mgi_-\>getPlanningFrame();\n";
    methods += "    geometry_msgs::msg::PoseStamped goal;\n";
    methods += "    goal.header.stamp = now();\n";
    methods += "    goal.header.frame_id = planning_frame;\n";
    methods += "    goal.pose.position.x = offs_xyz[0];\n";
    methods += "    goal.pose.position.y = offs_xyz[1];\n";
    methods += "    goal.pose.position.z = offs_xyz[2];\n\n";

    // TODO: These are all great candidates for capability parameters
    methods += "    arm_mgi_-\>setPoseReferenceFrame(planning_frame);\n";
    methods += "    arm_mgi_-\>setStartStateToCurrentState();\n";
    methods += "    arm_mgi_-\>clearPoseTargets();\n";
    methods += "    arm_mgi_-\>setGoalTolerance(0.01);\n";
    methods += "    arm_mgi_-\>setGoalOrientationTolerance(0.25);\n";
    methods += "    arm_mgi_-\>setPlanningTime(10.0);\n";
    methods += "    arm_mgi_-\>setNumPlanningAttempts(10);\n\n";

    methods += "    arm_mgi_-\>setPositionTarget(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, arm_mgi_-\>getEndEffectorLink());\n\n";

    methods += "    moveit::planning_interface::MoveGroupInterface::Plan plan;\n";
    methods += "    if (arm_mgi_-\>plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {\n";
    methods += "      RCLCPP_ERROR(get_logger(), \"MoveIt plan failed.\");\n";
    methods += "      if (<key>_failed)\n";
    methods += "        <key>_failed();\n";
    methods += "      return;\n";
    methods += "    }\n\n";

    methods += "    if (arm_mgi_-\>execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {\n";
    methods += "      RCLCPP_ERROR(get_logger(), \"MoveIt exec failed.\");\n";
    methods += "      if (grip_failed)\n";
    methods += "        grip_failed();\n";
    methods += "      return;\n";
    methods += "    }\n\n";

    // TODO: Do we really need this?
    // methods += "    rclcpp::sleep_for(std::chrono::seconds(2));\n";
    methods += "    if (!moveGripperHome())\n";
    methods += "    {\n";
    methods += "      RCLCPP_ERROR(get_logger(), \"Failed to set arm to drive position.\");\n";
    methods += "      if (grip_failed)\n";
    methods += "        grip_failed();\n";
    methods += "      return;\n";
    methods += "    }\n\n";

    methods += "    if (<key>_handled)\n";
    methods += "      <key>_handled();\n";
    methods += "  });\n";
    methods += "}\n\n";
  }

  methods += "bool <CLASS_NAME>::moveGripperHome()\n";
  methods += "{\n";
  methods += "  if (!arm_mgi_)\n";
  methods += "  {\n";
  methods += "    RCLCPP_WARN(get_logger(), \"No arm mgi\");\n";
  methods += "    return false;\n";
  methods += "  }\n\n";

  methods += "  auto drive_pose = get_parameter(\"drive_pose\").as_double_array();\n\n";

  methods += "  const auto planning_frame = arm_mgi_-\>getPlanningFrame();\n";
  methods += "  arm_mgi_-\>setPoseReferenceFrame(planning_frame);\n";
  methods += "  arm_mgi_-\>setStartStateToCurrentState();\n";
  methods += "  arm_mgi_-\>clearPoseTargets();\n";
  methods += "  arm_mgi_-\>setGoalTolerance(0.01);             // 1 cm\n";
  methods += "  arm_mgi_-\>setGoalOrientationTolerance(0.25);  // ~20°\n";
  methods += "  arm_mgi_-\>setPlanningTime(10.0);\n";
  methods += "  arm_mgi_-\>setNumPlanningAttempts(10);\n\n";

  methods += "  arm_mgi_-\>setPositionTarget(drive_pose[0], drive_pose[1], drive_pose[2], arm_mgi_-\>getEndEffectorLink());\n\n";

  methods += "  moveit::planning_interface::MoveGroupInterface::Plan plan;\n";
  methods += "  if (arm_mgi_-\>plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {\n";
  methods += "    RCLCPP_ERROR(get_logger(), \"MoveIt plan to home failed.\");\n";
  methods += "    return false;\n";
  methods += "  }\n\n";

  methods += "  if (arm_mgi_-\>execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)\n";
  methods += "  {\n";
  methods += "    RCLCPP_ERROR(get_logger(), \"MoveIt exec failed.\");\n";
  methods += "    return false;\n";
  methods += "  }\n\n";

  methods += "  return true;\n";
  methods += "}\n";

  return capData(includes, members, methods, parameters, constructor, startUp, deps, packageDeps, configParam);
}

// =============================================================
// Helpers
void generateDir(loc output)
{
  if (exists(output))
    return;

  mkDirectory(output);
}

str clean(value val)
  = replaceAll("<val>", "\"", "");

str toComponent(str name)
  = "c" + uncapitalize(replaceAll(name, " ", ""));

str toInterface(str name)
  = "i" + uncapitalize(replaceAll(name, " ", ""));

str toVariable(str name)
  = uncapitalize(replaceAll(name, " ", ""));

str toFilename(str name)
  = replaceAll(name, " ", "");

// =============================================================
// Argument
str argsToString(list[Argument] args)
{
  return "<for (i <- [0..size(args)]){><argToString(args[i])><if(i < size(args) - 1){>, <}><}>";
}

str argToString(Argument arg)
{
  if (\arg(str arg_type, str arg_id) := arg)
    return "<arg_type> <arg_id>";

  return "";
}

str argIdToString(Argument arg)
{
  if (\arg(str _, str arg_id) := arg)
    return "<arg_id>";

  return "";
}

str argsIdToString(list[Argument] args)
{
  return return "<for (i <- [0..size(args)]){><argIdToString(args[i])><if(i < size(args) - 1){>, <}><}>";;
}
// ============================================================
// EventDefStatement
public CallTpl generate(\event(str return_type, str id, list[Argument] args, list[EventDefComponent] _))
{
  println("event: <id> -\> <return_type>");
  return ctpl(id, args, return_type);
}

// ============================================================
// RosDefStatement
public Result generate(\trigger_block(EventDefStatement call), Env env)
{
  // println("trigger_block: <call>");
  eventTemplate = generate(call);
  if ("capabilities" in env && CapabilityDef task := env["capabilities"]) {
    task.trigger = eventTemplate;
    env["capabilities"] = task;
  }

  return <env, "">;
}

public Result generate(\return_block(EventDefStatement call), Env env)
{
  // println("return_block: <call>");
  eventTemplate = generate(call);
  if ("capabilities" in env && CapabilityDef task := env["capabilities"]) {
    task.onReturn = eventTemplate;
    env["capabilities"] = task;
  }
  return <env, "">;
}

public Result generate(\abort_block(EventDefStatement call), Env env)
{
  // println("abort_block: <call>");
  eventTemplate = generate(call);
  if ("capabilities" in env && CapabilityDef task := env["capabilities"]) {
    task.onAbort = eventTemplate;
    env["capabilities"] = task;
  }
  return <env, "">;
}

public Result generate(\error_block(EventDefStatement call), Env env)
{
  // println("error_block: <call>");
  eventTemplate = generate(call);
  if ("capabilities" in env && CapabilityDef task := env["capabilities"]) {
    task.onError = eventTemplate;
    env["capabilities"] = task;
  }
  return <env, "">;
}

public Result generate(\in_block(EventDefStatement call), Env env)
{
  println("in_block: <call>");
  return <env, "">;
}

public Result generate(\out_block(EventDefStatement call), Env env)
{
  println("out_block: <call>");
  return <env, "">;
}

// ============================================================
// Statement
public Result generate(\variables(list[VariableStatement] vars), Env env)
{
  println("variables <vars>");
  return <env, "">;
}

public Result generate(\action(str topic, str msg, list[RosDefStatement] events), Env env)
{
  println("action <topic> <msg>");

  env["capabilities"] = capDef("action", topic, msg, empty_ctpl(), empty_ctpl(), empty_ctpl(), empty_ctpl());
  for (event <- events) {
    <env, _> = generate(event, env);
  }

  return <env, "">;
}

public Result generate(\service(str topic, str msg, list[RosDefStatement] events), Env env)
{
  println("service <topic> <msg>");
  for (event <- events) {
    <env, _> = generate(event, env);
  }
  return <env, "">;
}

public Result generate(\topic(str topic, str msg, list[RosDefStatement] events), Env env)
{
  println("topic <topic> <msg>");
  for (event <- events) {
    <env, _> = generate(event, env);
  }
  return <env, "">;
}

public Result generate(\ros_def(RosDefStatement statement), Env env)
{
  println("ros_def <statement>");
  return <env, "">;
}

public Result generate(\tasks_block(list[Flow] flows), Env env) {
  println("\tasks_block <flows>");
  str output = "";
  return <env, "">;
}

// ============================================================
// Top Level
public bool generateCapability(str id, CapabilityDef cap)
{
  // Prepare methods
  str methodsDef = "";
  str methodsImpl = "";
  str callbacks = "";
  println("[ROSGenerator] Trying to generate ROS class: <id>");
  if (ctpl(str name, list[Argument] args, str ret) := cap.trigger)
  {
    println("[ROSGenerator]   ROS class: <id> has a trigger");
    str arguments = argsToString(args);
    methodsDef += "  <ret> <id>_<name>(<arguments>) override;\n";

    methodsImpl += "<ret> <toComponent(id)>::<id>_<name>(<arguments>) {\n";
    methodsImpl += "  auto supervisor = dzn_locator.get\<std::shared_ptr\<<CLASS_NAME>\>\>();\n";
    methodsImpl += "  if (supervisor == nullptr)\n";
    methodsImpl += "  {\n";
    methodsImpl += "    std::cout \<\< \"Failed to retrieve the <CLASS_NAME>\" \<\< std::endl;\n";
    methodsImpl += "    return;\n";
    methodsImpl += "  }\n\n";
    methodsImpl += "  std::cout \<\< \"Triggering: <id>Trigger\" \<\< std::endl;\n";
    methodsImpl += "  supervisor-\><id>Trigger(<argsIdToString(args)>);\n";
    methodsImpl += "}\n\n";

    if (ctpl(str retName, list[Argument] retArgs, str _) := cap.onReturn) {
      println("[ROSGenerator]   ROS class: <id> has a return");
      callbacks += "  supervisor-\><id>_<retName> = [this](<argsToString(retArgs)>) {\n";
      callbacks += "    auto& pump = dzn_locator.get\<dzn::pump\>();\n";
      callbacks += "    pump([this<if (size(retArgs) > 0){>, <}><argsIdToString(retArgs)>] {\n";
      callbacks += "     std::cout \<\< \"Before <id>_<retName>\" \<\< std::endl;\n";
      callbacks += "     <id>_<retName>(<argsIdToString(retArgs)>);\n";
      callbacks += "     std::cout \<\< \"After <id>_<retName>\" \<\< std::endl;\n";
      callbacks += "    });\n";
      callbacks += "  };\n";
    }
  }

  if (ctpl(str name, list[Argument] args, str ret) := cap.onAbort)
  {
    println("[ROSGenerator]   ROS class: <id> has a abort");
    str arguments = argsToString(args);
    methodsDef += "  <ret> <id>_<name>(<arguments>) override;\n";

    methodsImpl += "<ret> <toComponent(id)>::<id>_<name>(<arguments>) {\n";
    methodsImpl += "  auto supervisor = dzn_locator.get\<std::shared_ptr\<<CLASS_NAME>\>\>();\n";
    methodsImpl += "  if (supervisor == nullptr)\n";
    methodsImpl += "  {\n";
    methodsImpl += "    std::cout \<\< \"Failed to retrieve the <CLASS_NAME>\" \<\< std::endl;\n";
    methodsImpl += "    return;\n";
    methodsImpl += "  }\n";
    methodsImpl += "  supervisor-\>abort(<argsIdToString(args)>);\n";
    methodsImpl += "}\n\n";

    if (ctpl(str retName, list[Argument] retArgs, str _) := cap.onError) {
      println("[ROSGenerator]   ROS class: <id> has a error");
      callbacks += "  supervisor-\><id>_<retName> = [this](<argsToString(retArgs)>) {\n";
      callbacks += "    auto& pump = dzn_locator.get\<dzn::pump\>();\n";
      callbacks += "    pump([this<if (size(retArgs) > 0){>, <}><argsIdToString(retArgs)>] {\n";
      callbacks += "     std::cout \<\< \"Before <id>_<retName>\" \<\< std::endl;\n";
      callbacks += "     <id>_<retName>(<argsIdToString(retArgs)>);\n";
      callbacks += "     std::cout \<\< \"After <id>_<retName>\" \<\< std::endl;\n";
      callbacks += "    });\n";
      callbacks += "  };\n";
    }
  }

  // Generate header
  str header = trim("
#pragma once

#include \"a_<toComponent(id)>.hh\"

class <toComponent(id)> : public skel::<toComponent(id)>
{
public:
  <toComponent(id)>(dzn::locator const& locator);
  ~<toComponent(id)>();

  void start();
<methodsDef>
};");

  // Generate source
  str source = trim("
#include \"<toComponent(id)>.hh\"
#include \<dzn/pump.hh\>
#include \"<uncapitalize(CLASS_NAME)>.hh\"

<toComponent(id)>::<toComponent(id)>(dzn::locator const& locator)
    : skel::<toComponent(id)>(locator)
{
}

<toComponent(id)>::~<toComponent(id)>()
{
}

void <toComponent(id)>::start()
{
  auto supervisor = dzn_locator.get\<std::shared_ptr\<<CLASS_NAME>\>\>();
  if (supervisor == nullptr)
  {
    std::cout \<\< \"Failed to retrieve the <CLASS_NAME> during start\" \<\< std::endl;
    return;
  }

<callbacks>
}

<methodsImpl>
;");

  loc header_filename = INCLUDE_DIR + "<toComponent(id)>.hh";
  loc source_filename = SRC_OUTPUT_DIR + "<toComponent(id)>.cc";

  touch(header_filename);
  touch(source_filename);

  writeFile(header_filename, header);
  writeFile(source_filename, source);

  return true;
}

public bool generateCapabilities(map[str, str] capMap, Env env)
{
  println("generateCapability: <capMap>");

  for (key <- capMap)
  {
    if (key in env && CapabilityDef cap := env[key])
      generateCapability(capMap[key], cap);
  }

  return true;
}

public bool generateSupervisor(str taskId, map[str, str] capMap, Env env)
{
  str capTriggers = "";
  str capCallbacks = "// Callbacks ========================================================\n";
  // str members = "";

  str includes = "";
  str members = "";
  str methods = "";
  str parameters = "";
  str constructor = "";
  str startUp = "";
  list[str] deps = [];
  list[str] packageDeps = [];
  list[str] configParam = [];

  for (key <- capMap)
  {
    if (key in env && CapabilityDef cap := env[key])
    {
      if (ctpl(str _, list[Argument] args, str ret) := cap.trigger)
      {
        capTriggers += "  // <key> ===============================================\n";
        capTriggers += "  <ret> <capMap[key]>Trigger(<argsToString(args)>);\n";
      }

      if (ctpl(str name, list[Argument] args, str ret) := cap.onReturn)
      {
        capCallbacks += "  std::function\<<ret>(<argsToString(args)>)\> <capMap[key]>_<name>;\n";
      }

      if (ctpl(str name, list[Argument] args, str ret) := cap.onError)
      {
        capCallbacks += "  std::function\<<ret>(<argsToString(args)>)\> <capMap[key]>_<name>;\n";
      }

      cData = empty_data();
      if (key == "Drive")
        cData = generateDrive(capMap[key], cap);
      else if (key == "Vision")
        cData = generateVision(capMap[key], cap);
      else if (key == "Grip")
        cData = generateGrip(capMap[key], cap);
      else if (key == "Initialpose")
        cData = generateInitialPose(capMap[key], cap);

      includes += cData.includes;
      members += cData.members;
      methods += cData.methods;
      parameters += cData.parameters;
      constructor += cData.constructor;
      startUp += cData.startUp;
      deps += cData.deps;
      packageDeps += cData.packageDeps;
      configParam += cData.configParam;
    }

  }

  str header = trim("
#pragma once

#include \<functional\>
#include \<string\>
#include \<memory\>
#include \<thread\>
#include \<mutex\>
#include \<condition_variable\>

#include \<rclcpp/rclcpp.hpp\>
#include \<rclcpp_action/rclcpp_action.hpp\>
<includes>

class <CLASS_NAME> : public rclcpp::Node
{
public:
  <CLASS_NAME>();

  void start();
  void abort();

<capTriggers>
<capCallbacks>

  std::function\<void()\> started;

private:
  // Generic ========================================================
  std::mutex abort_mtx_;
  bool aborted_{false};
  std::thread task_thread_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr cbg_;

<members[..-2]>
};
");

  str source = trim("
#include \"<uncapitalize(CLASS_NAME)>.hh\"

using namespace std::chrono_literals;

<CLASS_NAME>::<CLASS_NAME>()
    : Node(\"<uncapitalize(CLASS_NAME)>\")
{
<parameters>
  // Generic ========================================================
  cbg_ = this-\>create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions options;
  options.callback_group = cbg_;

<constructor>
  RCLCPP_INFO(get_logger(), \"Supervisor ready.\");
  timer_ = create_wall_timer(2s, std::bind(&Supervisor::start, this));
}

void <CLASS_NAME>::start()
{
  timer_-\>cancel();

  {
    std::unique_lock\<std::mutex\> lock(abort_mtx_);
    aborted_ = false;
  }

<startUp[..-2]>

  if (started)
    started();

  return;
}

void <CLASS_NAME>::abort()
{
  {
    std::unique_lock\<std::mutex\> lock(abort_mtx_);
    aborted_ = true;
  }

  if (task_thread_.joinable())
    task_thread_.join();
}

<methods>
");

  loc header_filename = INCLUDE_DIR + "<uncapitalize(CLASS_NAME)>.hh";
  loc source_filename = SRC_OUTPUT_DIR + "<uncapitalize(CLASS_NAME)>.cc";

  touch(header_filename);
  touch(source_filename);

  writeFile(header_filename, header);
  writeFile(source_filename, source);

  generateCMakeLists(taskId, deps);
  generatePackage(taskId, packageDeps);
  generateLaunch(taskId);
  generateParameters(configParam);

  return true;
}

public bool generateMain(str taskId, map[str, str] capMap, Env env)
{
  str capabilities = "";
  for (key <- capMap)
    capabilities += "  system-\><capMap[key]>.start();\n";

  str source = trim("
#include \<iostream\>
#include \<memory\>
#include \<rclcpp/rclcpp.hpp\>

#include \"<toComponent(taskId)>_task.hh\"
#include \"<uncapitalize(CLASS_NAME)>.hh\"

std::ostream nullstream(nullptr);
dzn::runtime runtime;
dzn::locator locator;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared\<<CLASS_NAME>\>();

  auto system = std::make_shared\<<toComponent(taskId)>_task\>(locator.set(runtime).set(nullstream).set(node));

  system-\>api.out.failed = [] {
    std::cout \<\< \"Operation failed\" \<\< std::endl;
  };

<capabilities>
  node-\>started = [system] { system-\>api.in.start(3.6927, -0.048, 0.0, 2.08, 2.54, 1.55); };

  std::cout \<\< \"Initialized\" \<\< std::endl;

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  std::cout \<\< \"Done\" \<\< std::endl;
  system-\>api.in.abort();

  rclcpp::shutdown();
  return 0;
}
");

  loc main_filename = SRC_OUTPUT_DIR + "main.cpp";
  touch(main_filename);
  writeFile(main_filename, source);

  return true;
}

public Result generate(\task(str id, list[Argument] args, list[Statement] tstatements), Env env) {
  println("Generating ROS methods for: <id>");

  // Get the necessary arguments
  map[str, str] capMap = ();
  for (arg <- args)
  {
    if (\requires(str arg_type, str arg_id) := arg)
      capMap[arg_type] = arg_id;
  }

  // Generate capability files
  generateCapabilities(capMap, env);

  // Now, we generate the supervisor
  generateSupervisor(id, capMap, env);

  // Finally, we generate the main file
  generateMain(id, capMap, env);

  return <env, "">;
}

public Result generate(\capability(str id, list[Argument] args, list[Statement] tstatements), Env env) {
  println("Generating capability: <id>");

  // Get all the necessary capability info
  lenv = env;
  for (s <- tstatements, \tasks_block(_) !:= s)
  {
    <lenv, _> = generate(s, lenv);
    if ("capabilities" in lenv && CapabilityDef task := lenv["capabilities"])
      env[id] = task;
  }

  return <env, "">;
}


public int generate(koda::AST::System system, loc output_dir)
{
  BASE_DIR = output_dir;
  INCLUDE_DIR = BASE_DIR + "/include";
  generateDir(INCLUDE_DIR);

  SRC_OUTPUT_DIR = BASE_DIR + "/src";
  generateDir(SRC_OUTPUT_DIR);

  // The ROS component only needs the methods of the different capabilities
  Env env = ();

  // First we build the capabilities
  for (t <- system.components) {
    if (\capability(_, _, _) := t)
      <env, _> = generate(t, env);
  }

    for (t <- system.components) {
    if (\task(_, _, _) := t)
      <env, _> = generate(t, env);
  }

  return 0;
}

public int testGeneration(loc source, loc output_dir)
{
  src = koda::Parser::parsekoda(source);
  ast = koda::CST2AST::cst2ast(src);

  return generate(ast, output_dir);
}
