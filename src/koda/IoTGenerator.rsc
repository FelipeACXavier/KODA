module koda::IoTGenerator

import koda::AST;
import koda::Parser;
import koda::CST2AST;

import IO;
import List;
import String;

alias Env = map[str, str];
alias Result = tuple[Env, value];

Env genv = ();
Env ACTIONS = ();

loc OUTPUT_DIR = |project://koda/generated/|;
str CLASS_NAME = "Supervisor";

public int generate(koda::AST::System system)
{
  // The ROS component only needs the methods of the different capabilities
  Env env = ();
  env["supervisor_imports"]     = "";
  env["supervisor_definitions"]     = "";
  env["supervisor_members"]         = "";
  env["supervisor_constructor"]     = "";
  env["supervisor_implementations"] = "";

  for (t <- system.components) {
    if (\task(_, _, _, false, _) := t)
      <env, _> = generate(t, env);
  }

  println(env["supervisor_definitions"]);
  println(env["supervisor_implementations"]);
  println(env["supervisor_members"]);
  println(env["supervisor_constructor"]);

  // Write supervisor header file
  str header = trim("
#pragma once

#include \<chrono\>
#include \<functional\>
#include \<memory\>
#include \<string\>

#include \"rclcpp/rclcpp.hpp\"
#include \<rclcpp_action/rclcpp_action.hpp\>

<env["supervisor_imports"]>

class Supervisor : public rclcpp::Node
{
public:
  Supervisor();

  <env["supervisor_definitions"]>

private:
  <env["supervisor_members"]>
};");


  // Write supervisor source file
  str source = trim("
#include \"supervisor.hh\"

#include \<algorithm\>
#include \<chrono\>

Supervisor::Supervisor()
    : Node(\"supervisor\")
{
  <env["supervisor_constructor"]>
}

geometry_msgs::msg::Quaternion yawToQuat(double yaw) {
  geometry_msgs::msg::Quaternion q;
  // roll = pitch = 0
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  q.x = 0.0;
  q.y = 0.0;
  q.z = sy;
  q.w = cy;
  return q;
}

<env["supervisor_implementations"]>
");

  loc header_filename = OUTPUT_DIR + "supervisor.hh";
  loc source_filename = OUTPUT_DIR + "supervisor.cc";

  touch(header_filename);
  touch(source_filename);

  writeFile(header_filename, header);
  writeFile(source_filename, source);

  return 0;
}

public int testGeneration(loc source, loc output_dir)
{
  src = koda::Parser::parsekoda(source);
  ast = koda::CST2AST::cst2ast(src);

  OUTPUT_DIR = output_dir;

  return generate(ast);
}
