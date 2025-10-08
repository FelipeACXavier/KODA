module koda::ROSGenerator

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

data EventTpl = etpl(str name, list[str] args, str ret);
data TaskDef   = taskDef(
  str name,
  list[EventTpl] ins,
  list[EventTpl outs]);

loc OUTPUT_DIR = |project://koda/generated/|;
str CLASS_NAME = "Supervisor";

str clean(value val)
  = replaceAll("<val>", "\"", "");

// ======================================================================
// Message specific generators
// ======================================================================
public str generateString(str caller, str call, bool topublish, list[str] formals) {
  str output = "";
  if (topublish) {
    output = "message.data =<for (arg <- formals){> \"<arg>: \" + std::to_string(<arg>) +<}>";
    output = output[..-2] + ";\n";
  } else {
    output = "msg-\>data.erase(std::remove(msg-\>data.begin(), msg-\>data.end(), \' \'), msg-\>data.end());
    '<for (arg <- formals) {>  auto index<arg> =  msg-\>data.find(\"<arg>:\");\n<}>
    '  if (<for (int i <- [0..size(formals)]) {>index<formals[i]> == std::string::npos<if (i != size(formals) - 1){> || <}><}>)
    '  {
    '    if (<uncapitalize(caller)>_failed_callback) <uncapitalize(caller)>_failed_callback(dzn);
    '    return;
    '  }
    '<for (int i <- [0..size(formals)]) {><if (i != size(formals) - 1){>  auto <formals[i]> = msg-\>data.substr(index<formals[i]> + 2, index<formals[i + 1]> - 1 - (index<formals[i]> + 2));\n<}else{>  auto y = msg-\>data.substr(index<formals[i]> + 2, msg-\>data.size() - (index<formals[i]> + 2));<}><}>
    '
    '  if (<uncapitalize(caller)>_<call>) <uncapitalize(caller)>_<call>(<for (int i <- [0..size(formals)]) {>std::stof(<formals[i]>)<if (i != size(formals) - 1){>, <}><}>);
    '";
  }

  return output;
}

// ======================================================================
public Result generate(\var(str name), Env env)
{
  if (name in env)
    return <env, env[name]>;

  return <env, name>;
}

private Env publisherConstructor(str caller, str topic, str msg, Env env) {
  if (str con := env["supervisor_constructor"])
    env["supervisor_constructor"] = con + "<uncapitalize(caller)>_publisher_ = this-\>create_publisher\<<clean(msg)>\>(\"<clean(topic)>\", 10);\n";

  return env;
}

private Env subscriberConstructor(str caller, str topic, str msg, Env env) {
  if (str con := env["supervisor_constructor"])
    env["supervisor_constructor"] = con + "<uncapitalize(caller)>_subscription_ = this-\>create_subscription\<<clean(msg)>\>(\"<clean(topic)>\", 10, std::bind(&<CLASS_NAME>::<uncapitalize(caller)>_callback, this, std::placeholders::_1));\n";

  return env;
}

private Env actionConstructor(str caller, str topic, str msg, Env env) {
  if (str con := env["supervisor_constructor"])
    env["supervisor_constructor"] = con + "<uncapitalize(caller)>_action_ = rclcpp_action::create_client\<<clean(msg)>\>(this, \"<clean(topic)>\");\n";

  env["supervisor_members"] += "rclcpp_action::Client\<<clean(msg)>\>::SharedPtr <uncapitalize(caller)>_action_;\n";

  return env;
}

private Result generate(str caller, \ros_event(str comm, str topic, str msg), str call, list[str] formals, str inputType, Env env)
{
  str implementation = "";

  env["msg_type"] = clean(msg);

  if (comm == "topic") {
    // Constructor
    if (inputType == "trigger" || inputType == "abort") {
      env = publisherConstructor(caller, topic, msg, env);
    } else {
      env = subscriberConstructor(caller, topic, msg, env);
    }

    // Implementation
    if (inputType == "trigger" || inputType == "abort") {
      // Create message
      implementation += "auto message = <clean(msg)>();\n";
      // Message specific conversion
      if (clean(msg) == "std_msgs::msg::String")
        implementation += generateString(caller, call, true, formals);
      // Publish
      implementation += "<uncapitalize(caller)>_publisher_-\>publish(message);\n";
    } else {
      // Convert from ROS to internal type
      if (clean(msg) == "std_msgs::msg::String")
        implementation += generateString(caller, call, true, formals);
    }
  } else if (comm == "action") {
    if (topic notin ACTIONS) {
      env = actionConstructor(caller, topic, msg, env);
      println("Adding action: <topic>");
      ACTIONS[topic] = topic;
      env["supervisor_imports"] += "#include \<nav2_msgs/action/navigate_to_pose.hpp\>\n";
    }

    if (inputType == "trigger") {
      implementation += "if (!action_client_-\>wait_for_action_server(10s)) {\n";
      implementation += "  RCLCPP_ERROR(get_logger(), \"Nav2 action server not available after 10s.\");\n";
      implementation += "  return;\n";
      implementation += "}\n";

      // Convert arguments to message
      implementation += "<clean(msg)> goal_msg;\n";
      
      // Message specific stuff
      implementation += "goal_msg.pose.header.stamp = std::chrono_literals::now();\n";
      implementation += "goal_msg.pose.header.frame_id = \"map\";\n";

      implementation += "goal_msg.pose.pose.position.x = 1.0;\n";
      implementation += "goal_msg.pose.pose.position.y = 0.0;\n";
      implementation += "goal_msg.pose.pose.position.z = 0.0;\n";
      implementation += "goal_msg.pose.pose.orientation = yawToQuat(0.0);\n";

      //  RCLCPP_INFO(get_logger(), "Sending NavigateToPose goal (%.2f, %.2f)", goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

      implementation += "auto send_goal_options = rclcpp_action::Client\<<clean(msg)>\>::SendGoalOptions();\n";
      implementation += "send_goal_options.result_callback =\n";
      implementation += "  [this](const GoalHandleNav::WrappedResult &result) {\n";
      implementation += "    switch (result.code) {\n";
      implementation += "      case rclcpp_action::ResultCode::SUCCEEDED:\n";
      implementation += "        RCLCPP_INFO(this-\>get_logger(), \"Goal reached!\");\n";
      implementation += "        <uncapitalize(caller)>_callback();\n";
      implementation += "        break;\n";
      implementation += "      case rclcpp_action::ResultCode::ABORTED:\n";
      implementation += "        RCLCPP_ERROR(this-\>get_logger(), \"Goal was aborted\");\n";
      implementation += "        <uncapitalize(caller)>_error_callback();\n";
      implementation += "        break;\n";
      implementation += "      case rclcpp_action::ResultCode::CANCELED:\n";
      implementation += "        RCLCPP_WARN(this-\>get_logger(), \"Goal was canceled\");\n";
      implementation += "        break;\n";
      implementation += "      default:\n";
      implementation += "        RCLCPP_ERROR(this-\>get_logger(), \"Unknown result code\");\n";
      implementation += "        break;\n";
      implementation += "    }\n";
      implementation += "    rclcpp::shutdown();\n";
      implementation += "  };\n";
      implementation += "goal_future_ = action_client_-\>async_send_goal(goal_msg, send_goal_options);\n";
      
      env["supervisor_members"] += "rclcpp_action::Client\<<clean(msg)>\>::SharedFuture goal_future_;\n";

    } else if (inputType == "abort") {
      implementation += "if (!goal_future_.valid()) return;\n";
      implementation += "auto gh = goal_future_.get();\n";
      implementation += "if (!gh) return;\n";
      implementation += "RCLCPP_WARN(this-\>get_logger(), \"Cancelling goal...\");\n";
      implementation += "action_client_-\>async_cancel_goal(gh, [this](auto) { RCLCPP_INFO(this-\>get_logger(), \"Cancel request sent.\"); });\n";
    } else if (inputType == "return") {
      implementation += "if (<uncapitalize(caller)>_<call>)\n";
      implementation += "  <uncapitalize(caller)>_<call>();\n";
    } else if (inputType == "error") {
      implementation += "if (<uncapitalize(caller)>_<call>)\n";
      implementation += "  <uncapitalize(caller)>_<call>();\n";
    }
  }

  return <env, implementation>;
}

public Result generate(str caller, \trigger_block(EventDefStatement call), Env env)
{
  // println("\\trigger_block(<call>)");
  if (\event(str ret, str id, list[Argument] arguments, list[EventDefComponent] components) := call) {
    str args = "";
    list[str] formals = [];
    for (arg <- arguments) {
      if (\arg(str arg_type, str arg_id) := arg) {
        args += "<arg_type> <arg_id>, ";
        formals += arg_id;
      }
    }

    list[str] implementation = [];
    for (c <- components) {
      <env, a> = generate(caller, c, id, formals, "trigger", env);
      if (str b := a, !isEmpty(b))
        implementation += b;
    }

    if (str def := env["supervisor_definitions"])
      env["supervisor_definitions"] = def + "<ret> <uncapitalize(caller)>_<id>(<args[..-2]>);\n";
    
    if (str def := env["definitions"])
      env["definitions"] = def + "<ret> <uncapitalize(caller)>_<id>(<args[..-2]>) override;\n";

    if (str imp := env["supervisor_implementations"])
      env["supervisor_implementations"] = imp + "
      '<ret> <CLASS_NAME>::<uncapitalize(caller)>_<id>(<args[..-2]>) {
      '<for (a <- implementation){>  <a><}>}\n";
  }

  return <env, "">;
}

public Result generate(str caller, \abort_block(EventDefStatement call), Env env)
{
  // println("\\abort_block(<call>)");
  if (\event(str ret, str id, list[Argument] arguments, list[EventDefComponent] components) := call) {
    str args = "";
    list[str] formals = [];
    for (arg <- arguments) {
      if (\arg(str arg_type, str arg_id) := arg) {
        args += "<arg_type> <arg_id>, ";
        formals += arg_id;
      }
    }

    list[str] implementation = [];
    for (c <- components) {
      <env, a> = generate(caller, c, id, formals, "abort", env);
      if (str b := a, !isEmpty(b))
        implementation += b;
    }

    if (str def := env["supervisor_definitions"])
      env["supervisor_definitions"] = def + "<ret> <uncapitalize(caller)>_<id>(<args[..-2]>);\n";

    if (str def := env["definitions"])
      env["definitions"] = def + "<ret> <uncapitalize(caller)>_<id>(<args[..-2]>) override;\n";

    if (str imp := env["supervisor_implementations"])
      env["supervisor_implementations"] = imp + "
      '<ret> <CLASS_NAME>::<uncapitalize(caller)>_<id>(<args[..-2]>) {
      '<for (a <- implementation){>  <a><}>}\n";
  }

  return <env, "">;
}

public Result generate(str caller, \return_block(EventDefStatement call), Env env)
{
  // println("\\return_block(<call>)");
  if (\event(str ret, str id, list[Argument] arguments, list[EventDefComponent] components) := call) {
    str args = "";
    list[str] formals = [];
    for (arg <- arguments) {
      if (\arg(str arg_type, str arg_id) := arg) {
        args += "<arg_type> <arg_id>, ";
        formals += arg_id;
      }
    }

    list[str] implementation = [];
    for (c <- components) {
      <env, a> = generate(caller, c, id, formals, "return", env);
      if (str b := a, !isEmpty(b))
        implementation += b;
    }

    env["supervisor_definitions"] += "std::function\<<ret>(<args[..-2]>)\> <uncapitalize(caller)>_<id>;\n";
    // if (!isEmpty(implementation))
    env["supervisor_definitions"] += "void <uncapitalize(caller)>_callback(const <env["msg_type"]>::SharedPtr msg);\n";

    if (str imp := env["supervisor_implementations"]) {
      env["supervisor_implementations"] = imp + "
      '<ret> <CLASS_NAME>::<uncapitalize(caller)>_callback(const <env["msg_type"]>::SharedPtr msg) {
      '<for (a <- implementation){>  <a><}>}\n";
    }
  }

  return <env, "">;
}

public Result generate(str caller, \error_block(EventDefStatement call), Env env)
{
  // println("\\error_block(<call>)");
  if (\event(str ret, str id, list[Argument] arguments, list[EventDefComponent] components) := call) {
    str args = "";
    list[str] formals = [];
    for (arg <- arguments) {
      if (\arg(str arg_type, str arg_id) := arg) {
        args += "<arg_type> <arg_id>, ";
        formals += arg_id;
      }
    }

    list[str] implementation = [];
    for (c <- components) {
      <env, a> = generate(caller, c, id, formals, "error", env);
      if (str b := a, !isEmpty(b))
        implementation += b;
    }

    env["supervisor_definitions"] += "std::function\<<ret>(<args[..-2]>)\> <uncapitalize(caller)>_<id>;\n";
    // if (!isEmpty(implementation))
    env["supervisor_definitions"] += "void <uncapitalize(caller)>_error_callback(const <env["msg_type"]>::SharedPtr msg);\n";

    if (str imp := env["supervisor_implementations"]) {
      env["supervisor_implementations"] = imp + "
      '<ret> <CLASS_NAME>::<uncapitalize(caller)>_error_callback(const <env["msg_type"]>::SharedPtr msg) {
      '<for (a <- implementation){>  <a><}>}\n";
    }
  }
  return <env, "">;
}

public Result generate(\variables(list[VariableStatement] vars), Env env) {
  list[tuple[str, str, str, str]] variables = [];
  for (v <- vars) {
    if (\variable_def(str var_type, str var_name, Expression exp, Expression base) := v) {
      <_, a> = generate(exp, env);
      <_, b> = generate(base, env);
      variables += <var_type, var_name, a, b>;
    }
  }

  env["vars"] = variables;

  return <env, "">;
}

// Capabilities
public Result generate(str caller, \accepts(list[EventDefStatement] events), Env env)
{
  for (event <- events) {
    if (\event(str ret, str id, list[Argument] arguments, list[EventDefComponent] _) := event) {
      str args = "";
      for (arg <- arguments) {
        if (\arg(str arg_type, str arg_id) := arg)
          args += "<arg_type> <arg_id>, ";
      }

      if (str def := env["supervisor_definitions"])
        env["supervisor_definitions"] = def + "<ret> <uncapitalize(caller)>_<id>(<args[..-2]>);\n";

      if (str def := env["definitions"])
        env["definitions"] = def + "<ret> <uncapitalize(caller)>_<id>(<args[..-2]>);\n";

      if (str imp := env["supervisor_implementations"])
        env["supervisor_implementations"] = imp + "<ret> <CLASS_NAME>::<uncapitalize(caller)>_<id>(<args[..-2]>) {}\n";
    }
  }

  return <env, "">;
}

public Result generate(str caller, \emits(list[EventDefStatement] events), Env env)
{
  for (event <- events) {
    if (\event(str ret, str id, list[Argument] arguments, list[EventDefComponent] _) := event) {
      str args = "";
      for (arg <- arguments) {
        if (\arg(str arg_type, str arg_id) := arg)
          args += "<arg_type> <arg_id>, ";
      }

      if (str def := env["supervisor_definitions"])
        env["supervisor_definitions"] = def + "std::function\<<ret>(<args[..-2]>)\> <uncapitalize(caller)>_<id>;\n";

      if (str def := env["definitions"])
        env["definitions"] = def + "std::function\<<ret>(<args[..-2]>)\> <uncapitalize(caller)>_<id>;\n";
    }
  }

  return <env, "">;
}

public Result generate(\task(str id, list[Argument] args, list[Statement] tstatements, bool iscomponent, str ttype), Env env) {
  println("Generating ROS methods for: <id>");
  // println("\\task(<id>, <args>, <tstatements>)");

  env["supervisor_definitions"]     += "// <id> definitions ===============================\n";
  env["supervisor_members"]         += "// <id> members ===================================\n";
  env["supervisor_constructor"]     += "// <id> constructor ===============================\n";
  env["supervisor_implementations"] += "// <id> implementations ===========================\n";

  env["definitions"] = "";
  env["members"] = "";
  env["constructor"] = "";
  env["implementations"] = "";

  // TaskDef task = taskDef(id, [], []);

  for (s <- tstatements) {
    <env, _> = generate(id, s, env);
  }

  // Write supervisor header file
  str header = trim("
#pragma once

#include \<a_<uncapitalize(id)>.h\>

#include \"supervisor.hh\";

class <uncapitalize(id)> : public skel::c<uncapitalize(id)>
{
public:
  <uncapitalize(id)>(dzn::locator const& locator);
  ~<uncapitalize(id)>();

  void set_supervisor(std::shared_ptr\<<CLASS_NAME>\> <uncapitalize(CLASS_NAME)>);

  <env["definitions"]>
private:
  std::shared_ptr\<<CLASS_NAME>\> m<CLASS_NAME>;
};");


  // Write supervisor source file
  str source = trim("
#include \"<uncapitalize(id)>.hh\"

<uncapitalize(id)>::<uncapitalize(id)>()
    : skel::c<uncapitalize(id)>(locator)
{
}

void cdrive::set_supervisor(std::shared_ptr\<<CLASS_NAME>\> <uncapitalize(CLASS_NAME)>)
{
  m<CLASS_NAME> = <uncapitalize(CLASS_NAME)>;

  m<CLASS_NAME>-\>drive_in_position = [this](float x, float y) { drive_in_position(x, y); };
  m<CLASS_NAME>-\>drive_path_blocked = [this] { drive_path_blocked(); };
}

<env["supervisor_implementations"]>
");

  loc header_filename = OUTPUT_DIR + "<uncapitalize(id)>.hh";
  loc source_filename = OUTPUT_DIR + "<uncapitalize(id)>.cc";

  touch(header_filename);
  touch(source_filename);

  writeFile(header_filename, header);
  writeFile(source_filename, source);

  env["definitions"] = "";
  env["members"] = "";
  env["constructor"] = "";
  env["implementations"] = "";

  return <env, "">;
}

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
