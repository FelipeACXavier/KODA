module koda::IoTGenerator

import koda::AST;
import koda::Parser;
import koda::CST2AST;
import koda::Types;

import IO;
import List;
import String;

loc BASE_DIR = |project://koda/generated/cpp|;
loc INCLUDE_DIR = |project://koda/generated/include|;
loc SRC_OUTPUT_DIR = |project://koda/generated/source|;
str CLASS_NAME = "Bridge";

// ============================================================
// Helpers
str nameToMQTT(str name, str ttype, str mode)
{
  if (name != "action")
    return ttype;

  return "\"<clean(ttype)>/<mode>\"";
}

str argsToMQTT(str template, list[Argument] args)
{
  if (isEmpty(args))
    return "\"\"";

  for (arg <- args, \arg(str arg_type, str arg_id) := arg)
  {
    str extra = "";
    if (arg_type != "string")
      extra = "std::to_string(<arg_id>)";
    else
      extra = "<arg_id>";

    template = "\"{ \\\"cmd\\\": \\\"\" + <extra> + \"\\\" }\"";
  }

  return template;
}

str defaultForArgs(list[Argument] args)
{
  if (isEmpty(args))
    return "";

  str template = "";
  for (arg <- args, \arg(str arg_type, str arg_id) := arg)
  {
    if (arg_type == "string")
      template += "\"\", ";
    else if (arg_type == "boolean")
      template += "false, ";
    else
      template += "0, ";
  }

  return template[..-2];
}
// =============================================================
// Support files
public void generateCMakeLists(str packageName, list[str] deps)
{
  str output = trim("
cmake_minimum_required(VERSION 3.10)
project(<toLowerCase(packageName)> LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -g\")

find_package(PahoMqttCpp REQUIRED)

# Collect all .cc files
file(GLOB SRC_FILES src/*.cc)

# Add executable
add_executable(${PROJECT_NAME} src/main.cpp ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PahoMqttCpp::paho-mqttpp3)

# Add include directories so .hh files can be found
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/dzn
)
");

  loc filename = BASE_DIR + "CMakeLists.txt";
  touch(filename);
  writeFile(filename, output);
}

// ============================================================
// EventDefStatement
CallTpl generate(\event(str return_type, str id, list[Argument] args, list[EventDefComponent] _))
{
  println("event: <id> -\> <return_type>");
  return ctpl(id, args, return_type);
}

// ============================================================
// RosDefStatement
tuple[Env, CapabilityDef] generate(\trigger_block(EventDefStatement call), CapabilityDef cap, Env env)
{
  print("    trigger_block: ");
  cap.trigger = generate(call);
  return <env, cap>;
}

tuple[Env, CapabilityDef] generate(\return_block(EventDefStatement call), CapabilityDef cap, Env env)
{
  print("    return_block: ");
  cap.onReturn = generate(call);
  return <env, cap>;
}

tuple[Env, CapabilityDef] generate(\abort_block(EventDefStatement call), CapabilityDef cap, Env env)
{
  print("    abort_block: ");
  cap.onAbort = generate(call);
  return <env, cap>;
}

tuple[Env, CapabilityDef] generate(\error_block(EventDefStatement call), CapabilityDef cap, Env env)
{
  print("    error_block: ");
  cap.onError = generate(call);
  return <env, cap>;
}

tuple[Env, CapabilityDef] generate(\in_block(EventDefStatement call), CapabilityDef cap, Env env)
{
  print("    in_block: ");
  cap.trigger = generate(call);
  return <env, cap>;
}

tuple[Env, CapabilityDef] generate(\out_block(EventDefStatement call), CapabilityDef cap, Env env)
{
  print("    out_block: ");
  cap.onReturn = generate(call);
  return <env, cap>;
}

// ============================================================
// Statement
public tuple[Env, CapabilityDef] generate(\variables(list[VariableStatement] vars), Env env)
{
  println("  variables <vars>");
  return <env, empty_cdef()>;
}

public tuple[Env, CapabilityDef] generate(\action(str topic, str msg, list[RosDefStatement] events), Env env)
{
  println("  action <topic> <msg>");

  capDefinition = capDef("action", topic, msg, empty_ctpl(), empty_ctpl(), empty_ctpl(), empty_ctpl());
  for (event <- events) {
    <env, capDefinition> = generate(event, capDefinition, env);
  }

  return <env, capDefinition>;
}

public tuple[Env, CapabilityDef] generate(\service(str topic, str msg, list[RosDefStatement] events), Env env)
{
  println("  service <topic> <msg>");
  capDefinition = capDef("service", topic, msg, empty_ctpl(), empty_ctpl(), empty_ctpl(), empty_ctpl());
  for (event <- events) {
    <env, capDefinition> = generate(event, capDefinition, env);
  }
  return <env, capDefinition>;
}

public tuple[Env, CapabilityDef] generate(\topic(str topic, str msg, list[RosDefStatement] events), Env env)
{
  println("  topic <topic> <msg>");
  capDefinition = capDef("topic", topic, msg, empty_ctpl(), empty_ctpl(), empty_ctpl(), empty_ctpl());
  for (event <- events) {
    <env, _> = generate(event, capDefinition, env);
  }
  return <env, capDefinition>;
}

public tuple[Env, CapabilityDef] generate(\ros_def(RosDefStatement statement), Env env)
{
  println("  ros_def <statement>");
  capDefinition = capDef("action", "", "", empty_ctpl(), empty_ctpl(), empty_ctpl(), empty_ctpl());
  return generate(statement, capDefinition, env);
}

public tuple[Env, CapabilityDef] generate(\tasks_block(list[Flow] flows), Env env) {
  println("  tasks_block <flows>");
  return <env, empty_cdef()>;
}

// ============================================================
// Top Level
public bool generateCapability(str id, ServiceDef service)
{
  // Prepare methods
  str methodsDef = "";
  str methodsImpl = "";
  str callbacks = "";
  println("[IoTGenerator] Trying to generate ROS class: <id>");

  for (cap <- service.caps)
  {
    println("Generating capability: <cap.name> <cap.ttype> <cap.msg>");
    if (ctpl(str name, list[Argument] args, str ret) := cap.trigger)
    {
      println("  Trigger: <name> -\> <ret>");
      methodsDef += "  <ret> <id>_<name>(<argsToString(args)>) override;\n";

      methodsImpl += "<ret> <toComponent(id)>::<id>_<name>(<argsToString(args)>) {\n";
      methodsImpl += "  auto bridge = dzn_locator.get\<<CLASS_NAME>\>();\n\n";
      methodsImpl += "  std::cout \<\< \"Triggering: <id>_<name>\" \<\< std::endl;\n";
      methodsImpl += "  bridge.publish(<nameToMQTT(cap.name, cap.ttype, "trigger")>, <argsToMQTT(cap.msg, args)>);\n";
      methodsImpl += "}\n\n";
    }

    if (ctpl(str name, list[Argument] args, str ret) := cap.onAbort)
    {
      println("  Abort: <name> -\> <ret>");
      methodsDef += "  <ret> <id>_<name>(<argsToString(args)>) override;\n";

      methodsImpl += "<ret> <toComponent(id)>::<id>_<name>(<argsToString(args)>) {\n";
      methodsImpl += "  auto bridge = dzn_locator.get\<<CLASS_NAME>\>();\n\n";
      methodsImpl += "  std::cout \<\< \"Triggering: <id>_<name>\" \<\< std::endl;\n";
      methodsImpl += "  bridge.publish(<nameToMQTT(cap.name, cap.ttype, "abort")>, <argsToMQTT(cap.msg, args)>);\n";
      methodsImpl += "}\n\n";
    }

    if (ctpl(str name, list[Argument] args, str ret) := cap.onReturn)
    {
      println("  Return: <name> -\> <ret>");
      callbacks += "  bridge.register_handler(<nameToMQTT(cap.name, cap.ttype, "return")>, [this](const std::string& data) {\n";
      callbacks += "    std::cout \<\< \"Before <id>_<name>\" \<\< std::endl;\n";
      callbacks += "    <id>_<name>(<argsIdToString(args)>);\n";
      callbacks += "    return 0;\n";
      callbacks += "  });\n";
    }

    if (ctpl(str name, list[Argument] args, str ret) := cap.onError)
    {
      println("  Return: <name> -\> <ret>");
      callbacks += "  bridge.register_handler(<nameToMQTT(cap.name, cap.ttype, "error")>, [this](const std::string& data) {\n";
      callbacks += "    std::cout \<\< \"Before <id>_<name>\" \<\< std::endl;\n";
      callbacks += "    <id>_<name>(<argsIdToString(args)>);\n";
      callbacks += "    return 0;\n";
      callbacks += "  });\n";
    }
  }

  // Generate header
  str header = trim("
#pragma once

#include \<string\>

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
#include \<iostream\>
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
  auto bridge = dzn_locator.get\<<CLASS_NAME>\>();
<callbacks>
}

<methodsImpl>");

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
    if (key in env && ServiceDef service := env[key])
      generateCapability(uncapitalize(key), service);
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

#include \<mqtt/async_client.h\>
#include \"callback.hh\"

<includes>

class <CLASS_NAME>
{
public:
  <CLASS_NAME>(const std::string& name);

  void create();
  void start();
  void stop();
  void register_handler(const std::string& topic, std::function\<int(const std::string& message)\> handler);

  void publish(const std::string& topic, const std::string& msg);

private:
  // Generic ========================================================
  std::shared_ptr\<mqtt::async_client\> mClient;
  std::shared_ptr\<Callback\> mCallback;

  mqtt::connect_options mConnOpts;
};
");

  str source = trim("
#include \"<uncapitalize(CLASS_NAME)>.hh\"

const std::string SERVER_ADDRESS(\"localhost:1884\");
const std::string CLIENT_ID(\"iot-http-bridge\");

<CLASS_NAME>::<CLASS_NAME>(const std::string& name)
{
}

void <CLASS_NAME>::create()
{
  mClient = std::make_shared\<mqtt::async_client\>(SERVER_ADDRESS, CLIENT_ID);

  mConnOpts.set_keep_alive_interval(30);
  mConnOpts.set_clean_session(true);

  mCallback = std::make_shared\<Callback\>(mClient, mConnOpts);
}

void <CLASS_NAME>::start()
{
  mClient-\>set_callback(*mCallback);

  try
  {
    std::cout \<\< \"Connecting to the MQTT server \'\" \<\< SERVER_ADDRESS \<\< \"\'...\" \<\< std::flush;
    mClient-\>connect(mConnOpts, nullptr, *mCallback)-\>wait();
  } catch (const mqtt::exception& exc)
  {
    std::cerr \<\< \"ERROR: Unable to connect to MQTT server: \'\" \<\< SERVER_ADDRESS \<\< \"\'\" \<\< exc
              \<\< std::endl;
  }

  mCallback-\>bind_handlers();
}

void <CLASS_NAME>::stop()
{
  try
  {
    std::cout \<\< \"Disconnecting from the MQTT server...\" \<\< std::flush;
    mClient-\>disconnect()-\>wait();
    std::cout \<\< \"OK\" \<\< std::endl;
  } catch (const mqtt::exception& exc)
  {
    std::cerr \<\< exc \<\< std::endl;
  }
}

void <CLASS_NAME>::register_handler(const std::string& topic, std::function\<int(const std::string& message)\> handler)
{
  if (mCallback == nullptr)
  {
    std::cerr \<\< \"ERROR: start must be called before.\" \<\< std::endl;
    return;
  }

  mCallback-\>register_handler(topic, handler);
}

void <CLASS_NAME>::publish(const std::string& topic, const std::string& msg)
{
  if (mClient)
    mClient-\>publish(topic, msg);
}
");

  loc header_filename = INCLUDE_DIR + "<uncapitalize(CLASS_NAME)>.hh";
  loc source_filename = SRC_OUTPUT_DIR + "<uncapitalize(CLASS_NAME)>.cc";

  touch(header_filename);
  touch(source_filename);

  writeFile(header_filename, header);
  writeFile(source_filename, source);

  generateCMakeLists(taskId, deps);

  return true;
}

public bool generateMain(str taskId, map[str, str] capMap, CapabilityDef taskCap, Env env)
{
  str capabilities = "";
  for (key <- capMap)
    capabilities += "  system-\><capMap[key]>.start();\n";

  str callbacks = "";
  if (ctpl(str name, list[Argument] args, str _) := taskCap.trigger)
  {
    callbacks += "  <toLowerCase(CLASS_NAME)>.register_handler(\"<name>\", [&system](const std::string& data) {\n";
    callbacks += "    system-\>api.in.<name>(<defaultForArgs(args)>);\n";
    callbacks += "    std::cout \<\< \"Started\" \<\< std::endl;\n";
    callbacks += "    return 0;\n";
    callbacks += "  });\n";
  }

  if (ctpl(str name, list[Argument] args, str _) := taskCap.onAbort)
  {
    callbacks += "  <toLowerCase(CLASS_NAME)>.register_handler(\"<name>\", [&system](const std::string& data) {\n";
    callbacks += "    system-\>api.in.<name>(<defaultForArgs(args)>);\n";
    callbacks += "    std::cout \<\< \"Aborted\" \<\< std::endl;\n";
    callbacks += "    return 0;\n";
    callbacks += "  });\n";
  }

  str source = trim("
#include \<iostream\>
#include \<memory\>

#include \"<toComponent(taskId)>_task.hh\"
#include \"<uncapitalize(CLASS_NAME)>.hh\"

std::ostream nullstream(nullptr);
dzn::runtime runtime;
dzn::locator locator;

int main()
{
  <CLASS_NAME> <toLowerCase(CLASS_NAME)>(\"mqtt-bridge\");
  <toLowerCase(CLASS_NAME)>.create();

  auto system = std::make_unique\<<toComponent(taskId)>_task\>(locator.set(runtime).set(nullstream).set(bridge));

  system-\>api.out.failed = [] {
    std::cout \<\< \"Operation failed\" \<\< std::endl;
  };

<capabilities>

<callbacks>

  <toLowerCase(CLASS_NAME)>.start();
  std::cout \<\< \"Initialized\" \<\< std::endl;

  while (std::tolower(std::cin.get()) != \'q\');

  std::cout \<\< \"Done\" \<\< std::endl;
  system-\>api.in.abort();

  <toLowerCase(CLASS_NAME)>.stop();

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

  // Compile calls
  capDefinition = capDef("task", "", "", empty_ctpl(), empty_ctpl(), empty_ctpl(), empty_ctpl());
  for (s <- tstatements, \ros_def(RosDefStatement stat) := s)
  {
    <env, capDefinition> = generate(stat, capDefinition, env);
    println("Cap: <capDefinition>");
  }

  // Now, we generate the supervisor
  generateSupervisor(id, capMap, env);

  // Finally, we generate the main file
  generateMain(id, capMap, capDefinition, env);

  return <env, "">;
}

public Result generate(\capability(str id, list[Argument] args, list[Statement] tstatements), Env env) {
  println("Generating capability: <id>");

  // Get all the necessary capability info
  lenv = env;
  serv = serviceDef([]);
  for (s <- tstatements, \tasks_block(_) !:= s)
  {
    <lenv, cap> = generate(s, lenv);
    serv.caps += [cap];
  }

  env[id] = serv;

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
