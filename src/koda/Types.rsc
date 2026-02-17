module koda::Types

import koda::AST;

import IO;
import List;
import String;

// =============================================================
alias Env = map[str, value];
alias CEnv = map[str, TopLevelComponent];
alias TaskEnv = map[str, TaskDef];
alias Result = tuple[Env, value];

// =============================================================
data EventTpl
  = etpl(str name, list[str] args, str ret)
  | empty_etpl()
  ;

data EventCall
  = ecall(str name, list[str] args)
  ;

data TaskDef = taskDef(
  str name, list[str] formals, str ttype,
  EventTpl trigger,
  EventTpl onReturn, EventTpl onError, EventTpl onAbort,
  map[str,EventTpl] accepts, map[str,EventTpl] emits
);

// =============================================================
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
) | empty_cdef();

data ServiceDef = serviceDef(
  list[CapabilityDef] caps
);

data CapabilityData
  = capData(str includes, str members, str methods, str parameters, str constructor, str startUp,
            list[str] deps, list[str] packageDeps, list[str] configParam)
  | empty_data()
  ;

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
  {
    if (arg_type == "string")
      return "std::string <arg_id>";
    else if (arg_type == "boolean")
      return "bool <arg_id>";
    else
      return "<arg_type> <arg_id>";
  }

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