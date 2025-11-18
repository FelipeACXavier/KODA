module koda::CST2AST

import IO;
import Prelude;
import String;
import ParseTree;

import koda::AST;
import koda::Syntax;

/*
 * Implement a mapping from concrete syntax trees (CSTs) to abstract syntax trees (ASTs)
 * Hint: Use switch to do case distinction with concrete patterns
 * Map regular CST arguments (e.g., *, +, ?) to lists
 * Map lexical nodes to Rascal primitive types (bool, int, str)
 */
// ============================================================
// Basic
str loadBasic(Ident v) = "<v>";
str loadBasic(String v) = "<v>";
int loadBasic(Natural v) = toInt("<v>");
real loadBasic(Real v) = toReal("<v>");

str loadBasic(Time v) = "<v>";
str loadBasic(When v) = "<v>";
str loadBasic(ROSData v) = "<v>";
// str loadBasic(TaskType v) = "<v>";
// str loadBasic(TaskEnd v) = "<v>";

// ============================================================
// Argument
Argument loadArgument((Argument)`<Ident arg_type> <Ident arg_name>`)
  = \arg(loadBasic(arg_type), loadBasic(arg_name), src=arg_type@\loc);

Argument loadArgument((Argument)`<Ident arg_name> req <Ident arg_type>`)
  = \requires(loadBasic(arg_type), loadBasic(arg_name), src=arg_name@\loc);

Argument loadArgument((Argument)`<Ident arg_name> pro <Ident arg_type>`)
  = \provides(loadBasic(arg_type), loadBasic(arg_name), src=arg_name@\loc);

// ============================================================
// Expression
Expression loadExpression((Expression)`<Ident id>`)
  = \var(loadBasic(id), src=id@\loc);

Expression loadExpression((Expression)`<String svalue>`)
  = \str_const(loadBasic(svalue), src=svalue@\loc);

Expression loadExpression((Expression)`<Natural ivalue>`)
  = \int_const(loadBasic(ivalue));

Expression loadExpression((Expression)`<Real fvalue>`)
  = \float_const(loadBasic(fvalue));

// Event call
Expression loadExpression((Expression)`<EventStatement call>`)
  = \call_expr(loadEventStatement(call));

// Basic math operators
Expression loadExpression((Expression)`- <Expression arg>`)
  = \neg(loadExpression(arg));

Expression loadExpression((Expression)`<Expression lhs> * <Expression rhs>`)
  = \mul(loadExpression(lhs), loadExpression(rhs));

Expression loadExpression((Expression)`<Expression lhs> / <Expression rhs>`)
  = \div(loadExpression(lhs), loadExpression(rhs));

Expression loadExpression((Expression)`<Expression lhs> + <Expression rhs>`)
  = \add(loadExpression(lhs), loadExpression(rhs));

Expression loadExpression((Expression)`<Expression lhs> - <Expression rhs>`)
  = \sub(loadExpression(lhs), loadExpression(rhs));

// Basic comparison operators
Expression loadExpression((Expression)`! <Expression lhs>`)
  = \not(loadExpression(lhs));

Expression loadExpression((Expression)`<Expression lhs> == <Expression rhs>`)
  = \eq(loadExpression(lhs), loadExpression(rhs));

Expression loadExpression((Expression)`<Expression lhs> != <Expression rhs>`)
  = \neq(loadExpression(lhs), loadExpression(rhs));

Expression loadExpression((Expression)`<Expression lhs> \<= <Expression rhs>`)
  = \leq(loadExpression(lhs), loadExpression(rhs));

Expression loadExpression((Expression)`<Expression lhs> \>= <Expression rhs>`)
  = \geq(loadExpression(lhs), loadExpression(rhs));

Expression loadExpression((Expression)`<Expression lhs> \< <Expression rhs>`)
  = \lt(loadExpression(lhs), loadExpression(rhs));

Expression loadExpression((Expression)`<Expression lhs> \> <Expression rhs>`)
  = \gt(loadExpression(lhs), loadExpression(rhs));

// Basic logic operators
Expression loadExpression((Expression)`<Expression lhs> and <Expression rhs>`)
  = \conj(loadExpression(lhs), loadExpression(rhs));

Expression loadExpression((Expression)`<Expression lhs> or <Expression rhs>`)
  = \disj(loadExpression(lhs), loadExpression(rhs));

Expression loadExpression((Expression)`( <Expression lhs> )`)
  = loadExpression(lhs);

// ============================================================
// VariableStatement
VariableStatement loadVariableStatement((VariableStatement)`<Ident var_type> <Ident var_name> = <Expression expr> : <Expression base>`)
  = \variable_def(loadBasic(var_type), loadBasic(var_name), loadExpression(expr), loadExpression(base), src=var_type@\loc);

// ============================================================
// EventStatement
EventStatement loadEventStatement((EventStatement)`<Ident obj> . <Ident event> ( <{Expression ","}* args>)`)
  = \event_call(loadBasic(obj), loadBasic(event), [loadExpression(arg) | arg <- args], src=obj@\loc);

EventStatement loadEventStatement((EventStatement)`<Ident event> ( <{Expression ","}* args>)`)
  = \task_call(loadBasic(event), [loadExpression(arg) | arg <- args], src=event@\loc);

// ============================================================
// Flow
Flow loadFlow((Flow)`<Ident id> : <Strategy strategy> ;`)
  = \flow(loadBasic(id), [], loadStrategy(strategy), src=id@\loc);

Flow loadFlow((Flow)`<Ident id> [<{Ident ","}* args>] : <Strategy strategy> ;`)
  = \flow(loadBasic(id), [loadBasic(arg) | arg <- args], loadStrategy(strategy), src=id@\loc);

// ============================================================
// Strategy handlers
StrategyHandler loadStrategyHandler((StrategyHandler)`on error <Strategy a>`)
  = \error(loadStrategy(a), src=a@\loc);

StrategyHandler loadStrategyHandler((StrategyHandler)`on abort <Strategy a>`)
  = \abort(loadStrategy(a), src=a@\loc);

StrategyHandler loadStrategyHandler((StrategyHandler)`on <EventStatement call> <Strategy a>`)
  = \emitter(loadEventStatement(call), loadStrategy(a), src=a@\loc);

// ============================================================
// Strategy
Strategy loadStrategy((Strategy)`<EventStatement call> <StrategyHandler* handlers>`)
  = \task(loadEventStatement(call), [loadStrategyHandler(h) | h <- handlers], src=call@\loc);

Strategy loadStrategy((Strategy)`<Strategy a> --\> <Strategy b>`)
  = \seq(loadStrategy(a), loadStrategy(b), src=a@\loc);

Strategy loadStrategy((Strategy)`end`)
  = \end();

Strategy loadStrategy((Strategy)`repeat ( <Strategy a> )`)
  = \repeat(loadStrategy(a), src=a@\loc);

Strategy loadStrategy((Strategy)`join ( <{Strategy "|"}+ as> )`)
  = \par([loadStrategy(a) | a <- as], true, src=as[0]@\loc);

Strategy loadStrategy((Strategy)`either ( <{Strategy "|"}+ as> )`)
  = \par([loadStrategy(a) | a <- as], false, src=as[0]@\loc);

Strategy loadStrategy((Strategy)`let <Ident port> = <EventStatement a>`)
  = \let(loadBasic(port), loadEventStatement(a), src=port@\loc);

Strategy loadStrategy((Strategy)`if <Expression expr> then <Strategy a>`)
  = \if_strat(loadExpression(expr), loadStrategy(a));

Strategy loadStrategy((Strategy)`if <Expression expr> then <Strategy a> else <Strategy b>`)
  = \ifelse_strat(loadExpression(expr), loadStrategy(a), loadStrategy(b));

Strategy loadStrategy((Strategy)`within <Natural expr> do <Strategy a> else <Strategy b>`)
  = \within(loadBasic(expr), loadStrategy(a), loadStrategy(b), src=expr@\loc);

Strategy loadStrategy((Strategy)`guard { <Expression expr> }`)
  = \guard(loadExpression(expr), src=expr@\loc);

Strategy loadStrategy((Strategy)`every <Natural expr> { <Strategy a> } <StrategyHandler* handlers>`)
  = \every(loadBasic(expr), loadStrategy(a), [loadStrategyHandler(h) | h <- handlers], src=expr@\loc);

Strategy loadStrategy((Strategy)`<Ident s>`)
  = \ref(loadBasic(s), src=s@\loc);

Strategy loadStrategy((Strategy)`( <Strategy a> )`)
  = loadStrategy(a);

// ============================================================
// RosDefStatement
RosDefStatement loadRosDefStatement((RosDefStatement)`trigger: <EventDefStatement call> ;`)
  = \trigger_block(loadEventDefStatement(call), src=call@\loc);

RosDefStatement loadRosDefStatement((RosDefStatement)`return: <EventDefStatement call> ;`)
  = \return_block(loadEventDefStatement(call), src=call@\loc);

RosDefStatement loadRosDefStatement((RosDefStatement)`abort: <EventDefStatement call> ;`)
  = \abort_block(loadEventDefStatement(call), src=call@\loc);

RosDefStatement loadRosDefStatement((RosDefStatement)`error: <EventDefStatement call> ;`)
  = \error_block(loadEventDefStatement(call), src=call@\loc);

RosDefStatement loadRosDefStatement((RosDefStatement)`in: <EventDefStatement call> ;`)
  = \in_block(loadEventDefStatement(call), src=call@\loc);

RosDefStatement loadRosDefStatement((RosDefStatement)`out: <EventDefStatement call> ;`)
  = \out_block(loadEventDefStatement(call), src=call@\loc);

// Capability
EventDefStatement loadEventDefStatement((EventDefStatement)`<Ident return_type> <Ident id> ( <{Argument ","}* args> ) : <{EventDefComponent ","}+ components>`)
  = \event(loadBasic(return_type), loadBasic(id), [loadArgument(arg) | arg <- args], [loadEventDefComponent(c) | c <- components], src=return_type@\loc);

EventDefStatement loadEventDefStatement((EventDefStatement)`<Ident return_type> <Ident id> ( <{Argument ","}* args> )`)
  = \event(loadBasic(return_type), loadBasic(id), [loadArgument(arg) | arg <- args], [], src=return_type@\loc);

// ============================================================
// Statement
// Task
Statement loadStatement((Statement)`strategy { <Flow+ flows> }`)
  = \tasks_block([loadFlow(f) | f <- flows], src=flows[0]@\loc);

Statement loadStatement((Statement)`vars { <VariableStatement+ vars> }`)
  = \variables([loadVariableStatement(v) | v <- vars], src=vars[0]@\loc);

Statement loadStatement((Statement)`action <String id> <String msg> { <RosDefStatement* stats> }`)
  = \action(loadBasic(id), loadBasic(msg), [loadRosDefStatement(stat) | stat <- stats]);

Statement loadStatement((Statement)`service <String id> <String msg> { <RosDefStatement* stats> }`)
  = \service(loadBasic(id), loadBasic(msg), [loadRosDefStatement(stat) | stat <- stats]);

Statement loadStatement((Statement)`topic <String id> <String msg> { <RosDefStatement* stats> }`)
  = \topic(loadBasic(id), loadBasic(msg), [loadRosDefStatement(stat) | stat <- stats]);

Statement loadStatement((Statement)`<RosDefStatement stat>`)
  = \ros_def(loadRosDefStatement(stat));

// ============================================================
// EventDefComponent
EventDefComponent loadEventDefComponent((EventDefComponent)`<ROSData comm> : <String id> <String msg>`)
  = \ros_event(loadBasic(comm), loadBasic(id), loadBasic(msg));

EventDefComponent loadEventDefComponent((EventDefComponent)`timeout <Natural time> <Time unit> -\> <Ident call>`)
  = \timeout(loadBasic(time), loadBasic(unit), loadBasic(call));

EventDefComponent loadEventDefComponent((EventDefComponent)`allowed in <When val>`)
  = \when(loadBasic(val));

EventDefComponent loadEventDefComponent((EventDefComponent)`reply <Ident var> <When val>`)
  = \reply(loadBasic(var), loadBasic(val));

EventDefComponent loadEventDefComponent((EventDefComponent)`after <Ident var>`)
  = \depends(loadBasic(var));

EventDefComponent loadEventDefComponent((EventDefComponent)`once in <When val>`)
  = \oncein(loadBasic(val), []);

EventDefComponent loadEventDefComponent((EventDefComponent)`once in <When val> { <Statement* statements> }`)
  = \oncein(loadBasic(val), [loadStatement(statement) | statement <- statements]);

EventDefComponent loadEventDefComponent((EventDefComponent)`trigger`)
  = \start([]);

EventDefComponent loadEventDefComponent((EventDefComponent)`trigger { <Statement* statements> }`)
  = \start([loadStatement(statement) | statement <- statements]);

EventDefComponent loadEventDefComponent((EventDefComponent)`abort`)
  = \reset([]);

EventDefComponent loadEventDefComponent((EventDefComponent)`abort { <Statement* statements> }`)
  = \reset([loadStatement(statement) | statement <- statements]);

// ============================================================
// TopLevelComponent
TopLevelComponent loadTopLevel((TopLevelComponent)`capability <Ident id> ( <{Argument ","}* args> ) { <Statement* statements> }`)
  = \capability(loadBasic(id), [loadArgument(arg) | arg <- args], [loadStatement(statement) | statement <- statements], src=id@\loc);

TopLevelComponent loadTopLevel((TopLevelComponent)`task <Ident id> ( <{Argument ","}* args> ) { <Statement* statements> }`)
  = \task(loadBasic(id), [loadArgument(arg) | arg <- args], [loadStatement(statement) | statement <- statements], src=id@\loc);

// ============================================================
// Starting point
public koda::AST::System cst2ast((start[System])`<TopLevelComponent* components>`)
  = \system([loadTopLevel(component) | component <- components]);
