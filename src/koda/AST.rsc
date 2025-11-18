module koda::AST

import koda::Syntax;

data System(loc src=|unknown:///|)
  = \system(list[TopLevelComponent] components)
  ;

data TopLevelComponent(loc src=|unknown:///|)
  = \task(str id, list[Argument] args, list[Statement] statements)
  | \capability(str id, list[Argument] args, list[Statement] statements)
  ;

data Argument(loc src=|unknown:///|)
  = \arg(str arg_type, str arg_id)
  | \provides(str arg_type, str arg_id)
  | \requires(str arg_type, str arg_id)
  ;

data Flow(loc src=|unknown:///|)
  = \flow(str id, list[str] arguments, Strategy strategy)
  ;

data EventStatement(loc src=|unknown:///|)
  = \event_call(str obj, str event, list[Expression] arguments)
  | \task_call(str event, list[Expression] arguments)
  ;

data StrategyHandler(loc src=|unknown:///|)
  = \error(Strategy a)
  | \abort(Strategy a)
  | \emitter(EventStatement call, Strategy a)
  ;

data Strategy(loc src=|unknown:///|)
  = \task(EventStatement call, list[StrategyHandler] handlers)
  | \seq(Strategy a, Strategy b)
  | \par(list[Strategy] as, bool joint)
  | \let(str port, EventStatement call)
  | \if_strat(Expression condition, Strategy a)
  | \ifelse_strat(Expression condition, Strategy a, Strategy b)
  | \within(int timer, Strategy a, Strategy b)
  | \guard(Expression expr)
  | \every(int timer, Strategy a, list[StrategyHandler] handlers)
  | \repeat(Strategy a)
  | \ref(str id)
  | \end()
  ;

data EventDefStatement(loc src=|unknown:///|)
  = \event(str return_type, str id, list[Argument] args, list[EventDefComponent] components)
  ;

data RosDefStatement(loc src=|unknown:///|)
  = \trigger_block(EventDefStatement call)
  | \return_block(EventDefStatement call)
  | \abort_block(EventDefStatement call)
  | \error_block(EventDefStatement call)
  | \in_block(EventDefStatement call)
  | \out_block(EventDefStatement call)
  ;

data VariableStatement(loc src=|unknown:///|)
  = \variable_def(str var_type, str var_name, Expression exp, Expression base)
  ;

data Statement(loc src=|unknown:///|)
  = \tasks_block(list[Flow] flows)
  | \variables(list[VariableStatement] stats)
  | \action(str topic, str msg, list[RosDefStatement] events)
  | \service(str topic, str msg, list[RosDefStatement] events)
  | \topic(str topic, str msg, list[RosDefStatement] events)
  | \ros_def(RosDefStatement def)
  ;

data EventDefComponent(loc src=|unknown:///|)
  = \ros_event(str comm, str topic, str msg)
  | \timeout(int time, str unit, str event)
  | \when(str when)
  | \reply(str var, str when)
  | \depends(str var)
  | \oncein(str when, list[Statement] assign)
  | \start(list[Statement] statements)
  | \reset(list[Statement] assign)
  ;

data Expression(loc src=|unknown:///|)
  = \var(str name)
  | \str_const(str svalue)
  | \int_const(int ivalue)
  | \float_const(real fvalue)

  | \call_expr(EventStatement call)

  | \neg(Expression arg)
  | \mul(Expression lhs, Expression rhs)
  | \div(Expression lhs, Expression rhs)
  | \add(Expression lhs, Expression rhs)
  | \sub(Expression lhs, Expression rhs)

  | \not(Expression lhs)
  | \eq(Expression lhs, Expression rhs)
  | \neq(Expression lhs, Expression rhs)
  | \leq(Expression lhs, Expression rhs)
  | \geq(Expression lhs, Expression rhs)
  | \lt(Expression lhs, Expression rhs)
  | \gt(Expression lhs, Expression rhs)

  | \conj(Expression lhs, Expression rhs)
  | \disj(Expression lhs, Expression rhs)
  ;
