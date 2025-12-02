module koda::Generator

import koda::AST;
import koda::Parser;
import koda::CST2AST;
import koda::Automaton;
import koda::ROSGenerator;

import IO;
import Map;
import Set;
import List;
import String;
import Boolean;

// =============================================================
// Data formats
data TaskData
  = \task_data(str trigger, list[str] trigger_call,
               str success, list[str] success_call,
               str abort, list[str] abort_call,
               str error, list[str] error_call);

data EventTpl  = etpl(str name, list[str] args, str ret) | empty_etpl();
data EventCall = ecall(str name, list[str] args);
data TaskDef   = taskDef(
  str name, list[str] formals, str ttype,
  EventTpl trigger,
  EventTpl onReturn, EventTpl onError, EventTpl onAbort,
  map[str,EventTpl] accepts, map[str,EventTpl] emits
);

alias Env = map[str, value];
alias CEnv = map[str, TopLevelComponent];
alias TaskEnv = map[str, TaskDef];
alias Result = tuple[Env, value];

// =============================================================
// Globals
Env genv = ();
CEnv cenv = ();
TaskEnv tenv = ();

loc OUTPUT_DIR = |project://koda/generated/|;
str EXTERNALS = "types.dzn";

str INCLUDES = "%INCLUDES%";
str TYPES = "%TYPES%";
str API = "api";

bool USE_ALARM = false;
list[Label] WATCHED_EVENTS = [];

// =============================================================
// Graph methods
// =============================================================
// Helpers
private tuple[State, int] fresh(int next)
  = <sid(next), next + 1>;

private tuple[State, State, int] fresh2(int next) {
  <a, n1> = fresh(next);
  <b, n2> = fresh(n1);
  return <a, b, n2>;
}

// =============================================================
// Helpers
public str toString(value val)
  = "<val>";

void generateDir(loc output)
{
  if (exists(output))
    return;

  mkDirectory(output);
}

str clean(value val)
  = replaceAll("<val>", "\"", "");

str cleanNewLine(value val)
  = "<val>"[..-1];

void createExternalTypes()
{
  generateDir(OUTPUT_DIR);
  touch(OUTPUT_DIR + EXTERNALS);

  str output = "
// <INCLUDES>
// <TYPES>
extern float $float$;
extern int $int$;
extern boolean $bool$;
extern string $std::string$;
extern pose $const geometry_msgs::msg::PoseStamped&$;
";

  writeFile(OUTPUT_DIR + EXTERNALS, output);
}

// Conversions
str toComponent(str name)
  = "c" + uncapitalize(replaceAll(name, " ", ""));

str toInterface(str name)
  = "i" + uncapitalize(replaceAll(name, " ", ""));

str toVariable(str name)
  = uncapitalize(replaceAll(name, " ", ""));

str toFilename(str name)
  = replaceAll(name, " ", "");

public str argToStr(list[Argument] args, Env env) {
  str out = "";
  for (arg <- args) {
    <_, a> = generate(arg, env);
    out += "<a>, ";
  }
  return out[..-2];
}

public str argToStr(list[Expression] args, Env env) {
  str out = "";
  for (arg <- args) {
    <_, a> = generate(arg, env);
    out += "<a>, ";
  }

  return out[..-2];
}

// ============================================================
// Expression
// Variables and Constants
public Result generate(\var(str name), Env env)
{
  if (name in env)
    return <env, env[name]>;

  if ("flow_args" in env && map[str, str] args := env["flow_args"] && name in args) {
    return <env, args[name]>;
  }

  if (name == "true" || name == "false")
    return <env, "$<name>$">;

  return <env, name>;
}

public Result generate(\str_const(str val), Env env)
{
  return <env, "$<val>$">;
}

public Result generate(\int_const(int val), Env env)
{
  return <env, "$<val>$">;
}

public Result generate(\float_const(real val), Env env)
{
  return <env, "$<val>$">;
}

// Call
public Result generate(\call_expr(EventStatement stmt), Env env)
{
  return generate(stmt, env);
}

// Basic math operators
public Result generate(\neg(Expression arg), Env env)
{
 return <env, "">;
}

public Result generate(\mul(Expression lhs, Expression rhs), Env env)
{
 return <env, "">;
}

public Result generate(\div(Expression lhs, Expression rhs), Env env)
{
 return <env, "">;
}

public Result generate(\add(Expression lhs, Expression rhs), Env env)
{
 return <env, "">;
}

public Result generate(\sub(Expression lhs, Expression rhs), Env env)
{
 return <env, "">;
}

// Basic boolean operators
public Result generate(\not(Expression lhs), Env env)
{
 return <env, "">;
}

public Result generate(\eq(Expression lhs, Expression rhs), Env env)
{
 return <env, "">;
}

public Result generate(\neq(Expression lhs, Expression rhs), Env env)
{
  <lenv, l> = generate(lhs, env);
  <lenv, r> = generate(rhs, lenv);
  return <env, "<l> != <r>">;
}

public Result generate(\leq(Expression lhs, Expression rhs), Env env)
{
  <lenv, l> = generate(lhs, env);
  <lenv, r> = generate(rhs, lenv);
  return <env, "<l> \<= <r>">;
}

public Result generate(\geq(Expression lhs, Expression rhs), Env env)
{
  <lenv, l> = generate(lhs, env);
  <lenv, r> = generate(rhs, lenv);
  return <env, "<l> \>= <r>">;
}

public Result generate(\lt(Expression lhs, Expression rhs), Env env)
{
  <lenv, l> = generate(lhs, env);
  <lenv, r> = generate(rhs, lenv);
  return <env, "<l> \< <r>">;
}

public Result generate(\gt(Expression lhs, Expression rhs), Env env)
{
  <lenv, l> = generate(lhs, env);
  <lenv, r> = generate(rhs, lenv);
  return <env, "<l> \> <r>">;
}

public Result generate(\conj(Expression lhs, Expression rhs), Env env)
{
 return <env, "">;
}

public Result generate(\disj(Expression lhs, Expression rhs), Env env)
{
 return <env, "">;
}

// ============================================================
// Strategy
public tuple[NFAx,int] build(\task(EventStatement call, list[StrategyHandler] handlers), int next, Env env) {
  <lenv, a> = generate(call, env);

  str arg = "";
  if (call is \event_call) {
    arg = env[lenv["member"]];
  } else {
    str name = "<lenv["call"]>";
    if (name in env) {
      arg = env[lenv["call"]];
    } else if (map[str, Strategy] strategies := env["flows"] && name in strategies && map[str, list[str]] flowArgs := env["all_flow_args"]) {
      env["flow_args"] = (a: val | <str a, str val> <- zip2(flowArgs[name], lenv["args"]));
      return build(strategies[name], next, env);
    } else {
      throw "Unknown call: <lenv["call"]>";
    }
  }
  // str arg = toString(call is \event_call ? env[lenv["member"]] : env[lenv["call"]]);

  set[State] resets = {};
  if (TaskDef task := tenv[arg]) {
    if (task.ttype == "sync") {
      // --- SYNC: fire-and-forget: s --act--> fOK (no wait, no sigs)
      <s, fOK, n1> = fresh2(next);
      rel[State,Label,State] T = {};

      if (call is \event_call)
        T = { <s, actm(lenv["member"], lenv["call"], lenv["args"], ""), fOK> };
      else
        T = { <s, act(lenv["call"], lenv["args"], ""), fOK> };

      return < nfax(\nfa({ s, fOK }, s, { fOK }, T), resets), n1 >;
    } else {
      <s, w, n1> = fresh2(next);
      <fOK, n2> = fresh(n1);

      set[State] S = {};
      rel[State,Label,State] T = {};

      if (call is \event_call) {
        // --- SYNC: fire-and-forget: s --act--> fOK (no wait, no sigs)
        assert size(handlers) == 0;
        S = { s, fOK };
        T = { <s, actm(lenv["member"], lenv["call"], lenv["args"], ""), fOK> };
      } else {
        // -- ASYNC: fire-and-wait: s --act--> w --sig--> fOk
        S = { s, w, fOK };
        T += <s, act(lenv["call"], lenv["args"], ""), w>;
        T += <w, sig(lenv["call"], "return", lenv["args"]), fOK>;
      }

      set[State] F = { fOK };

      for (handler <- handlers) {
        if (\error(Strategy e_strat) := handler) {
          <mxa, n3> = build(e_strat, n2, env);
          ma = mxa.m;
          S += ma.S;
          F += ma.F;
          T += ma.T;
          T += { <w, sig(lenv["call"], "error", lenv["args"]), ma.s0> };
          n2 = n3;
          resets += mxa.resets;
        } else if (\abort(Strategy a_strat) := handler) {
          // compile the abort handler subflow
          <mxa, n3> = build(a_strat, n2, env);
          ma = mxa.m;

          // abort gate: on api.abort, emit t.abort(), then enter handler
          <sA, wA, n4> = fresh2(n3);

          S += ma.S + { sA, wA };
          F += ma.F;
          T += ma.T
          + { <w,  sig(API, env["abort_call"], env["abort_args"]), sA> }  // input
          + { <sA, abrt(lenv["call"], lenv["args"]), wA> }                // output x.abort()
          + { <wA, eps(), ma.s0> };                                       // step into handler

          resets += mxa.resets;
          n2 = n4;
        } else if (\emitter(EventStatement call, Strategy o_strat) := handler) {
          bool stay = false;
          if (\seq(Strategy body, \ref("continue")) := o_strat) {
            stay = true;
            o_strat = body;
          }

          <mxa, n3> = build(o_strat, n2, env);
          ma = mxa.m;
          S += ma.S;
          T += ma.T;
          <lenv, a> = generate(call, env);

          Label label = sig(lenv["member"], lenv["call"], lenv["args"]);

          if (stay) {
            // Handle & STAY in 'w':
            // w --label--> ma.s0 --(ma.T)--> ma.F --eps--> w
            if (ma.S == {}) {
              T += { <w, label, w> };
            } else {
              T += { <w, label, ma.s0> };
              for (f <- ma.F) {
                T += { <f, eps(), w> };
              }
            }
          } else {
            T += { <w, label, ma.s0> };
            F += ma.F;
          }

          n2 = n3;
          resets += mxa.resets;

          if (label notin WATCHED_EVENTS)
            WATCHED_EVENTS += label;
        }
      }

      return < \nfax(\nfa(S, s, F, T), resets), n2 >;
    }
  }

  throw "Called inexistent task: <lenv["call"]>";
}

public tuple[NFAx,int] build(\ref(str name), int next, Env env) {
  if ("flows" notin env)
    throw "Flows not intialized";

  if (map[str, Strategy] strategies := env["flows"] && name in strategies)
    return build(strategies[name], next, env);

  throw "<name> not in environment";
}

private tuple[NFAx,int] build(\end(), int next, Env env) {
  <e, n1> = fresh(next);
  NFA m = nfa({e}, e, {e}, {});

  // End is a reset state
  return < nfax(m, { e }), n1 >;
}

public tuple[NFAx,int] build(\seq(Strategy a, Strategy b), int next, Env env) {
  <mxa, n1> = build(a, next, env);
  <mxb, n2> = build(b, n1, env);

  // Get NFA from NFAx
  ma = mxa.m;
  mb = mxb.m;

  set[State] S  = ma.S + mb.S;
  State s0      = ma.s0;
  set[State] F  = mb.F;

  set[State] linkFrom = ma.F - mxa.resets;

  // ε from every final of A to start of B
  rel[State, Label, State] T = ma.T + mb.T + { <f, eps(), mb.s0> | f <- linkFrom };

  return <\nfax(\nfa(S, s0, F, T), mxa.resets + mxb.resets), n2>;
}

public tuple[NFAx,int] build(\par(list[Strategy] bs, bool joint), int next, Env env)
{
  tuple[NFAx,int] parProductX(NFAx ax, NFAx bx, bool joint, int next) {
    NFA ma = ax.m;
    NFA mb = bx.m;

    set[State] S = { spair(sa, sb) | sa <- ma.S, sb <- mb.S };
    State s0     = spair(ma.s0, mb.s0);

    rel[State,Label,State] Ta =
      { <spair(sa,sb), l, spair(sa2,sb)> | <sa, l, sa2> <- ma.T, sb <- mb.S };
    rel[State,Label,State] Tb =
      { <spair(sa,sb), l, spair(sa, sb2)> | <sb, l, sb2> <- mb.T, sa <- ma.S };
    rel[State,Label,State] T = Ta + Tb;

    set[State] F =
      joint
        ? { spair(fa, fb) | fa <- ma.F, fb <- mb.F }
        : { spair(fa, sb) | fa <- ma.F, sb <- mb.S }
          + { spair(sa, fb) | sa <- ma.S, fb <- mb.F };

    set[State] R = { spair(ra, sb) | ra <- ax.resets, sb <- mb.S } + { spair(sa, rb) | sa <- ma.S,     rb <- bx.resets };

    return <\nfax(\nfa(S, s0, F, T), R), next>;
  }

  // build and combine the rest
  <acc, n> = build(bs[0], next, env);
  for (int i <- [1 .. size(bs)]) {
    <m, n2> = build(bs[i], n, env);
    <acc2, n3> = parProductX(acc, m, joint, n2);
    acc = acc2;
    n = n3;
  }

  return <acc, n>;
}

public tuple[NFAx,int] build(\repeat(Strategy body), int next, Env env) {
  <mxb, n1> = build(body, next, env);
  mb = mxb.m;

  // finals that should loop vs. finals that should reset
  set[State] loopFrom = mb.F - mxb.resets;

  rel[State,Label,State] T2 = mb.T + { <f, eps(), mb.s0> | f <- loopFrom };

  // carry reset points unchanged; augment step will wire them to State0
  return <\nfax(\nfa(mb.S, mb.s0, mb.F, T2), mxb.resets), n1>;
}

private tuple[NFAx,int] build(\let(str p, EventStatement call), int next, Env env) {
  <lenv, _> = generate(call, env);
  <s, fOK, n1> = fresh2(next);

  rel[State,Label,State] T = {};
  if (call is \event_call)
    T = { <s, actm(lenv["member"], lenv["call"], lenv["args"], p), fOK> };
  else
    T = { <s, act(lenv["call"], lenv["args"], p), fOK> };

  // label remains actm; binding handled in emitter
  NFA mfa = nfa({s, fOK}, s, {fOK}, T);
  return < nfax(mfa, {}), n1 >;
}

private tuple[NFAx,int] build(\guard(Expression expr), int next, Env env) {
  println("Guard: <expr>");
  <lenv, cond> = generate(expr, env);
  <s, fOK, n1> = fresh2(next);

  rel[State,Label,State] T = { <s, act("guard", [cond], ""), fOK> };

  // label remains actm; binding handled in emitter
  NFA mfa = nfa({s, fOK}, s, {fOK}, T);
  return < nfax(mfa, {}), n1 >;
}

private tuple[NFAx,int] build(\if_strat(Expression condition, Strategy t), int next, Env env) {
  <tx, n1> = build(t, next, env);

  <lenv, a> = generate(condition, env);

  // create a branch node B with epsilon to both
  <b, n2> = fresh(n1);
  NFA m = \nfa(
    tx.m.S + { b },
    b,
    tx.m.F,
    // tx.m.T + { <b, eps(), tx.m.s0> }
    tx.m.T + { <b, act("if", [a], ""), tx.m.s0> }
  );

  // println(m);

  return < nfax(m, tx.resets), n2 >;
}

private tuple[NFAx,int] build(\ifelse_strat(Expression condition, Strategy t, Strategy e), int next, Env env) {
  <tx, n1> = build(t, next, env);
  <ex, n2> = build(e, n1,   env);

  <lenv, a> = generate(condition, env);

  // create a branch node B with epsilon to both
  <b, n3> = fresh(n2);
  NFA m = \nfa(
    tx.m.S + ex.m.S + { b },
    b,
    tx.m.F + ex.m.F,
    // tx.m.T + ex.m.T + { <b, eps(), tx.m.s0>, <b, eps(), ex.m.s0> }
    tx.m.T + ex.m.T + { <b, act("if", [], ""), tx.m.s0>, <b, act("else", [], ""), ex.m.s0> }
  );

  return < nfax(m, tx.resets + ex.resets), n3 >;
}

private NFA addTimeoutHandlerToRegion(NFA region, Label timeoutLbl, State elseStart) {
  set[State] nonFinals = region.S - region.F;
  rel[State,Label,State] extra = { <p, timeoutLbl, elseStart> | p <- nonFinals };

  return \nfa(region.S, region.s0, region.F, region.T + extra);
}

private tuple[NFAx,int] build(\within(int duration, Strategy doStrat, Strategy elseStrat), int next, Env env) {
  USE_ALARM = true;
  // 1 Build both branches against a fresh continuation.
  <doX,   n1> = build(doStrat,   next, env);
  <elseX, n2> = build(elseStrat, n1,   env);

  // 2 Fresh nodes: entry 'b', join 'j', and a tiny node 'r' to host alarm.reset()
  <b, n3> = fresh(n2);
  <r, n4> = fresh(n3);
  <r0, n5> = fresh(n4);

  // 3 Materialise labels for alarm
  Label Lset     = actm("alarm", "set",    ["$<duration>$"], "");
  Label Lreset   = actm("alarm", "reset",  [], "");
  Label Ltimeout = sig ("alarm", "timeout", ["false"]);

  // 4 From entry 'b': alarm.set(duration) → start the work branch
  rel[State,Label,State] head = { <b, Lset, doX.m.s0> };

  // 5 When the work finishes: alarm.reset() → join
  rel[State,Label,State] resetEdges = { <f, Lreset, r> | f <- doX.m.F - doX.resets };
  rel[State,Label,State] resetEdges2 = { <f, Lreset, r0> | f <- doX.resets };
  // rel[State,Label,State] toJoin = { <r, eps(), j> };

  // 6 Timeout: from ANY non-final state of the work region, on timeout → else-branch
  NFA doWithTimeout = addTimeoutHandlerToRegion(doX.m, Ltimeout, elseX.m.s0);

  // 8 Compose the whole NFA
  set[State] S = doWithTimeout.S + elseX.m.S + { b, r, r0 };
  State s0     = b;
  set[State] F = { r, r0 } + elseX.m.F;

  rel[State,Label,State] T = doWithTimeout.T + elseX.m.T + head + resetEdges + resetEdges2;

  NFA m = \nfa(S, s0, F, T);

  // 9 Resets bookkeeping (union of subgraphs)
  set[State] resets = doX.resets + elseX.resets + r0;

  return < nfax(m, resets), n5 >;
}

private tuple[NFAx,int] build(\every(int period, Strategy body, list[StrategyHandler] hs), int next, Env env) {
  USE_ALARM = true;

  // 1 Fresh states
  <loop, next> = fresh(next);
  <wait, next> = fresh(next);
  <r0, next> = fresh(next);

  Label Lset     = actm("alarm", "set",    ["$<period>$"], ""); // or keep Expression → strings uniformly in your labels
  Label Lreset   = actm("alarm", "reset",  [], "");
  Label Ltimeout = sig ("alarm", "timeout", ["true"]);

  // 2 Static scaffolding: set() then go wait
  set[State] S = {loop, wait};
  set[State] F = {};
  rel[State,Label,State] T = { <loop, Lset, wait> };
  set[State] resets = {};

  // Build body against 'r' continuation
  <mbx, next> = build(body, next, env);

  S += mbx.m.S;
  T += mbx.m.T;
  F += mbx.m.F;
  resets += mbx.resets;

  // 4 EVENT/ERROR/ABORT branches: on X --> reset --> S --> j (exit)
  // helper to build a branch: wait --L--> (reset) --> strategy --> j
  lenv = env;
  for (handler <- hs) {
    if (\error(Strategy e_strat) := handler) {
      <j, next> = fresh(next);
      <mxa, next> = build(e_strat, next, env);
      ma = mxa.m;
      S += ma.S + j;
      F += ma.F;
      T += ma.T;
      T += { <wait, sig(lenv["call"], "error", lenv["args"]), j>, <j, Lreset, ma.s0> };
      resets += mxa.resets;
    } else if (\abort(Strategy a_strat) := handler) {
      // compile the abort handler subflow
      <j, next> = fresh(next);
      <mxa, next> = build(a_strat, next, env);
      ma = mxa.m;

      // abort gate: on api.abort, emit t.abort(), then enter handler
      <sA, wA, next> = fresh2(next);

      S += ma.S + { sA, wA, j };
      F += ma.F;
      T += ma.T
        + { <wait,  sig(API, env["abort_call"], env["abort_args"]), sA> }
        + { <sA, eps(), wA> }
        + { <wA, eps(), j> }
        + { <j, Lreset, ma.s0> };

      resets += mxa.resets;
    } else if (\emitter(EventStatement call, Strategy o_strat) := handler) {
      <j, next> = fresh(next);

      bool stay = false;
      if (\seq(Strategy body, \ref("continue")) := o_strat) {
        stay = true;
        o_strat = body;
      }

      <mxa, next> = build(o_strat, next, env);
      ma = mxa.m;
      S += ma.S + j;
      // F += ma.F;
      <lenv, a> = generate(call, env);
      Label label = sig(lenv["member"], lenv["call"], lenv["args"]);

      T += ma.T;
      if (stay) {
        if (ma.S == {}) {
          T += { <wait, label, wait> };
        } else {
          // T += { <wait, label, ma.s0> };
          T += { <wait, label, ma.s0> };
          for (f <- ma.F) {
            T += { <f, eps(), wait> };
          }
        }
      } else {
        T += { <wait, label, j>, <j, Lreset, ma.s0> };
        F += ma.F;
      }

      // T += ma.T + { <wait, label, j>, <j, Lreset, ma.s0> };

      if (label notin WATCHED_EVENTS)
        WATCHED_EVENTS += label;

      resets += mxa.resets;
    }
  }

  // All final states should set the alarm again
  rel[State,Label,State] resetEdges = { <f, Lset, wait> | f <- F - resets};
  rel[State,Label,State] resetEdges2 = { <f, Lreset, r0> | f <- resets };

  S += { r0 };
  F += { r0 };
  T += { <wait, Ltimeout, mbx.m.s0> } + resetEdges + resetEdges2;
  resets += { r0 };

  // 5 Return NFAx:
  //    - single entry is 'loop' (callers will ε-link into it)
  //    - single final is 'j' (so the whole thing converges)
  //    - no special 'resets' here (keep it empty so seq() linker works)
  NFA m = nfa(S, loop, F, T);

  return <\nfax(m, resets), next>;
}

// ===========================================================================
// Bloody helpful
private str ppArgs(list[str] xs) = intercalate(", ", xs);
private str ppCall(str port, EventCall call) = "<port>.<call.name>(<ppArgs(call.args)>)";
private bool isCall(Label l) = (l is act) || (l is abrt) || (l is actm);
private list[tuple[int,Label,int]] outsFrom(DFAI d, int q) = [ t | t <- d.T, <q, _, _> := t];

public NFA augmentWithTriggerAndResets(NFAx x, str trigName, list[str] formals) {
  int maxSid(State s) {
    switch (s) {
      case sid(i): return i;
      case spair(l, r): return max([maxSid(l), maxSid(r)]);
      default: throw "Not a valid SID";
    }
  }

  int maxSidNFA(NFA m) {
    int mmax = -1;
    for (s <- m.S) mmax = max([mmax, maxSid(s)]);
    for (<State a, Label _, State b> <- m.T) {
      mmax = max([mmax, maxSid(a)]);
      mmax = max([mmax, maxSid(b)]);
    }
    return mmax;
  }

  NFA m = x.m;
  int next = maxSidNFA(m) + 1;
  State state0 = sid(next);

  set[State] S2 = m.S + { state0 };
  rel[State,Label,State] T2 = m.T
    + { <state0, sig(API, trigName, formals), m.s0> }
    + { <r, eps(), state0> | r <- x.resets }
    + { <r, sig(API,"__reset__",[]), state0> | r <- x.resets };

  set[State] F2 = m.F - x.resets;

  return \nfa(S2, state0, F2, T2);
}

private str outKey(OutCall c) {
  switch (c) {
    case ocall(str t, str m, list[str] as, str _): return "O:" + t + "." + m + "(" + toString(as) + ")";
    case mcall(str t, str m, list[str] as, str _): return "M:" + t + "." + m + "(" + toString(as) + ")";
    case oabort(str t, list[str] as):       return "A:" + t + "(" + toString(as) + ")";
  }
  throw "Unknown outcall: <c>";
}

private list[tuple[int,Label,int]] sortedCallOuts(DFAI d, int q) {
  list[tuple[int,Label,int]] outs = [ <a,l,b> | <int a, Label l,int b> <- d.T, a == q, isCall(l) ];
  return sort(outs, bool(tuple[int,Label,int] x, tuple[int,Label,int] y) {
    Label lx = x[1];
    Label ly = y[1];
    str k(Label l) {
      switch (l) {
        case act (str t, list[str] as, str _):        return "0:" + t + ":trigger:" + toString(as);
        case actm(str t, str m, list[str] as, str _): return "0:" + t + ":" + m + ":" + toString(as);
        case abrt(str t, list[str] as):        return "1:" + t + ":abort" + toString(as);
        default: return "z";
      }
    }
    return k(lx) < k(ly);
  });
}

public tuple[map[int,list[OutCall]], map[int,int]] entryPlan(DFAI dFull, Env env) {
  OutCall asOut(Label l) {
    switch (l) {
      case act(str n, list[str] as, str ret): {
        if (n == "guard") {
          return mcall(n, "guard", as, ret);
        } else if (n == "if") {
          return mcall(n, "if", as, ret);
        }

        str meth = tenv[env[n]].trigger.name;
        return mcall(n, meth, as, ret);
      }
      case actm(str t, str m, list[str] as, str ret): return ocall(t, m, as, ret);
      case abrt(str n, list[str] as):        return oabort(n, as);
      default:                               throw "not an output call";
    }
  }

  tuple[list[OutCall], int] callClosure(int q0) {
    set[int] seenStates = {};
    list[int] queue = [ q0 ];
    list[OutCall] calls = [];
    set[str] seenCallKeys = {};
    set[int] sinks = {};

    while (!isEmpty(queue)) {
      int q = head(queue); queue = tail(queue);
      if (q in seenStates) continue;
      seenStates += {q};

      list[tuple[int,Label,int]] callOut = sortedCallOuts(dFull, q);
      if (isEmpty(callOut)) {
        sinks += {q};
        continue;
      }

      // process calls from this state in a deterministic order, but KEEP their sequence
      for (<int _, Label l, int b> <- callOut) {
        OutCall oc = asOut(l);
        str key = outKey(oc);
        if (!(key in seenCallKeys)) {
          calls += [ oc ];             // append in discovery order
          seenCallKeys += { key };
        }
        queue += [ b ];
      }
    }

    if (size(sinks) != 1) throw "callClosure: non-unique sink from <q0>: <sinks>";
    return <calls, head(toList(sinks))>;
  }

  map[int,list[OutCall]] entryCalls = ();
  map[int,int] entrySink = ();
  for (<int _, Label l, int b> <- dFull.T) {
    if (l is sig) {
      <cs, sink> = callClosure(b);
      entryCalls[b] = cs;
      entrySink[b] = sink;
    }
  }
  return <entryCalls, entrySink>;
}

private list[int] candidateTargetsForEdge(DFAI dFull, map[int,int] full2wait, int srcW, Label lSig, int dstW) {
  list[int] out = [];

  for (<int aF, Label lf, int bF> <- dFull.T) {
    if (!(lf is sig))
      continue;
    if (!(lf == lSig))
      continue;

    int srcWaitFull = (aF in full2wait) ? aF : callSink(dFull, aF);
    srcWaitFull = followReset(dFull, srcWaitFull);

    if (!(srcWaitFull in full2wait))
      continue;
    if (full2wait[srcWaitFull] != srcW)
      continue;

    int sinkFull = callSink(dFull, bF);
    sinkFull = followReset(dFull, sinkFull);

    if (!(sinkFull in full2wait))
      continue;
    if (full2wait[sinkFull] != dstW)
      continue;

    out += [ bF ];
  }
  return out;
}

private list[OutCall] pickCanonicalCalls(list[int] bFs, map[int,list[OutCall]] entryCalls) {
  list[list[OutCall]] options = [];
  for (int bF <- bFs) {
    list[OutCall] cs =
      (bF in entryCalls) ? entryCalls[bF]
                         : [];
    options += [ cs ];
  }

  // dedup options by content
  set[str] seenKey = {};
  list[list[OutCall]] uniq = [];
  for (list[OutCall] cs <- options) {
    // canonical string key
    str key = intercalate("|", [ c.task + ":" + toString(c.args) | c <- cs ]);
    if (key in seenKey)
      continue;

    seenKey += { key };
    uniq += [ cs ];
  }

  if (isEmpty(uniq)) return []; // no immediate calls

  // pick the smallest; tie-break lexicographically by the key
  list[list[OutCall]] sorted =
    sort(uniq, bool(list[OutCall] u, list[OutCall] v) {
      if (size(u) != size(v))
        return size(u) < size(v);

      str ku = intercalate("|", [ c.task + ":" + toString(c.args) | c <- u ]);
      str kv = intercalate("|", [ c.task + ":" + toString(c.args) | c <- v ]);
      return ku < kv;
    });

  return head(sorted);
}

public EventCall applyTpl(EventTpl t, list[str] actuals) {
  // println(t);
  // println(actuals);
  map[str,str] sub = ( t.args[i] : actuals[i] | i <- [0 .. size(t.args)] );
  list[str] out = [ (a in sub) ? sub[a] : a | a <- t.args ];
  return ecall(t.name, out);
}

public map[int,str] dezyneStateNames(DFAI di) {
  list[int] rest = sort([ s | s <- toList(di.S), s != di.s0 ], bool(int a, int b) { return a < b; });
  map[int,str] names = ( di.s0 : "State0" );

  int k = 1;
  for (int s <- rest) {
    names[s] = "State<k>";
    k += 1;
  }

  return names;
}

private tuple[str, bool] emitSignal(Label l, Env env) {
  if (sig(str src, str ev, list[str] as) := l) {
    if (src == API) {
      if (ev == "__reset__")
        return <"", true>;

      return <"on <API>.<ev>(<ppArgs(as)>):", true>;
    } else if (src == "alarm") {
      assert size(as) == 1;
      return <"on alarm.<ev>():", fromString(as[0])>;
    }

    // task return/error/abort -> map to concrete event name
    str arg = "<env[src]>";
    if (!(arg in tenv)) throw "Unknown task in sig: <src>";

    TaskDef td = tenv[arg];
    EventTpl tpl = empty_etpl();

    switch (ev) {
      case "return": { tpl = td.onReturn is empty_etpl ? etpl("return", as, "") : td.onReturn;  }
      case "error":  { tpl = td.onError  is empty_etpl ? etpl("error",  as, "") : td.onError;   }
      case "abort":  { tpl = td.onAbort  is empty_etpl ? etpl("abort",  as, "") : td.onAbort;   }
      default:       {
        if (etpl(ev, list[str] _, str _) := td.onReturn)
          tpl = td.onReturn;
        else if (etpl(ev, list[str] _, str _) := td.onError)
          tpl = td.onError;
        else if (etpl(ev, list[str] _, str _) := td.onAbort)
          tpl = td.onAbort;
        else
          tpl = ev in td.emits ? td.emits[ev] : empty_etpl();
      }
    };

    EventCall concrete = applyTpl(tpl, as);
    return <"on <ppCall(src, concrete)>:", (ev != "error")>;
  }

  return <"", true>; // only called for sig
}

private str emitOutCall(OutCall c, Env env) {
  switch (c) {
    case mcall(str t, str _, list[str] as, str ret): {
      if (t == "guard") {
        assert size(as) == 1;
        return "if (!(<as[0]>)) illegal";
      } else if (t == "if") {
        return "if (<as[0]>) {";
      }

      TaskDef td = tenv[env[t]];
      EventCall trig = applyTpl(td.trigger, as);
      return (isEmpty(ret) ? "" : "<ret> = ") + "<t>.<trig.name>(<ppArgs(trig.args)>)";
    }
    case ocall(str t, str obj, list[str] as, str ret): {
      if (t == "alarm") {
        return (isEmpty(ret) ? "" : "<ret> = ") + "<t>.<obj>(<ppArgs(as)>)";
      }

      TaskDef td = tenv[env[t]];
      if (td.ttype == "sync") {
        EventCall trig = applyTpl(td.accepts[obj], as);
        return (isEmpty(ret) ? "" : "<td.accepts[obj].ret> <ret> = ") + "<t>.<trig.name>(<ppArgs(trig.args)>)";
      }

      if (obj == "abort" || etpl(obj, list[str] _, str _) := td.onAbort) {
        EventCall trig = applyTpl(td.onAbort, as);
        return (isEmpty(ret) ? "" : "<ret> = ") + "<t>.<trig.name>(<ppArgs(trig.args)>)";
      } else if (obj == "trigger" || etpl(obj, list[str] _, str _) := td.trigger) {
        EventCall trig = applyTpl(td.trigger, as);
        return (isEmpty(ret) ? "" : "<ret> = ") + "<t>.<trig.name>(<ppArgs(trig.args)>)";
      }
    }
    case oabort(str t, list[str] as): {
      TaskDef td = tenv[env[t]];
      EventCall trig = applyTpl(td.onAbort, as);
      return "<t>.<trig.name>(<ppArgs(trig.args)>)";
    }
  }

  throw "Invalid Outcall: <c>";
}

private bool hasCallOut(DFAI d, int q)
  = !isEmpty([a | <int a, Label l, int _> <- d.T, l is act, a == q]);

private int callSink(DFAI d, int q0) {
  set[int] seen = {};
  list[int] todo = [q0];
  set[int] sinks = {};

  while (!isEmpty(todo)) {
    int q = head(todo);
    todo = tail(todo);
    if (q in seen)
      continue;

    seen += {q};

    list[tuple[int,Label,int]] callOut = [ e | e <- outsFrom(d,q), (<_,Label l,_> := e), isCall(l) ];
    if (isEmpty(callOut)) {
      sinks += {q};
      continue;
    }
    for (<int _, Label _, int b> <- callOut) todo += [ b ];
  }

  if (size(sinks) != 1) throw "callSink: parallel region from <q0> doesnt converge: <sinks>";

  return head(toList(sinks));
}

private int followReset(DFAI d, int q) {
  int cur = q;
  bool moved = true;
  while (moved) {
    moved = false;
    for (<int a, Label l, int b> <- d.T) {
      if (a == cur && (l is sig) && (sig(str c, str ev, list[str] _) := l && ev == "__reset__")) {
        cur = b;
        moved = true;
        break;
      }
    }
  }
  return cur;
}

public tuple[DFAI, map[int,int]] projectToWaitDFAWithMap(DFAI dFull) {
  // wait states in full graph
  set[int] W = { q | q <- dFull.S, !hasCallOut(dFull, q) };
  int s0wFull = (dFull.s0 in W) ? dFull.s0 : callSink(dFull, dFull.s0);
  s0wFull = followReset(dFull, s0wFull);

  rel[int,Label,int] Tsig =
    { <a, l, followReset(dFull, callSink(dFull, b))>
      | <int a, Label l, int b> <- dFull.T, (l is sig), a in W
    };

  // reachable wait states via sig-only graph
  set[int] seen = { s0wFull }; list[int] todo = [ s0wFull ];
  while (!isEmpty(todo)) {
    int x = head(todo);
    todo = tail(todo);

    for (<int a, Label _, int y> <- Tsig) {
      if (a == x && y notin seen) {
        seen += {y};
        todo += [y];
      }
    }
  }

  set[int] S2 = seen;
  rel[int,Label,int] T2 = { <a,l,b> | <int a, Label l, int b> <- Tsig, a in S2, b in S2 };

  // reindex to 0..n-1 with Idle first, and build full->wait map
  list[int] order = [ s0wFull ]
    + sort([ q | q <- toList(S2), q != s0wFull ], bool(int a,int b){ return a < b; });
  map[int,int] full2wait = ( order[i] : i | i <- [0 .. size(order)] );

  DFAI diWait = dfai(
    { full2wait[q] | q <- S2 },
     full2wait[s0wFull],
    {},
    { < full2wait[a], l, full2wait[b] > | <int a, Label l, int b> <- T2 }
  );
  return <diWait, full2wait>;
}

public str emitBehaviorWait(DFAI dFull, DFAI dWait, map[int,int] full2wait, Env env) {
  // Plan immediate calls on FULL graph
  println("Getting entry plan");
  <entryCalls, _> = entryPlan(dFull, env);

  // Names on projected wait-only DFA
  println("Getting dezyne names");
  map[int,str] names = dezyneStateNames(dWait);

  // Emit one handler per projected sig-edge
  list[int] orderedStates =
    [ dWait.s0 ] + sort([ q | q <- toList(dWait.S), q != dWait.s0 ],
                        bool(int a,int b){ return a < b; });

  str out = "  behavior {\n";
  out += "    enum State { " + intercalate(", ", [ names[s] | s <- orderedStates ]) + " };\n";
  out += "    State state = State." + names[dWait.s0] + ";\n";
  if ("vars" in env && list[tuple[str,str,str,str]] vars := env["vars"]) {
    for (var <- vars)
      out += "    <var[0]> <var[1]> = <var[3]>;\n";
  }
  out += "\n";

  for (int sW <- orderedStates) {
    out += "    [state." + names[sW] + "] {\n";

    // all projected sig-edges out of sW
    list[tuple[int,Label,int]] sigOut =
      [ e | e <- outsFrom(dWait, sW), (<_,Label l,_> := e), l is sig ];

    // stable order
    sigOut = sort(sigOut,
                  bool(tuple[int,Label,int] x, tuple[int,Label,int] y) {
                    a = emitSignal(x[1], env);
                    b = emitSignal(y[1], env);
                    return  a[0] < b[0];
                  });

    bool hasStart = false;
    list[Label] signals = [];
    for (<int _, Label lSig, int dstW> <- sigOut) {
      // collect all FULL candidates that normalize to this projected edge
      list[int] bFs = candidateTargetsForEdge(dFull, full2wait, sW, lSig, dstW);

      // pick canonical immediate calls
      list[OutCall] cs = pickCanonicalCalls(bFs, entryCalls);

      // header text (rendered via templates)
      <header, is_success> = emitSignal(lSig, env);
      out += "      <header> {\n";
      bool apiCall = false;
      if (sig(str c, str ev, list[str] _) := lSig) {
        if (sW == 0 && ev == env["trigger_call"] && "vars" in env && list[tuple[str,str,str,str]] vars := env["vars"]) {
          for (var <- vars)
            out += "        <var[1]> = <var[2]>;\n";
          out += "\n";
        }

        if (c == "api") {
          apiCall = true;
          if (ev == env["trigger_call"])
            hasStart = true;
        }
      }

      // Generate output calls
      for (OutCall oc <- cs)
        out += "        <emitOutCall(oc, env)>;\n";

      // Make sure the top level also gets the call
      if (!is_success && "error_call" in env && dstW == 0)
        out += "        <API>.<env["error_call"]>(<if ("error_args" in env){><ppArgs(env["error_args"])><}>);\n";

      if (is_success && !apiCall &&
          "State0" == names[dstW] && "return_call" in env)
        out += "        <API>.<env["return_call"]>(<if ("return_args" in env){><ppArgs(env["return_args"])><}>);\n";

      if (dstW != sW)
        out += "        state = State." + names[dstW] + ";\n";

      signals += lSig;
      out += "      }\n";
    }

    for (label <- WATCHED_EVENTS) {
      if (label in signals)
        continue;

      <watchedSignal, _> = emitSignal(label, env);
      out += "      <watchedSignal> {}\n";
    }

    // Define what is allowed in the other states for the provided port
    if (sW == 0) {
      if ("abort_call" in env)
        out += "      on <API>.<env["abort_call"]>(<if ("abort_args" in env){><ppArgs(env["abort_args"])><}>): {}\n";
    } else {
      if (!hasStart)
        out += "      on <API>.<env["trigger_call"]>(<if ("trigger_args" in env){><ppArgs(env["trigger_args"])><}>): {}\n";
    }

    out += "    }\n";
  }

  out += "  }";
  return out;
}

// ============================================================
// RosDefStatement
public Result generate(\trigger_block(EventDefStatement call), Env env)
{
  if (\event(str ret, str id, list[Argument] arguments, list[EventDefComponent] _) := call) {
    list[str] args = [];
    for (arg <- arguments) {
      if (\arg(str arg_type, str arg_id) := arg)
      // <_, a> = generate(arg, env);
        args += "<arg_id>";
    }

    env["trigger_call"] = id;
    env["trigger_args"] = args;
    env["trigger_ret"] = ret;
  }
  return <env, "">;
}

public Result generate(\return_block(EventDefStatement call), Env env)
{
  if (\event(str ret, str id, list[Argument] arguments, list[EventDefComponent] _) := call) {
    list[str] args = [];
    for (arg <- arguments) {
      if (\arg(str arg_type, str arg_id) := arg)
      // <_, a> = generate(arg, env);
        args += "<arg_id>";
    }

    env["return_call"] = id;
    env["return_args"] = args;
    env["return_ret"] = ret;
  }
  return <env, "">;
}

public Result generate(\abort_block(EventDefStatement call), Env env)
{
  if (\event(str ret, str id, list[Argument] arguments, list[EventDefComponent] _) := call) {
    list[str] args = [];
    for (arg <- arguments) {
      if (\arg(str _, str arg_id) := arg)
      // <_, a> = generate(arg, env);
        args += "<arg_id>";
    }

    env["abort_call"] = id;
    env["abort_args"] = args;
    env["abort_ret"] = ret;
  }
  return <env, "">;
}

public Result generate(\error_block(EventDefStatement call), Env env)
{
  if (\event(str ret, str id, list[Argument] arguments, list[EventDefComponent] _) := call) {
    list[str] args = [];
    for (arg <- arguments) {
      if (\arg(str _, str arg_id) := arg)
      // <_, a> = generate(arg, env);
        args += "<arg_id>";
    }

    env["error_call"] = id;
    env["error_args"] = args;
    env["error_ret"] = ret;
  }
  return <env, "">;
}

public Result generate(\in_block(EventDefStatement call), Env env)
{
  map[str,EventTpl] evs = ();
  map[str, str] dependencies = ();

  if (\event(str ret, str id, list[Argument] arguments, list[EventDefComponent] _) := call) {
    evs[id] = \etpl(id, [a | arg <- arguments, <_, str a> := generate(arg, env)], ret);
    if ("accepts" notin env)
      env["accepts"] = evs;
    else if (map[str,EventTpl] accepts := env["accepts"])
      env["accepts"] = accepts + evs;

    // env["accepts_dep"] = dependencies;
  }

  return <env, "">;
}

public Result generate(\out_block(EventDefStatement call), Env env)
{
  map[str,EventTpl] evs = ();
  map[str, str] dependencies = ();

  if (\event(str ret, str id, list[Argument] arguments, list[EventDefComponent] _) := call) {
    evs[id] = \etpl(id, [a | arg <- arguments, <_, str a> := generate(arg, env)], ret);
    if ("emits" notin env)
      env["emits"] = evs;
    else if (map[str,EventTpl] emits := env["emits"])
      env["emits"] = emits + evs;

    // env["emits_dep"] = dependencies;
  }

  return <env, "">;
}

// ============================================================
// Statement
public Result generate(\action(str topic, str msg, list[RosDefStatement] events), Env env)
{
  for (event <- events)
    <env, _> = generate(event, env);

  return <env, "">;
}

public Result generate(\service(str topic, str msg, list[RosDefStatement] events), Env env)
{
  for (event <- events)
    <env, _> = generate(event, env);

  return <env, "">;
}

public Result generate(\topic(str topic, str msg, list[RosDefStatement] events), Env env)
{
  for (event <- events)
    <env, _> = generate(event, env);

  return <env, "">;
}

public Result generate(\ros_def(RosDefStatement statement), Env env)
{
  return generate(statement, env);
}

public Result generate(\tasks_block(list[Flow] flows), Env env) {
  str output = "";

  println("================== Non-main flows =======================");
  map[str, Strategy] fenv = ();
  map[str, list[str]] fenvArgs = ();
  for (flow <- flows, \flow(str name, list[str] args, Strategy strat) := flow) {
    fenv[name] = strat;
    fenvArgs[name] = args;
  }
  env["flows"] = fenv;
  env["all_flow_args"] = fenvArgs;

  println("================== Main flows =======================");
  println("================== NFA =======================");

  <n0, _> = build(fenv["main"], 0, env);
  NFA na = augmentWithTriggerAndResets(n0, env["trigger_call"], env["trigger_args"]);

  println("================== DFA =======================");
  DFA dm = determinize(na);
  dm = minimise(dm);
  // println(toPlantUML(dm));

  println("================== DFAI =======================");
  <diFull, idx> = reindexDFAReachable(dm);

  println("Projecting DFA");
  <diWait, full2wait > = projectToWaitDFAWithMap(diFull);
  println(toPlantUML(diWait));
  output = emitBehaviorWait(diFull, diWait, full2wait, env);

  return <env, output>;
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
public Result generate(\accepts(list[EventDefStatement] events), Env env)
{
  map[str,EventTpl] evs = ();
  map[str, str] dependencies = ();
  for (event <- events) {
    if (\event(str ret, str id, list[Argument] args, list[EventDefComponent] deps) := event) {
      evs[id] = \etpl(id, [a | arg <- args, <_, str a> := generate(arg, env)], ret);
      for (dep <- deps) {
        if (\reset(list[Statement] _) := dep)
          dependencies[id] = "abort";
      }
    }
  }

  env["accepts"] = evs;
  env["accepts_dep"] = dependencies;

  return <env, "">;
}

public Result generate(\emits(list[EventDefStatement] events), Env env)
{
  map[str,EventTpl] evs = ();
  map[str, str] dependencies = ();
  for (event <- events) {
    if (\event(str ret, str id, list[Argument] args, list[EventDefComponent] deps) := event) {
      evs[id] = \etpl(id, [a | arg <- args, <_, str a> := generate(arg, env)], ret);
      for (dep <- deps) {
        if (\depends(str var) := dep)
          dependencies[id] = var;
      }
    }
  }

  env["emits"] = evs;
  env["emits_dep"] = dependencies;

  return <env, "">;
}

// Generic
public Result generate(\if(Expression cond, list[Statement] ifstatements), Env env)
{
  str output = "
    bool cond = <generate(cond, env)>;
    if (cond) {
  ";
  for (s <- ifstatements) {
    output += "    " + generate(s, env);
  }

  output += "    }\n";

  return output;
}

public Result generate(\ifelse(Expression cond, list[Statement] ifstatements, list[Statement] elsestatements), Env env)
{
  str output = "
    bool cond = <generate(cond, env)>;
    if (cond) {
  ";
  for (s <- ifstatements)
  {
    output += "    " + generate(s, env);
  }
  output += "    } else {\n";
  for (s <- elsestatements)
  {
    output += "      " + generate(s, env);
  }
  output += "    }\n";

  return output;
}

public Result generate(\tassign(Assignment assign), Env env)
{
  return generate(assign, env);
}

public Result generate(\tvar_def(VariableStatement var_def), Env env)
{
  return generate(var_def, env);
}

// ============================================================
// EventStatement
public Result generate(\event_call(str obj, str event, list[Expression] arguments), Env env)
{
  list[str] args = [];
  for (arg <- arguments) {
    <_, a> = generate(arg, env);
    args += "<a>";
  }

  env["call"] = event;
  env["member"] = obj;
  env["args"] = args;

  return <env, "<obj>.<event>(<intercalate(",", args)>)">;
}

public Result generate(\task_call(str event, list[Expression] arguments), Env env)
{
  list[str] args = [];
  for (arg <- arguments) {
    <_, a> = generate(arg, env);
    args += "<a>";
  }

  env["call"] = event;
  env["args"] = args;

  return <env, "">;
}

// ============================================================
// VariableStatement
public Result generate(\variable_def(str var_type, str var_name, Expression exp, Expression base), Env env)
{
  return <env, "<var_type> <var_name> = <generate(exp, env)>">;
}

// ============================================================
// EventDefStatement
public Result generate(\event(str return_type, str id, list[Argument] args, list[EventDefComponent] components), Env env)
{
  str output = "";

  // Event definition
  str arg_str = "";
  for (arg <- args) {
    <_, e> = generate(arg, env);
    arg_str += "<e>, ";
  }

  output += "<return_type> <id>(<arg_str[..-2]>);";

  env["event_id"] = id;

  // Components (i.e. behaviour)
  for (component <- components) {
    <env, _> = generate(component, env);
  }

  return <env, output>;
}

// ============================================================
// EventDefComponent
public Result generate(\ros_event(str comm, str topic, str msg), Env env)
{
  return <env, "">;
}

public Result generate(\timeout(int time, str unit, str event), Env env)
{
  return <env, "">;
}

public Result generate(\when(str when), Env env)
{
  str output = "<env[when]>";
  env[when] = output + "      on <env["event_id"]> : {}\n";
  return <env, "">;
}

public Result generate(\oncein(str when, list[Statement] assign), Env env)
{
  str output = "<env[when]>";
  env[when] = output + "      on optional: { <env["event_id"]>; state = State.<when == "mission" ? "Idle": "InMission">; }\n";
  return <env, "">;
}

public Result generate(\reply(str var, str when), Env env)
{
  return <env, "">;
}

public Result generate(\depends(str var), Env env)
{
  return <env, "">;
}


public Result generate(\start(list[Statement] statements), Env env)
{
  str output = "<env["idle"]>";

  list[str] stmts = [];
  for (stmt <- statements) {
    <env, s> = generate(stmt, env);
    stmts += "<s>;\n";
  }

  env["idle"] = output + "      on <env["event_id"]> : {
                         '        <for (s <- stmts){><s><}>
                         '        state = State.InMission;
                         '      }\n";
  return <env, "">;
}

public Result generate(\reset(list[Statement] statements), Env env)
{
  str output = "<env["mission"]>";

  list[str] stmts = [];
  for (stmt <- statements) {
    <env, s> = generate(stmt, env);
    stmts += "<s>;\n";
  }

  env["mission"] = output + "      on <env["event_id"]> : {
                            '        state = State.Idle;
                            '        <for (s <- stmts){><s><}>
                            '      }\n";
  return <env, "">;
}

// ============================================================
// Assignment
public Result generate(\assign(str var, str val), Env env)
{
  return <env, "<var> = <val>">;
}

// ============================================================
// Argument
public Result generate(\arg(str arg_type, str arg_id), Env env)
{
  return <env, "<arg_type> <arg_id>">;
}

public Result generate(\provides(str arg_type, str arg_id), Env env)
{
  return <env, "">;
}

public Result generate(\requires(str arg_type, str arg_id), Env env)
{
  return <env, "">;
}

// ============================================================
// Task
public void createInterface(str id, list[Argument] arguments, list[Statement] statements, str ttype, Env env)
{
  // Make sure the output directory exists
  generateDir(OUTPUT_DIR);

  // Create the file for this interface
  loc filename = OUTPUT_DIR + "<toInterface(id)>.dzn";
  touch(filename);

  // Make a copy of the env so we don't modify it
  lenv = env;
  for (s <- statements, \tasks_block(_) !:= s)
    <lenv, _> = generate(s, lenv);

  map[str, str] args = ();
  for (a <- arguments) {
    if (\arg(str arg_type, str arg_id) := a)
      args[arg_id] = "<arg_type> <arg_id>";
  }

  str argToString(list[str] formals) {
    str output = "";
    for (a <- formals)
      if (a in args)
        output += "<args[a]>, ";
      else
        output += "<a>, ";

    return output[..-2];
  }

  str callToString(str id, str portType) {
    str call = "<id>_call";
    str args = "<id>_args";
    str ret = "<id>_ret";

    if (call in lenv && size("<lenv[call]>") > 0)
      return "<portType> <lenv[ret]> <lenv[call]>(<argToString(lenv[args])>);";

    if (id == "accepts" || id == "emits") {
      if (id in lenv && size("<lenv[id]>") > 0 && map[str,EventTpl] methods := lenv[id]) {
        str output = "";
        for (m <- methods) {
          output += "  <portType> <methods[m].ret> <methods[m].name>(<argToString(methods[m].args)>);\n";
        }
        return output[..-1];
      }
    }

    return "";
  }

  str emits = "";
  str accepts = "";
  str variables = "";
  map[str, str] emits_deps = ();
  map[str, str] accepts_deps = ();

  if ("emits_dep" in lenv && map[str, str] deps := lenv["emits_dep"])
    emits_deps = deps;
  if ("accepts_dep" in lenv && map[str, str] deps := lenv["accepts_dep"])
    accepts_deps = deps;

  if ("emits" in lenv && map[str,EventTpl] methods := lenv["emits"]){
    for (m <- methods) {
      str guard = "";
      if (m in emits_deps) {
        guard = "guard_<emits_deps[m]>";
        variables += "bool <guard> = false;\n";
      }

      emits += "      <isEmpty(guard) ? "" : "[<guard>] ">on inevitable: { <m>;<isEmpty(guard) ? "" : " <guard> = false;"> }\n";
    }
  }

  if ("accepts" in lenv && map[str,EventTpl] methods := lenv["accepts"]){
    for (m <- methods){
      str guard = "";
      if (m in range(emits_deps)) {
        guard = " guard_<m> = true; ";
      } else if (m in accepts_deps && accepts_deps[m] == "abort") {
        for (n <- emits_deps)
          guard += " guard_<emits_deps[n]> = false;";
      }

      if (methods[m].ret == "bool"){
        for(r <- ["true","false"]){
          accepts += "       on <m>: { <guard>reply(<r>); }\n";
        }
      } else {
        accepts += "      on <m>: {<guard>}\n";
      }
    }
  }

  str output = "";
  if (ttype == "sync") {
    output = trim("
import types.dzn;

interface <toInterface(id)>
{
  <callToString("trigger", "in")>
<callToString("accepts", "in")>
<callToString("emits", "out")>

  behavior {
    enum State { Idle };
    State state = State.Idle;
    <variables>
    [state.Idle] {
      <if ("trigger_call" in lenv){>on <lenv["trigger_call"]>: {}<}>
<accepts[..-1]>
<emits[..-1]>
    }
  }
}");
  } else {
    output = trim("
import types.dzn;

interface <toInterface(id)>
{
  <callToString("trigger", "in")>
  <callToString("abort", "in")>
  <callToString("return", "out")>
  <callToString("error", "out")>

  behavior {
    enum State { Idle, Busy };
    State state = State.Idle;

    [state.Idle] {
      on <lenv["trigger_call"]>: { state = State.Busy; }
      <if("abort_call" in lenv && size("<lenv["abort_call"]>") > 0){>on <lenv["abort_call"]>: {} <}>
    }

    [state.Busy] {
      <if("abort_call" in lenv && size("<lenv["abort_call"]>") > 0)  {>on <lenv["abort_call"]>: { state = State.Idle; } <}>
      <if("return_call" in lenv && size("<lenv["return_call"]>") > 0){>on inevitable: { <lenv["return_call"]>; state = State.Idle; } <}>
      <if("error_call" in lenv && size("<lenv["error_call"]>") > 0)   {>on inevitable: { <lenv["error_call"]>; state = State.Idle; } <}>

      on <lenv["trigger_call"]>: {}
    }
  }
}");
  }

  writeFile(filename, output);
}

public void createComponent(str id)
{
  // Make sure the output directory exists
  generateDir(OUTPUT_DIR);

  // Create the file for this interface
  loc filename = OUTPUT_DIR + "a_<toComponent(id)>.dzn";
  touch(filename);

  str output = trim("
import <toInterface(id)>.dzn;

component <toComponent(id)>
{
  provides <toInterface(id)> <uncapitalize(id)>;
}
  ");

  writeFile(filename, output);
}

public EventTpl createCall(str id, Env env) {
  str call = "<id>_call";
  str args = "<id>_args";

  if (call notin env)
    return empty_etpl();
  else if (args notin env)
    return etpl(env[call], [], "");

  return etpl(env[call], env[args], "");
}

public map[str,EventTpl] createCallMap(str id, Env env) {
  if (id in env && map[str,EventTpl] a := env[id])
    return a;

  return ();
}

public Result generate(\capability(str id, list[Argument] args, list[Statement] tstatements), Env env) {
  println("Generating capability: <id>");

  // Make sure the output directory exists
  generateDir(OUTPUT_DIR);

  // Create the file for this component
  loc filename = OUTPUT_DIR + "<toInterface(id)>.dzn";
  touch(filename);

  lenv = env;
  str ttype = "sync";
  for (s <- tstatements) {
    if (\action(_, _, _) := s)
      ttype = "async";

    <lenv, _> = generate(s, lenv);
  }

  list[str] arguments = [];
  for (a <- args) {
    <lenv, e> = generate(a, lenv);
    arguments += e;
  }

  createInterface(id, args, tstatements, ttype, env);
  createComponent(id);

  tenv[id] = taskDef(id, arguments, ttype,
                      createCall("trigger", lenv),
                      createCall("return", lenv),
                      createCall("error", lenv),
                      createCall("abort", lenv),
                      createCallMap("accepts", lenv),
                      createCallMap("emits", lenv));

  return <env, "">;
}

public Result generate(\task(str id, list[Argument] args, list[Statement] tstatements), Env env) {
  println("Generating component task: <id>");

  // Make sure the output directory exists
  generateDir(OUTPUT_DIR);

    // Create the file for this component
  loc filename = OUTPUT_DIR + "<toComponent(id)>.dzn";
  touch(filename);

  // Include interfaces
  str importString = "import types.dzn;\n";

  // Create internal environment
  lenv = env;

  str componentsBody = "";
  str components = "    <toComponent(id)> <uncapitalize(id)>;\n";
  str importComponents = "import types.dzn;\n";

  str requires = "";
  for (req <- args, \requires(str arg_type, str arg_name) := req) {
    importString += "import <toInterface(arg_type)>.dzn;\n";
    requires += "  requires <toInterface(arg_type)> <arg_name>;\n";
    lenv[arg_name] = arg_type;

    importComponents += "import a_<toComponent(arg_type)>.dzn;\n";
    components += "    <toComponent(arg_type)> <arg_name>;\n";
    componentsBody += "    <uncapitalize(id)>.<arg_name> \<=\> <arg_name>.<uncapitalize(arg_type)>;\n";
  }

  str provides = "";
  for (pro <- args, \provides(str arg_type, str arg_name) := pro) {
    importString += "import <toInterface(arg_type)>.dzn;\n";
    provides += "  provides <toInterface(arg_type)> <arg_name>;\n";

    importComponents += "import a_<toComponent(arg_type)>.dzn;\n";
    components += "    <toComponent(arg_type)> <arg_name>;\n";
  }

  // Create the provided interface
  createInterface(id, args, tstatements, "async", lenv);

  for (c <- tstatements, \tasks_block(_) !:= c)
    <lenv, _> += generate(c, lenv);

  // Add state 0 to the graph
  str body = "";
  for (c <- tstatements, \tasks_block(_) := c)
    <lenv, body> += generate(c, lenv);

  if (USE_ALARM) {
    requires += "  requires ialarm alarm;\n";
    importString += "import ialarm.dzn;\n";
    components += "    calarm alarm;\n";

    importComponents += "import a_calarm.dzn;\n";
    componentsBody += "    <uncapitalize(id)>.alarm \<=\> alarm.api;\n";

    generateAlarm();
  }

  str output = trim("
<importString>
// Always provided
import <toInterface(id)>.dzn;

component <toComponent(id)>
{
  provides <toInterface(id)> <API>;
<requires[..-1]>

<body>
}");

  writeFile(filename, output);

  // System file
  // api is hard-coded, but it actually allows us to nest component tasks if needed.
  str output_system = trim("
<importComponents>
// Always provided
import <toComponent(id)>.dzn;
import <toInterface(id)>.dzn;

component <toComponent(id)>_task
{
  provides <toInterface(id)> <API>;
  system
  {
<components>
    <API> \<=\> <uncapitalize(id)>.<API>;
<componentsBody>
  }
}");

  loc system_filename = OUTPUT_DIR + "<toComponent(id)>_task.dzn";
  touch(system_filename);
  writeFile(system_filename, output_system);

  return <env, "">;
}

private void generateAlarm() {
  // Create alarm component
  loc component_filename = OUTPUT_DIR + "a_calarm.dzn";
  touch(component_filename);
  str coutput = trim("
// Always provided
import ialarm.dzn;

component calarm
{
  provides ialarm <API>;
}");

  writeFile(component_filename, coutput);

  // Create alarm interface
  loc interface_filename = OUTPUT_DIR + "ialarm.dzn";
  touch(interface_filename);

  str ioutput = trim("
import types.dzn;

interface ialarm
{
  in void set(int millis);
  in void reset();
  out void timeout();

  behavior {
    enum State { Idle, Running };
    State state = State.Idle;

    [state.Idle] {
      on set: { state = State.Running; }
      on reset: {}
    }

    [state.Running] {
      on set: {}
      on reset: { state = State.Idle; }
      on optional: { timeout; state = State.Idle; }
    }
  }
}");

  writeFile(interface_filename, ioutput);
}

// =============================================================
// Base statements
public int generate(koda::AST::System system)
{
  createExternalTypes();

  for (t <- system.components) {
    if (\capability(_, _, _) := t)
      generate(t, genv);
  }

  for (t <- system.components) {
    if (\task(_, _, _) := t)
      generate(t, genv);
  }

  return 0;
}

public int generate(loc source, loc output_dir)
{
  OUTPUT_DIR = output_dir;

  src = koda::Parser::parsekoda(source);
  ast = koda::CST2AST::cst2ast(src);

  // koda::ROSGenerator::generate(ast, OUTPUT_DIR);

  return generate(ast);
}
