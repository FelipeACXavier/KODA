module koda::Automaton

import IO;
import Set;
import Map;
import List;
import String;
import Relation;

public alias Edge = rel[State,Label,State];

// =============================================================
// Data formats
data State
  = sid(int id)              // atomic state id
  | spair(State l, State r)  // product-state for parallel
  ;

data Label
  = act(str name, list[str] args, str ret)
  | sig(str source, str event, list[str] args)
  | actm(str task, str method, list[str] args, str ret)
  | abrt(str name, list[str] args)
  | eps() // ε for silent wiring
  ;

data OutCall
  = mcall(str task, str method, list[str] args, str ret)
  | ocall(str task, str obj, list[str] args, str ret)
  | oabort(str task, list[str] args)
  ;

data NFA
  = nfa(set[State] S, State s0, set[State] F, rel[State, Label, State] T);

data NFAx
  = nfax(NFA m, set[State] resets);

data DFA
  = dfa(set[set[State]] S, set[State] s0, set[set[State]] F, rel[set[State],Label,set[State]] T)
  | empty_dfa()
  ;

// A compact, integer-indexed DFA for codegen
data DFAI = dfai(set[int] S, int s0, set[int] F, rel[int, Label, int] T);

// =============================================================
// Pretty
private str showArgs(list[str] as, bool hideArgs) =
  hideArgs ? "" : "(" + intercalate(", ", as) + ")";

private str showLabel(Label l, bool hideArgs, bool bangForOutputs) {
  switch (l) {
    case sig(str src, str ev, list[str] as):
      return src + "." + ev + showArgs(as, hideArgs);
    case abrt(str t, list[str] as):
      return (bangForOutputs ? "!" : "") + t + showArgs(as, hideArgs);
    case act(str t, list[str] as, str ret):
      return (bangForOutputs ? "!" : "") + (isEmpty(ret) ? "" : "<ret> = ") + t + showArgs(as, hideArgs);
    case actm(str t, str m, list[str] as, str ret):
      return (bangForOutputs ? "!" : "") + (isEmpty(ret) ? "" : "<ret> = ") + t + "." + m + showArgs(as, hideArgs);
    default:
      return "ε";
  }
}

private str showState(State s) {
  switch (s) {
    case sid(i): return "s<i>";
    case spair(l, r): return "<showState(l)>_<showState(r)>";
    default: return "";
  }
}

private str esc(str s) = replaceAll(s, "\"", "\\\"");

// =============================================================
// 0 Helpers
private bool isEps(Label l) = l == eps();

private set[Label] sigma(NFA m) = { l | <_,l,_> <- m.T, !isEps(l) };

private set[State] step(Edge T, State s, Label a) = { t | <s, a, t> <- T };
private set[State] stepAll(Edge T, set[State] S, Label a) = union({ step(T, s, a) | s <- S });

private set[State] epsClosure(Edge T, State s) {
  set[State] seen = {s};
  list[State] todo = [s];

  while (!isEmpty(todo)) {
    State x = head(todo); todo = tail(todo);
    for (<State a, Label l, State y> <- T) {
      if (a == x && isEps(l) && !(y in seen)) { seen += {y}; todo += [y]; }
    }
  }
  return seen;
}

private map[State,set[State]] epsClosures(NFA m) = ( s : epsClosure(m.T, s) | s <- m.S );

// =============================================================
// 1 - Trim (reachability + co-reachability)
public NFA trimNFA(NFA m) {
  // forward reachability
  set[State] fwd = reachFrom(m.s0, m.T);

  // backwards from finals (on reversed graph)
  rel[State,Label,State] Tr = { <b,l,a> | <a,l,b> <- m.T };
  set[State] back = {};
  for (State f <- m.F) back += reachFrom(f, Tr);

  set[State] keep = size(back) == 0 ? fwd : fwd & back;
  return nfa(
    keep,
    m.s0,
    m.F & keep,
    { <a,l,b> | <a,l,b> <- m.T, a in keep, b in keep }
  );
}

public DFA trimDFA(DFA d) {
  set[set[State]] seen = { d.s0 };
  list[set[State]] q = [ d.s0 ];

  while (!isEmpty(q)) {
    set[State] x = head(q); q = tail(q);
    for (<set[State] a, Label _, set[State] b> <- d.T) {
      if (a == x && !(b in seen)) {
        seen += { b };
        q += [ b ];
      }
    }
  }

  return dfa(seen, d.s0, d.F & seen, { <a,l,b> | <set[State] a, Label l, set[State] b> <- d.T, a in seen, b in seen } );
}

// Helpers
public set[State] reachFrom(State s0, Edge T) {
  set[State] seen = {s0};
  list[State] todo = [s0];

  while (!isEmpty(todo)) {
    State x = head(todo);
    todo = tail(todo);
    for (<State a, Label _, State y> <- T) {
      if (a == x && !(y in seen)) {
        seen += {y};
        todo += [y];
      }
    }
  }
  return seen;
}

// =============================================================
// 2 - ε-elimination (Kept states; rewires edges via ε-closures)
public NFA removeEpsilons(NFA m) {
  map[State,set[State]] EC = epsClosures(m);

  // finals: any state whose ε-closure hits an original final
  set[State] Fp = { p | p <- m.S, !isEmpty(EC[p] & m.F) };

  // transitions: for each p, for each x in EC[p], for each non-ε edge x -a-> y,
  // add p -a-> q for all q in EC[y]
  rel[State,Label,State] Tp =
    { <p, a, q>
      | p <- m.S,
        x <- EC[p],
        <State z, Label a, State y> <- m.T,
        z == x, !isEps(a),
        q <- EC[y]
    };

  return nfa(m.S, m.s0, Fp, Tp);
}

// =============================================================
// 3 - Determinisation (subset construction)
public DFA determinize(NFA nfa0) {
  NFA m = removeEpsilons(trimNFA(nfa0));    // robust start
  set[Label] sig = sigma(m);

  set[set[State]] S = {};
  set[set[State]] F = {};
  rel[set[State],Label,set[State]] T = {};

  set[State] q0 = { m.s0 };
  list[set[State]] work = [ q0 ];
  set[set[State]] seen = { q0 };

  while (!isEmpty(work)) {
    set[State] Q = head(work); work = tail(work);
    S += { Q };
    if (!isEmpty(Q & m.F)) F += { Q };

    for (Label a <- sig) {
      set[State] dest = stepAll(m.T, Q, a);
      if (!isEmpty(dest)) {
        T += { <Q, a, dest> };
        if (!(dest in seen)) { seen += {dest}; work += [dest]; }
      }
    }
  }
  return dfa(S, q0, F, T);
}

// =============================================================
// 4 - DFA minimisation (partition refinement)
public DFA minimise(DFA d) {
  d = dfaTrim(d);                      // drop unreachable DFA states

  set[Label] alphabet = { l | <_,l,_> <- d.T };   // alphabet in this DFA

  // Initial partition: finals vs non-finals
  list[set[set[State]]] P = [];
  set[set[State]] nonFinal = d.S - d.F;
  if (!isEmpty(d.F)) P += [ d.F ];
  if (!isEmpty(nonFinal)) P += [ nonFinal ];

  // Refine until stable
  bool changed = true;
  while (changed) {
    changed = false;
    list[set[set[State]]] Pnew = [];
    for (set[set[State]] B <- P) {
      // group states in B by their transition signatures
      map[list[int], set[set[State]]] groups = ();
      for (set[State] s <- B) {
        list[int] sig = [ blockId(next(d, s, a), P) | Label a <- alphabet ];
        groups[sig] = (sig in groups) ? groups[sig] + {s} : {s};
      }
      Pnew += [ groups[k] | k <- domain(groups) ];
      if (size(groups) > 1) changed = true;
    }
    P = Pnew;
  }

  // Map each block to a representative DFA state (one element of the block)
  set[State] pickRep(set[set[State]] B) = head(toList(B)); // deterministic enough
  map[set[set[State]], set[State]] rep = ( B : pickRep(B) | B <- P );

  // For looking up the block of an original DFA state
  map[set[State], set[set[State]]] blockOf = ( s : B | B <- P, s <- B );

  // Build quotient DFA using representatives
  set[set[State]] S2 = { rep[B] | B <- P };
  set[set[State]] F2 = { rep[B] | B <- P, size(B & d.F) > 0 };
  set[State] s0p     = rep[ blockOf[d.s0] ];

  rel[set[State],Label,set[State]] T2 =
    { <rep[B], a, rep[ blockOf[t] ]>
      | B <- P,
        Label a <- alphabet,
        set[State] r := rep[B],
        set[State] t := next(d, r, a),
        !isEmpty(t)
    };

  DFA dm = dfa(S2, s0p, F2, T2);
  return dfaTrim(dm);
}

// Helpers
private int blockId(set[State] s, list[set[set[State]]] P) {
  if (isEmpty(s)) return -1;
  for (int i <- [0 .. size(P)]) if (s in P[i]) return i;
  return -1;
}

private set[State] next(DFA d, set[State] s, Label a) {
  set[set[State]] outs = { t | <s,a,t> <- d.T };
  return isEmpty(outs) ? {} : head(toList(outs));
}

private DFA dfaTrim(DFA d) {
  set[set[State]] seen = { d.s0 }; list[set[State]] todo = [ d.s0 ];
  while (!isEmpty(todo)) {
    set[State] x = head(todo); todo = tail(todo);
    for (<set[State] a, Label l, set[State] y> <- d.T) {
      if (a == x && !(y in seen)) { seen += {y}; todo += [y]; }
    }
  }
  return dfa(
    seen,
    d.s0,
    d.F & seen,
    { <a,l,b> | <a,l,b> <- d.T, a in seen, b in seen }
  );
}

// =============================================================
// 5 - Optional: reindex (pretty names)
public tuple[NFA,map[State,int]] reindex(NFA m) {
  list[State] order = toList(m.S);
  map[State,int] id = ( s : i | i <- [0..size(order)], s := order[i] );
  set[State] S2 = { sid(id[s]) | s <- m.S };
  State s02 = sid(id[m.s0]);
  set[State] F2 = { sid(id[s]) | s <- m.F };
  rel[State,Label,State] T2 = { <sid(id[a]), l, sid(id[b])> | <a,l,b> <- m.T };
  return <nfa(S2, s02, F2, T2), id>;
}

// =============================================================
// 6 - To plantUML for visualization
// ---------- NFA -> PlantUML ----------
public str toPlantUML(NFA m, bool showEps = true, bool hideArgs = false, bool bangForOutputs = true) {
  // stable ordering for ids
  list[State] order = sort(toList(m.S), bool(State a, State b) {
    return "<a>" < "<b>"; });

  map[State,str] id = ( s : "S<i>" | i <- [0 .. size(order)], s := order[i]);

  str out = "@startuml\n"
          + "hide empty description\n\n";

  // declare states with readable labels
  for (s <- order) {
    out += "state \"" + esc(showState(s)) + "\" as " + id[s] + "\n";
  }
  out += "\n";

  // initial arrow
  out += "[*] --\> " + id[m.s0] + "\n";

  // transitions
  for (<State a, Label l, State b> <- m.T) {
    if (!showEps && l == eps())
      continue;
    out += id[a] + " --\> " + id[b] + " : " + esc(showLabel(l, hideArgs, bangForOutputs)) + "\n";
  }

  // finals -> [*]
  for (f <- m.F) {
    out += id[f] + " --\> [*]\n";
  }

  out += "@enduml";
  return out;
}

public void printT(rel[State,Label,State] T, State curr, str indent) {
  for (t <- T, <State p, Label l, State n> := t) {
    if (p == curr) {
      println("<indent><p> -- <l> --\> <n>");
      printT(T, n, indent + " ");
    }
  }
}

public void printNFA(NFA m) {
  println("nfa");
  println("  S: <intercalate(", ", [showState(s) | s <- m.S ])>");
  println(" s0: <m.s0>");
  println("  F: <intercalate(", ", [showState(s) | s <- m.F ])>");
  println("  T:");

  if (<State s0, Label _, State _> := head(sort(m.T)))
    printT(m.T, s0, "   ");
}
// ---------- DFA -> PlantUML ----------
private str showDfaState(set[State] q) {
  if (isEmpty(q)) return "∅";
  list[str] parts = sort([ showState(s) | s <- q ]);
  return "{" + intercalate(", ", parts) + "}";
}

public str toPlantUML(DFA d, bool hideArgs = false, bool bangForOutputs = true) {
  // stable ordering for ids
  list[set[State]] order = sort(toList(d.S), bool(set[State] a, set[State] b) {
    return "<a>" < "<b>"; });

  map[set[State],str] id = ( q : "Q<i>" | i <- [0 .. size(order)], q := order[i] );

  str out = "@startuml\n"
          + "hide empty description\n\n";

  for (q <- order) {
    out += "state \"" + esc(showDfaState(q)) + "\" as " + id[q] + "\n";
  }
  out += "\n";

  out += "[*] --\> " + id[d.s0] + "\n";

  for (<set[State] a, Label l, set[State] b> <- d.T) {
    out += id[a] + " --\> " + id[b] + " : " + esc(showLabel(l, hideArgs, bangForOutputs)) + "\n";
  }

  for (q <- d.F) {
    out += id[q] + " --\> [*]\n";
  }

  out += "@enduml";
  return out;
}

// ---------- DFAI -> PlantUML ----------
private str edgeArrow(Label l) = (l is sig) ? "--\>" : "..\>";

public str toPlantUML(DFAI d,
                      bool hideArgs = false,        // hide argument lists in labels
                      bool bangForOutputs = true,   // prefix outputs with '!' for clarity
                      bool showLegend = true) {

  // stable state ordering: S0 first, then ascending
  list[int] order = [ d.s0 ]
    + sort([ q | q <- toList(d.S), q != d.s0 ], bool(int a,int b){ return a < b; });

  str out = "@startuml\n"
          + "hide empty description\n\n";

  // declare states
  for (int q <- order) {
    out += "state S<q>\n";
  }
  out += "\n";

  // initial arrow
  out += "[*] --\> S<d.s0>\n";

  // transitions
  for (<int a, Label l, int b> <- d.T) {
    // only draw transitions between known (reachable) states
    if (!(a in d.S) || !(b in d.S))
      continue;

    str lab = esc(showLabel(l, hideArgs, bangForOutputs));
    out += "S<a> <edgeArrow(l)> S<b> :<lab>\n";
  }

  // finals (if you use them)
  for (int f <- d.F) {
    if (f in d.S) out += "S<f> --\> [*]\n";
  }

  if (showLegend) {
    out += "\nlegend left\n";
    out += "  solid  = input (sig)\n";
    out += "  dotted = output (call/act)\n";
    out += "endlegend\n";
  }

  out += "@enduml";
  return out;
}
// =============================================================
// 7 - Clean up for codegen
// --------- BFS ordering (reachable first; deterministic tie-breaker) ---------
private list[set[State]] bfsOrder(DFA d) {
  set[set[State]] seen = { d.s0 };
  list[set[State]] order = [];
  list[set[State]] q = [ d.s0 ];

  // Deterministic neighbour iteration
  list[set[State]] nexts(set[State] x) =
    sort(toList({ b | <set[State] a, Label _, set[State] b> <- d.T, a == x }),
         bool(set[State] u, set[State] v) { return "<u>" < "<v>"; });

  while (!isEmpty(q)) {
    set[State] x = head(q); q = tail(q);
    order += [x];
    for (set[State] y <- nexts(x)) {
      if (!(y in seen)) { seen += {y}; q += [y]; }
    }
  }

  // If anything unreachable remains (shouldn't after trimming), append in stable order
  set[set[State]] rest = d.S - toSet(order);
  if (!isEmpty(rest)) {
    list[set[State]] tailOrd =
      sort(toList(rest), bool(set[State] u, set[State] v) { return "<u>" < "<v>"; });
    order += tailOrd;
  }

  return order;
}

// --------- Reindex to integers ---------
public tuple[DFAI, map[set[State], int]] reindexDFA(DFA d) {
  list[set[State]] order = bfsOrder(d);

  // map each original DFA state (subset) to a dense int 0..n-1
  map[set[State], int] id = ( q : i | i <- [0 .. size(order)], q := order[i] );

  set[int] S2 = { id[q] | q <- d.S };
  int s02     = id[d.s0];
  set[int] F2 = { id[q] | q <- d.F };
  rel[int, Label, int] T2 =
    { < id[a], l, id[b] > | <set[State] a, Label l, set[State] b> <- d.T };

  return < dfai(S2, s02, F2, T2), id >;
}

public tuple[DFAI, map[set[State], int]] reindexDFAReachable(DFA d) {
  // BFS from start over d.T
  set[set[State]] seen = { d.s0 };
  list[set[State]] order = [];
  list[set[State]] q = [ d.s0 ];

  list[set[State]] nexts(set[State] x) =
    sort(toList({ b | t <- d.T, <x, _, set[State] b> := t }),
         bool(set[State] u, set[State] v) { return toString(u) < toString(v); });

  while (!isEmpty(q)) {
    set[State] x = head(q);
    q = tail(q);
    order += [ x ];

    for (set[State] y <- nexts(x)) {
      if (!(y in seen)) {
        seen += {y};
        q += [y];
      }
    }
  }

  // Map only the reachable subset
  map[set[State], int] id = ( order[i] : i | i <- [0 .. size(order)]);

  set[int] S2 = { id[a] | a <- toSet(order) };
  int s02     = id[d.s0];
  set[int] F2 = { id[a] | a <- (d.F & toSet(order)) };
  rel[int, Label, int] T2 =
    { < id[a], l, id[b] >
      | <set[State] a, Label l, set[State] b> <- d.T, a in toSet(order), b in toSet(order)
    };

  return < dfai(S2, s02, F2, T2), id >;
}

// Optional: get human-friendly names for codegen (e.g., "S0", "S1", …)
public map[set[State], str] dfaNames(map[set[State], int] id) = ( q : "S<id[q]>" | q <- domain(id) );
