module koda::Syntax

// ===========================================================================
// Comment and whitespace parsing
layout Layout = WhitespaceOrComment* !>> [\ \t\n\f\r] !>> "//" !>> "/*";

syntax WhitespaceOrComment
  = whitespace: Whitespace
  | comment: Comment
  ;

lexical Whitespace
  = [\u0009-\u000D \u0020 \u0085 \u00A0 \u1680 \u180E \u2000-\u200A \u2028 \u2029 \u202F \u205F \u3000];

syntax Comment
  = LineComment
  | CStart CommentChar* CEnd
  ;

lexical LineComment
  = @category="Comment" "//"  ![\n]* $;

syntax CStart = @category="Comment" "/*";
syntax CEnd = @category="Comment" "*/";

syntax CommentChar
  = @category="Comment" ![*{}\ \t\n\f\r]
  | @category="Comment" [*] !>> [/]
  | @category="Comment" Embed
  ;

syntax Embed
  = TopLevelComponent
  ;

// ===========================================================================
// Actual language grammar
keyword Keywords
    = "end"
    | "repeat"
    | "join"
    | "either"
    | "on error"
    | "on abort"
    ;

lexical Natural = [0-9]+ !>> [0-9];
lexical Real = [0-9]+[.][0-9]+ !>> [0-9];
lexical Ident = ([a-zA-Z_\-:$][a-zA-Z0-9_\-:$]* !>> [a-zA-Z0-9_\-$]) \ Keywords;
lexical UnicodeEscape
    = utf16: "\\" [u] [0-9 A-F a-f] [0-9 A-F a-f] [0-9 A-F a-f] [0-9 A-F a-f]
    | utf32: "\\" [U] [0-9 A-F a-f] [0-9 A-F a-f] [0-9 A-F a-f] [0-9 A-F a-f] [0-9 A-F a-f] [0-9 A-F a-f]
    ;
lexical StringChar
    = ![\" \\]                 // " or \ are invalid unless they are preceed by \
    | "\\" [\" \\ / b f n r t] // Thus, these are valid \" \\ \/ \b \f \n \r \t
    | UnicodeEscape
    ;
lexical String = "\"" StringChar* "\"";
lexical Any = "\<(" ![$]* ")\>";

start syntax System
  = TopLevelComponent*
  ;

syntax TopLevelComponent
  = @Foldable "task"  Ident "(" {Argument ","}* ")" "{" Statement* statements "}"
  | @Foldable "capability" Ident "(" {Argument ","}* ")" "{" Statement* statements "}"
  ;

syntax Argument
  = Ident Ident
  | Ident "req" Ident
  | Ident "pro" Ident
  ;

syntax Flow
  = \flow: Ident ("[" {Ident ","}* "]")? ":" Strategy ";"
  ;

syntax EventStatement
  = Ident "." Ident "(" {Expression ","}* ")"
  | Ident "(" {Expression ","}* ")"
  ;

syntax StrategyHandler
 = \error: "on error" Strategy
 | \abort: "on abort" Strategy
 | \emitter: "on" EventStatement Strategy
 ;

syntax Strategy
  = left (
      left \join: "join" "(" {Strategy "|"}+ ")"
    | left \either: "either" "(" {Strategy "|"}+ ")"
    | left \seq: Strategy "--\>" Strategy
  )
  | \let: "let" Ident "=" EventStatement
  | \within: "within" Natural "do" Strategy "else" Strategy
  | \ifelse: "if" Expression "then" Strategy ("else" Strategy)?
  | \repeat: "repeat" "(" Strategy ")"
  | \guard: "guard" "{" Expression "}"
  | \every: "every" Natural "{" Strategy "}" StrategyHandler*
  | \end: "end"
  | \ref: Ident
  | \task: EventStatement StrategyHandler*
  | @Foldable bracket "(" Strategy ")"
  ;

syntax DefWithLayout
  = EventDefStatement ";"
  ;

syntax EventDefStatement
  = \event: Ident Ident "(" {Argument ","}* args ")" (":" {EventDefComponent ","}+)?
  ;

syntax RosDefStatement
  = \trigger_block: "trigger" ":" EventDefStatement call ";"
  | \return_block: "return" ":" EventDefStatement call ";"
  | \abort_block: "abort" ":" EventDefStatement call ";"
  | \error_block: "error" ":" EventDefStatement call ";"
  | \in_block: "in" ":" EventDefStatement call ";"
  | \out_block: "out" ":" EventDefStatement call ";"
  ;

syntax VariableStatement
  = \variable_def: Ident var_type Ident var_name "=" Expression ":" Expression
  ;

syntax Statement
  = @Foldable \tasks_block: "strategy" "{" Flow+ "}"
  > @Foldable \variables: "vars" "{" VariableStatement+ "}"
  | @Foldable \action: "action" String id String msg "{" RosDefStatement* "}"
  | @Foldable \service: "service" String id String msg "{" RosDefStatement* "}"
  | @Foldable \topic: "topic" String id String msg "{" RosDefStatement* "}"
  > RosDefStatement
  ;

syntax EventDefComponent
  = \ros_event: ROSData comm ":" String id String msg
  | \timeout: "timeout" Natural Time "-\>" Ident
  | \when: "allowed in" When
  | \reply: "reply" Ident When
  | \depends: "after" Ident
  | \oncein: "once in" When ("{" Statement* "}")?
  | \start: "trigger" ("{" Statement* "}")?
  | \reset: "abort" ("{" Statement* "}")?
  ;

syntax Expression
  = \id: Ident name
  | \str_const: String svalue
  | \int_const: Natural ivalue
  | \float_const: Real fvalue
  | bracket "(" Expression ")"
  > \call: EventStatement
  > non-assoc (
      left \neg: "-" Expression argument
    | left \mul: Expression lhs "*" Expression rhs
    | non-assoc \div: Expression lhs "/" Expression rhs
  )
  > left (
      left \add: Expression lhs "+" Expression rhs
    | left \sub: Expression lhs "-" Expression rhs
  )
  > non-assoc (
      right \not: "!" Expression lhs
    | non-assoc \eq: Expression lhs "==" Expression rhs
    | non-assoc \neq: Expression lhs "!=" Expression rhs
    | non-assoc \leq: Expression lhs "\<=" Expression rhs
    | non-assoc \geq: Expression lhs "\>=" Expression rhs
    | non-assoc \lt: Expression lhs "\<" Expression rhs
    | non-assoc \gt: Expression lhs "\>" Expression rhs
  )
  > left (
      left \conj: Expression lhs "and" Expression rhs
    | left \disj: Expression lhs "or" Expression rhs
  )
  ;

syntax When
  = "always"
  | "mission"
  | "idle"
  ;

syntax Time
  = "s"
  | "ms"
  | "us"
  | "ns"
  ;

syntax ROSData
  = "topic"
  | "service"
  | "action"
  ;
