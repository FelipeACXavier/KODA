module koda::Plugin

import IO;
import ParseTree;

import util::Reflective;
import util::IDEServices;
import util::LanguageServer;

import koda::Server;
import koda::Parser;
// import koda::CST2AST;
// import koda::Check;

/*
 * This function is defined to test the functionality of the whole assignment. It receives a file path as a parameter and returns true if the program satisfies the specification or false otherwise.
 * First, it calls the parser (Parser.rsc). Then, it transforms the resulting parse tree of the previous program and calls the function cst2ast (CST2AST.rsc), responsible for transforming a parse tree into an abstract syntax tree.
 * Finally, the resulting AST is used to evaluate the well-formedness of the labour program using the check function (Check.rsc).
 */
bool checkWellformedness(loc fil) {
  // Parsing
  // resource = parse(fil);
  // Transform the parse tree into an abstract syntax tree
  // ast = cst2ast(resource, false);
  // Check the well-formedness of the program
  return true; // checkBoulderWallConfiguration(ast);
}

/*
 * This is the main function of the project. This function enables the editor's syntax highlighting.
 * After calling this function from the terminal, all files with extension .labour will be parsed using the parser defined in module koda::Parser.
 * If there are syntactic errors in the program, no highlighting will be shown in the editor.
 */

int main() {
  // we register a new language to Rascal's LSP multiplexer
  // the multiplexer starts a new evaluator and loads this module and function
  registerLanguage(
    language(
      pathConfig(srcs=[|project://koda/src|]),
      "System",                 // name of the language
      "kd",                    // extension, e.g., example.lbr
      "koda::Server",         // module to import, this one
      "contributions"
    )
  );
  return 0;
}

/*
 * Use this function to clear all traces of your language from VS code.
 */
void clearLanguage() {
  unregisterLanguage("System", {"kd"});
}

/*
 * Use this function to run your tests. It will show how many tests succeeded.
 * Note that an invalid test "succeeds" when the check fails
 */
void runTests() {
  fails = 0;
  validFiles = |project://koda/test/valid|.ls;
  invalidFiles = |project://koda/test/invalid|.ls;

  println("\nValid tests");
  for (file <- validFiles) {
    if (checkWellformedness(file)) {
      println("SUCCESS: <file.path> returns true");
    } else {
      println("FAILURE: <file.path> returns false");
      fails += 1;
    }
  }

  println("\nInvalid tests");

  for (file <- invalidFiles) {
    try {
      if (checkWellformedness(file)) {
        println("FAILURE: <file.path> returns true");
        fails +=1;
      } else {
        println("SUCCESS: <file.path> returns false");
      }
    } catch e: {
      println("Intended error behaviour due to cst that does not match syntax: <e> (type: <typeOf(e)>)");
      println("SUCCESS: <file.path> errors out as expected");
    }
  }

  println("<fails> failed tests");
}
