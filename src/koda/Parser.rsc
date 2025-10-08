module koda::Parser

import ParseTree;
import IO;

import koda::Syntax;
import koda::AST;

public start[System] parsekoda(str input) = parse(#start[System], input);
public start[System] parsekoda(loc filePath) = parse(#start[System], filePath);

// Get NACTG AST
// public koda::AST load(str input) = cst2ast(parseNACTG(input));
// public koda::AST load(loc location) = cst2ast(parseNACTG(location));
// Use implode for now
public koda::AST::System load(str input) = implode(#System, parsekoda(input));
public koda::AST::System load(loc location) = implode(#System, parsekoda(location));

// |project://koda/test/valid/basic.rzn|
