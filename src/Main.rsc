module Main

import IO;
import String;
import List;
import Location;
import Exception;

import koda::Generator;

int main(list[str] args){
  // By default, generate the valid tests
  if (size(args) != 1) {
    println("No argument provided");
    return -1;
  }

  loc input = |project://koda/test/valid/| + args[0];
  loc output = |project://koda/generated/| + Location::locFromUnixPath(args[0]).filename;

  println(input);
  println(output);

  // Generate test files
  // return koda::Generator::generate();
  return 0;
}