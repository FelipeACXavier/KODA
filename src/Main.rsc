module Main

import IO;
import String;
import List;
import Location;
import Exception;

import koda::Generator;

int main(list[str] args){
  // By default, generate the valid tests
  if (size(args) < 2) {
    println("Not enough arguments provided");
    return -1;
  }

  for (arg <- args) println("Args: <arg>");

  loc input = locFromUnixPath("<args[0]>");
  loc output = locFromUnixPath("<args[1]>");
  str generationType = "ros";
  if (size(args) > 2)
    generationType = "<args[2]>";

  println(input);
  println(output);

  // Generate test files
  return koda::Generator::generate(input, output, generationType);
}