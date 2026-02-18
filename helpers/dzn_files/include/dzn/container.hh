// dzn-runtime -- Dezyne runtime library
//
// Copyright © 2015, 2016, 2017, 2019, 2020, 2021, 2022 Rutger van Beusekom <rutger@dezyne.org>
// Copyright © 2017, 2018, 2019, 2020, 2021, 2023 Janneke Nieuwenhuizen <janneke@gnu.org>
//
// This file is part of dzn-runtime.
//
// All rights reserved.
//
//
// Commentary:
//
// Code:

#ifndef DZN_CONTAINER_HH
#define DZN_CONTAINER_HH

#include <dzn/config.hh>
#include <dzn/locator.hh>
#include <dzn/runtime.hh>
#include <dzn/pump.hh>

#include <algorithm>
#include <functional>
#include <iostream>
#include <iterator>
#include <map>
#include <queue>
#include <sstream>
#include <string>

namespace dzn
{
template <typename System, typename Function>
struct container: public component
{
  dzn::meta dzn_meta;
  dzn::locator dzn_locator;
  dzn::runtime dzn_runtime;
  System system;

  std::map<std::string, Function> lookup;
  std::queue<std::string> trail;
  dzn::pump pump;

  friend std::ostream &operator << (std::ostream &os, container<System, Function> &)
  {
    return os;
  }

  container ()
    : dzn_meta{"<external>", "container", 0, {},
               {&system.dzn_meta},
               {[this]{dzn::check_bindings (system);}}}
    , dzn_locator ()
    , dzn_runtime ()
    , system (dzn_locator.set (dzn_runtime).set (pump))
    , pump ()
  {
    dzn_locator.get<illegal_handler> ().illegal = [] (char const *location = "")
    {
      std::clog << location << (location[0] ? ":0: " : "")
                << "<illegal>" << std::endl;
      std::exit (0);
    };
    system.dzn_meta.name = "sut";
  }
  ~container ()
  {
    dzn::pump *p = system.dzn_locator.template try_get<dzn::pump> (); // only shells have a pump
    //resolve the race condition between the shell pump dtor and the container pump dtor
    if (p && p != &pump) pump ([p] {p->stop ();});
    pump.wait ();
  }
  bool perform (bool on_pump = false)
  {
    auto trigger = [this]{this->perform (true);};
    std::string str;
    if (std::getline (std::cin, str))
      {
        if (std::count (str.begin (), str.end (), '.') > 1)
          return perform (on_pump);

      if (str == "<defer>")
        {
          pump.handle (0, 0, trigger);
          return false;
        }

      trail.push (str);

      auto it = lookup.find (str);
      if (it != lookup.end ())
        {
          if (on_pump) pump (trigger);
          it->second ();
          return true;
        }
    }
    return false;
  }
  void sync_trigger ()
  {
    if (perform ())
      sync_trigger ();
  }
  void operator () (std::map<std::string, Function> &&lookup)
  {
    this->lookup = std::move (lookup);
    pump ([this]{this->perform (true);});
    pump.wait ();
  }
  void match (std::string const& perform)
  {
    if (trail.empty ())
      throw std::runtime_error ("unmatched expectation: behavior performs: \""
                                + perform + "\" but trail empty");

    std::string expect = trail.front ();
    trail.pop ();
    if (expect != perform)
      throw std::runtime_error ("unmatched expectation: behavior performs: \""
                                + perform
                                + "\" but trail expects: \"" + expect + "\"");
  }
};
}

#endif //DZN_CONTAINER_HH
//version: 2.18.3
