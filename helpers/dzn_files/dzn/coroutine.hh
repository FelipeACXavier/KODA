// dzn-runtime -- Dezyne runtime library
//
// Copyright © 2016, 2017, 2023 Jan Nieuwenhuizen <janneke@gnu.org>
// Copyright © 2016 Henk Katerberg <hank@mudball.nl>
// Copyright © 2015 - 2018, 2022, 2034 Rutger van Beusekom <rutger@dezyne.org>
//
// This file is part of dzn-runtime.
//
// All rights reserved.
//
//
// Commentary:
//
// Code:

#ifndef DZN_COROUTINE_HH
#define DZN_COROUTINE_HH

#include <dzn/config.hh>

#if HAVE_BOOST_COROUTINE
#include <boost/coroutine/all.hpp>
namespace dzn
{
typedef boost::coroutines::symmetric_coroutine<void>::call_type context;
typedef boost::coroutines::symmetric_coroutine<void>::yield_type yield;
typedef boost::coroutines::detail::forced_unwind forced_unwind;
}
#else
#include <dzn/context.hh>
namespace dzn
{
typedef context::forced_unwind forced_unwind;
typedef std::function<void (context &)> yield;
}
#endif

namespace dzn
{
struct coroutine
{
  size_t id;
  dzn::context context;
  dzn::yield yield;
  void *component;
  void *port;
  bool finished;
  bool skip_block;
  coroutine ();
  coroutine (size_t id, std::function<void()> &&worker);
  void yield_to (dzn::coroutine &that);
  void call (coroutine &that);
  void release ();
};
}
#endif //DZN_COROUTINE_HH
//version: 2.18.3
