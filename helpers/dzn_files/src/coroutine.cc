// dzn-runtime -- Dezyne runtime library
//
// Copyright © 2024 Rutger van Beusekom <rutger@dezyne.org>
//
// This file is part of dzn-runtime.
//
// All rights reserved.
//
//
// Commentary:
//
// Code:

#include <dzn/coroutine.hh>

namespace dzn
{
coroutine::coroutine (size_t id, std::function<void()>&& worker)
  : id (id)
  , context ([this, worker] (dzn::yield &yield)
  {
    this->yield = std::move (yield);
    worker ();
  })
  , port ()
  , finished ()
  , skip_block ()
{}
coroutine::coroutine ()
  : id (0)
  , context ()
  , port ()
  , finished ()
  , skip_block ()
{}
void coroutine::yield_to (dzn::coroutine &that)
{
  this->yield (that.context);
}
#if HAVE_BOOST_COROUTINE
void coroutine::call (dzn::coroutine &)
{
  this->context ();
}
void coroutine::release () {}
#else //!HAVE_BOOST_COROUTINE
void coroutine::call (dzn::coroutine &that)
{
  this->context.call (that.context);
}
void coroutine::release ()
{
  this->context.release ();
}
#endif // !HAVE_BOOST_COROUTINE
}
//version: 2.18.3
