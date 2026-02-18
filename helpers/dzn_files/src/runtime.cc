// dzn-runtime -- Dezyne runtime library
//
// Copyright © 2014, 2015, 2016, 2017, 2019, 2020, 2021, 2022 Jan (janneke) Nieuwenhuizen <janneke@gnu.org>
// Copyright © 2015, 2016, 2017, 2019, 2020, 2021, 2022, 2024 Rutger van Beusekom <rutger@dezyne.org>
// Copyright © 2015 Paul Hoogendijk <paul@dezyne.org>
//
// This file is part of dzn-runtime.
//
// All rights reserved.
//
//
// Commentary:
//
// Code:

#include <dzn/runtime.hh>
#include <dzn/coroutine.hh>

#include <algorithm>
#include <iostream>

namespace dzn
{
std::ostream debug (nullptr);
runtime::runtime () : defer () {}

void
trace_in (std::ostream &os, port::meta const &meta, const char *event_name)
{
  if (!os.rdbuf ())
    return;
  os << path (meta.require.meta, meta.require.name) << "." << event_name << " -> "
     << path (meta.provide.meta, meta.provide.name) << "." << event_name << std::endl;
}

void
trace_out (std::ostream &os, port::meta const &meta, const char *event_name)
{
  if (!os.rdbuf ())
    return;
  os << path (meta.require.meta, meta.require.name) << "." << event_name << " <- "
     << path (meta.provide.meta, meta.provide.name) << "." << event_name << std::endl;
}

void
trace_qin (std::ostream &os, port::meta const &meta, const char *event_name)
{
  if (!os.rdbuf ())
    return;
  if (path (meta.require.meta, "") == "<external>")
    trace_out (os, meta, event_name);
  else
    os <<  path (meta.require.meta, "<q>")
       << " <- "
       << path (meta.provide.meta, meta.provide.name) << "." << event_name << std::endl;
}

void
trace_qout (std::ostream &os, port::meta const &meta, const char *event_name)
{
  if (!os.rdbuf ())
    return;
  if (path (meta.require.meta, "") == "<external>")
    return;
  os << path (meta.require.meta, meta.require.name) << "." << event_name
     << " <- "
     << path (meta.require.meta, "<q>") << std::endl;
}

bool
runtime::external (dzn::component *component)
{
  return (states.find (component) == states.end ());
}

int &
runtime::activity (dzn::locator const& locator)
{
  return coroutine_id2activity[coroutine_id(locator)];
}

size_t &
runtime::handling (dzn::component *component)
{
  return states[component].handling;
}

size_t &
runtime::blocked (dzn::component *component)
{
  return states[component].blocked;
}

std::function<void()> &
runtime::deferred_flush (dzn::component *component)
{
  return states[component].port_update;
}

dzn::component *&
runtime::deferred (dzn::component *component)
{
  return states[component].deferred;
}

std::queue<std::function<void ()> > &
runtime::queue (dzn::component *component)
{
  return states[component].queue;
}

bool &
runtime::performs_flush (dzn::component *component)
{
  return states[component].performs_flush;
}

bool &
runtime::native (dzn::component *component)
{
  return states[component].native;
}

bool
runtime::skip_block (dzn::component *component, void *port)
{
  return states[component].skip == port;
}

void
runtime::set_skip_block (dzn::component *component, void *port)
{
  states[component].skip = port;
}

void
runtime::reset_skip_block (dzn::component *component)
{
  states[component].skip = nullptr;
}

void
runtime::flush (dzn::component *component, size_t coroutine_id, bool sync_p)
{
  std::queue<std::function<void ()> > &q = queue (component);
  auto &flush = this->deferred_flush (component);
  handling (component) = coroutine_id;
  bool flushed = false;
  while (!q.empty ())
    {
      std::function<void ()> event = q.front ();
      q.pop ();
      if (!flushed && !sync_p && flush)
      {
        flush ();
        flushed = true;
      }
      flush = nullptr;
      event ();
    }
  handling (component) = 0;
  dzn::component *target = deferred (component);
  if (!sync_p && target)
    {
      deferred (component) = nullptr;
      if (!handling (target))
        runtime::flush (target, coroutine_id, sync_p);
    }
  else if (!sync_p && this->deferred_flush (nullptr))
    {
      this->deferred_flush (nullptr) ();
      this->deferred_flush (nullptr) = nullptr;
    }
}
}
//version: 2.18.3
