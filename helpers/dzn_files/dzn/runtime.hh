// dzn-runtime -- Dezyne runtime library
//
// Copyright © 2016, 2017, 2019 - 2023, 2024 Janneke Nieuwenhuizen <janneke@gnu.org>
// Copyright © 2016 Rob Wieringa <rma.wieringa@gmail.com>
// Copyright © 2016 Henk Katerberg <hank@mudball.nl>
// Copyright © 2016 - 2024 Rutger van Beusekom <rutger@dezyne.org>
//
// This file is part of dzn-runtime.
//
// All rights reserved.
//
//
// Commentary:
//
// Code:

#ifndef DZN_RUNTIME_HH
#define DZN_RUNTIME_HH

#include <dzn/config.hh>
#include <dzn/meta.hh>
#include <dzn/locator.hh>
#include <dzn/coroutine.hh>

#include <algorithm>
#include <cstddef>
#include <future>
#include <iostream>
#include <map>
#include <queue>
#include <tuple>
#include <vector>

// Set to 1 for experimental state tracing feature.
#ifndef DZN_STATE_TRACING
#define DZN_STATE_TRACING 0
#endif

namespace dzn
{
template <typename T>
std::string to_string (T t);
inline std::string
to_string (bool b)
{
  return b ? "true" : "false";
}

inline std::string
to_string (int i)
{
  return std::to_string (i);
}

inline void
to_void (std::string const &)
{
}

inline int
to_int (std::string s)
{
  return std::stoi (s);
}

inline bool
to_bool (std::string s)
{
  return s == "true";
}

extern std::ostream debug;

inline std::string
component_to_string (dzn::component *c)
{
  return c ? reinterpret_cast<component_meta *> (c)->dzn_meta.name
    : "<external>";
}

void trace_qin (std::ostream &, port::meta const &, char const *);
void trace_qout (std::ostream &, port::meta const &, char const *);

void trace_in (std::ostream &, port::meta const &, char const *);
void trace_out (std::ostream &, port::meta const &, char const *);

inline void
apply (dzn::meta const *m, const std::function<void (dzn::meta const *)> &f)
{
  f (m);
  for (auto c : m->children)
    {
      apply (c, f);
    }
}

inline void
check_bindings (dzn::meta const *c)
{
  apply (c, [] (dzn::meta const * m)
  {
    std::for_each (m->ports_connected.begin (), m->ports_connected.end (), [] (std::function<void ()> const &p) {p ();});
  });
}

inline void
check_bindings (dzn::component &c)
{
  check_bindings (&reinterpret_cast<component_meta const *> (&c)->dzn_meta);
}

template <typename Foreign>
inline void
check_bindings (Foreign const& f)
{
  check_bindings (&f.dzn_meta);
}

inline void
dump_tree (std::ostream &os, dzn::meta const *c)
{
  apply (c, [&] (dzn::meta const * m)
  {
    os << path (m) << ":" << m->type << std::endl;
  });
}

inline void
dump_tree (dzn::component const &c)
{
  dump_tree (std::clog, &reinterpret_cast<component_meta const *> (&c)->dzn_meta);
}

// implemented conditionally in pump.cc
void collateral_block (locator const &, dzn::component *);
bool port_blocked_p (locator const &, void *);
void port_block (locator const &, dzn::component *, void *);
void port_release (locator const &, dzn::component *, void *);
size_t coroutine_id (locator const &);
void defer (locator const &, std::function<bool ()> &&, std::function<void (size_t)> &&);
void prune_deferred (locator const &);

struct runtime
{
  runtime (runtime const &) = delete;
  runtime (runtime &&) = delete;
  struct state
  {
    int activity;
    size_t handling;
    size_t blocked;
    void *skip;
    bool native;
    bool performs_flush;
    std::function<void()> port_update;
    dzn::component *deferred;
    std::queue<std::function<void ()>> queue;
  };
  bool defer;
  std::map<dzn::component *, state> states;
  std::map<size_t, int> coroutine_id2activity;
  bool skip_block (dzn::component *, void *);
  void set_skip_block (dzn::component *, void *);
  void reset_skip_block (dzn::component *);

  bool external (dzn::component *);
  int &activity (dzn::locator const&);
  size_t &handling (dzn::component *);
  size_t &blocked (dzn::component *);
  std::function<void()> &deferred_flush (dzn::component *);
  dzn::component *&deferred (dzn::component *);
  std::queue<std::function<void ()> > &queue (dzn::component *);
  bool &performs_flush (dzn::component *);
  bool &native (dzn::component *);
  void flush (dzn::component *, size_t, bool sync_p);
  template <typename T>
  void flush (T *t)
  {
    flush (t, coroutine_id (t->dzn_locator), false);
  }
  runtime ();
};

struct scoped_activity
{
  int &value;
  int initial_activity;
  scoped_activity (dzn::runtime& runtime, dzn::locator const& locator, int v)
    : value (runtime.activity (locator))
    , initial_activity (value)
  {
    if (initial_activity == 0)
      value = v;
  }
  ~scoped_activity ()
  {
    if (initial_activity == 0)
      value = 0;
  }
};

struct scoped_handling
{
  size_t &value;
  scoped_handling (dzn::runtime& runtime, dzn::locator const& locator,
                   dzn::component *component)
    : value (runtime.handling (component))
  {
    value = coroutine_id (locator);
  }
  ~scoped_handling ()
  {
    value = 0;
  }
};

template <typename C, typename P, typename E>
void defer (C *component, P &&predicate, E const &statement)
{
  defer (component->dzn_locator, std::function<bool ()> (predicate),
         std::function<void (size_t)> ([ = ] (size_t coroutine_id)
         {
           scoped_activity activity (component->dzn_runtime,
                                     component->dzn_locator, -1);
           component->dzn_runtime.defer = true;
           statement ();
           component->dzn_runtime.flush (component, coroutine_id, false);
           component->dzn_runtime.defer = false;
         }));
}

//https://cp-algorithms.com/string/string-hashing.html
inline std::uint32_t
hash (char const* label, std::uint32_t state)
{
  // numeric base for beginning of [0-9a-zA-Z] - 1, i.e. '0' = 48 - 1
  constexpr std::uint32_t base = 47;
  // smallest prime encompassing [0-9a-zA-Z] numerically
  constexpr std::uint32_t prime = 79;
  std::string n = std::to_string (state);
  char const* s = n.c_str ();
  std::uint32_t power = 1;
  std::uint32_t h = 0;
  while (*s)
    {
      power *= prime;
      h = h + (*s++ - base) * power;
    }
  char const* p = label;
  while (*p)
    {
      power *= prime;
      h = h + (*p++ - base) * power;
    }
#if 0
    std::cout << '"' << label << '"'
              << " -> state " << state
              << " hash " << h << std::endl;
#endif
  return h;
}

namespace in
{
template <typename Signature>
struct event;

template <typename R, typename...Args>
struct event<R (Args...)>
{
  std::function <void ()> dzn_out_binding;
  bool dzn_strict_p;
  R reply;
  void* port;
  std::function <void (char const*)> port_update;
  dzn::port::meta* dzn_port_meta;
  dzn::component* component;
  std::function <void ()> write_state;
  dzn::meta* dzn_meta;
  dzn::locator const* dzn_locator;
  dzn::runtime* dzn_runtime;
  std::ostream* os;
  char const* name;

  std::function<R (Args...)> e;
  event ()
    : dzn_strict_p (false)
    , reply ()
    , port ()
    , port_update ([] (char const*){})
    , dzn_port_meta ()
    , component ()
    , write_state ([]{})
    , dzn_meta ()
    , dzn_locator ()
    , dzn_runtime ()
    , os ()
    , name ()
  {}
  event (event const&) = default;
  event (event&&) = default;
  template <typename Component, typename Interface>
  void set (Component* c, Interface* i, char const* n)
  {
    this->dzn_strict_p = true;
    this->port = i;
    this->port_update = [i,c] (char const* s)
    {
      i->dzn_label = s;
      i->dzn_update_state (c->dzn_locator);
    };
    this->dzn_port_meta = &i->dzn_meta;
    this->component = c;
    this->write_state = []{};
    this->dzn_meta = &c->dzn_meta;
    this->dzn_locator = &c->dzn_locator;
    this->dzn_runtime = &c->dzn_runtime;
    this->os = &dzn_locator->get<std::ostream> ();
    this->name = n;
  }
  operator bool () const
  {
    return static_cast<bool> (this->e);
  }
  event& operator = (event const& that)
  {
    assert (that.port);
    assert (that.port_update);
    assert (that.dzn_port_meta);
    assert (that.component);
    assert (that.write_state);
    assert (that.dzn_meta);
    assert (that.dzn_locator);
    assert (that.dzn_runtime);

    std::function<R (Args...)> g (that.e);
    this->e = [g,this] (Args...args)
    {
      if (!this->dzn_strict_p)
        return g (args...);
      struct RAII
      {
        event* that;
        std::string reply_string;
        RAII (event* other)
          : that (other)
        {
          that->port_update (that->name);
        }
        R&& operator () (R&& r)
        {
          that->reply = r;
          return std::forward<R> (r);
        }
        ~RAII ()
        {
          reply_string = ::dzn::to_string (that->reply);
          that->port_update (reply_string.c_str ());
        }
      } raii (this);
      return raii (g (args...));
    };
    return *this;
  }
  template <typename Lambda>
  event& operator = (Lambda const& l)
  {
    return operator = (std::function<R (Args...)> (l));
  }
  event& operator = (std::function<R (Args...)>const & f)
  {
    this->e = [f,this] (Args ... args) -> decltype (f (args...))
    {
      if (!this->dzn_strict_p)
        return f (args...);

      assert (this->port);
      assert (this->port_update);
      assert (this->dzn_port_meta);
      assert (this->component);
      assert (this->write_state);
      assert (this->dzn_meta);
      assert (this->dzn_locator);
      assert (this->dzn_runtime);

      dzn::locator const& locator = *this->dzn_locator;
      dzn::runtime &runtime = *this->dzn_runtime;
      dzn::component &component = *this->component;
      dzn::component *provide = this->dzn_port_meta->provide.component;

      if ((runtime.handling (provide) || port_blocked_p (locator, this->port))
          && runtime.native (provide))
        collateral_block (locator, provide);
      runtime.reset_skip_block (&component);
      trace_in (*this->os, *this->dzn_port_meta, this->name);
      this->port_update (this->name);
      this->write_state ();
      scoped_handling handling (runtime, locator, &component);
      scoped_activity activity (runtime, locator, 1);
      this->reply = f (args...);
      runtime.flush (provide, coroutine_id (locator), true);
      std::string reply_string = ::dzn::to_string (this->reply);
      trace_out (*this->os, *this->dzn_port_meta, reply_string.c_str ());
      this->port_update (reply_string.c_str ());
      this->write_state ();
      prune_deferred (locator);
      if (this->dzn_out_binding)
        this->dzn_out_binding ();

      return this->reply;
    };
    return *this;
  }
  R operator () (Args...args)
  {
    return this->e (args...);
  }
};

template <typename...Args>
struct event<void (Args...)>
{
  std::function <void ()> dzn_out_binding;
  bool dzn_strict_p;
  void* port;
  std::function <void (char const*)> port_update;
  dzn::port::meta* dzn_port_meta;
  dzn::component* component;
  std::function <void ()> write_state;
  dzn::meta* dzn_meta;
  dzn::locator const* dzn_locator;
  dzn::runtime* dzn_runtime;
  std::ostream* os;
  char const* name;

  std::function<void (Args...)> e;
  event ()
    : dzn_strict_p (false)
    , port ()
    , port_update ([] (char const*){})
    , dzn_port_meta ()
    , component ()
    , write_state ([]{})
    , dzn_meta ()
    , dzn_locator ()
    , dzn_runtime ()
    , os ()
    , name ()
  {}
  event (event const&) = default;
  event (event&&) = default;
  template <typename Component, typename Interface>
  void set (Component* c, Interface* i, char const* n)
  {
    this->dzn_strict_p = true;
    this->port = i;
    this->port_update = [i,c] (char const* s)
    {
      i->dzn_label = s;
      i->dzn_update_state (c->dzn_locator);
    };
    this->dzn_port_meta = &i->dzn_meta;
    this->component = c;
    this->write_state = []{};
    this->dzn_meta = &c->dzn_meta;
    this->dzn_locator = &c->dzn_locator;
    this->dzn_runtime = &c->dzn_runtime;
    this->os = &dzn_locator->get<std::ostream> ();
    this->name = n;
  }
  operator bool () const
  {
    return static_cast<bool> (this->e);
  }
  event& operator = (event const& that)
  {
    assert (that.port);
    assert (that.port_update);
    assert (that.dzn_port_meta);
    assert (that.component);
    assert (that.write_state);
    assert (that.dzn_meta);
    assert (that.dzn_locator);
    assert (that.dzn_runtime);

    std::function<void (Args...)> g (that.e);
    this->e = [g,this] (Args...args)
    {
      if (!this->dzn_strict_p)
        return g (args...);

      struct RAII
      {
        event* that;
        char const* reply_string;
        RAII (event* other)
          : that (other)
          , reply_string ("return")
        {
          that->port_update (that->name);
        }
        ~RAII ()
        {
          that->port_update (reply_string);
        }
      } raii (this);
      return g (args...);
    };
    return *this;
  }
  template <typename Lambda>
  event& operator = (Lambda const& l)
  {
    return operator = (std::function<void (Args...)> (l));
  }
  event& operator = (std::function<void (Args...)> const& f)
  {
    this->e = [f,this] (Args... args) -> decltype (f (args...))
    {
      if (!dzn_strict_p)
        return f (args...);

      assert (this->port);
      assert (this->port_update);
      assert (this->dzn_port_meta);
      assert (this->component);
      assert (this->write_state);
      assert (this->dzn_meta);
      assert (this->dzn_locator);
      assert (this->dzn_runtime);

      dzn::locator const& locator = *this->dzn_locator;
      dzn::runtime &runtime = *this->dzn_runtime;
      dzn::component &component = *this->component;
      dzn::component *provide = this->dzn_port_meta->provide.component;

      if ((runtime.handling (provide)
           || port_blocked_p (locator, this->port))
          && runtime.native (provide))
        collateral_block (locator, provide);
      runtime.reset_skip_block (&component);
      trace_in (*this->os, *this->dzn_port_meta, this->name);
      this->port_update (this->name);
      this->write_state ();
      scoped_handling handling (runtime, locator, &component);
      scoped_activity activity (runtime, locator, 1);
      f (args...);
      runtime.flush (provide, coroutine_id (locator), true);
      trace_out (*this->os, *this->dzn_port_meta, "return");
      this->port_update ("return");
      this->write_state ();
      prune_deferred (locator);
      if (this->dzn_out_binding)
        this->dzn_out_binding ();
    };
    return *this;
  }
  void operator () (Args...args)
  {
    this->e (args...);
  }
};

}
namespace out
{
template <typename Signature>
struct event;

template <typename...Args>
struct event<void (Args...)>
{
  bool dzn_strict_p;
  void* port;
  std::function <void (char const*)> port_update;
  std::function <void (char const*)> other_port_update;
  dzn::port::meta* dzn_port_meta;
  dzn::component* component;
  std::function <void ()> write_state;
  dzn::meta* dzn_meta;
  dzn::locator const* dzn_locator;
  dzn::runtime* dzn_runtime;
  std::ostream* os;
  char const* name;
  std::function<void (Args...)> e;

  event ()
    : dzn_strict_p (false)
    , port ()
    , port_update ([] (char const*){})
    , dzn_port_meta ()
    , component ()
    , write_state ([]{})
    , dzn_meta ()
    , dzn_locator ()
    , dzn_runtime ()
    , os ()
    , name ()
  {}
  event (event const&) = default;
  event (event&&) = default;
  template <typename Component, typename Interface>
  void set (Component* c, Interface* i, char const* n)
  {
    this->dzn_strict_p = true;
    this->port = i;
    this->port_update = [this,i,c] (char const* s)
    {
      i->dzn_label = s;
      i->dzn_update_state (c->dzn_locator);
    };
    this->dzn_port_meta = &i->dzn_meta;
    this->component = c;
    this->write_state = []{};
    this->dzn_meta = &c->dzn_meta;
    this->dzn_locator = &c->dzn_locator;
    this->dzn_runtime = &c->dzn_runtime;
    this->os = &dzn_locator->get<std::ostream> ();
    this->name = n;
  }
  operator bool () const
  {
    return static_cast<bool> (this->e);
  }
  event& operator = (event const& that)
  {
    assert (that.port);
    assert (that.port_update);
    assert (that.dzn_port_meta);
    assert (that.component);
    assert (that.write_state);
    assert (that.dzn_meta);
    assert (that.dzn_locator);
    assert (that.dzn_runtime);

    std::function<void (Args...)> g (that.e);
    this->e = [g,this] (Args...args)
    {
      if (!this->dzn_strict_p)
        return g (args...);

      this->port_update (this->name);
      g (args...);
    };
    return *this;
  }
  template <typename Lambda>
  event& operator = (Lambda const& l)
  {
    return operator = (std::function<void (Args...)> (l));
  }
  event& operator = (std::function<void (Args...)> const& f)
  {
    this->e = [f,this] (Args...args)
    {
      if (!this->dzn_strict_p)
        return f (args...);

      assert (this->port);
      assert (this->port_update);
      assert (this->dzn_port_meta);
      assert (this->component);
      assert (this->write_state);
      assert (this->dzn_meta);
      assert (this->dzn_locator);
      assert (this->dzn_runtime);

      trace_qin (*this->os, *this->dzn_port_meta, this->name);
      this->write_state ();
      this->port_update (this->name);

      dzn::locator const &locator = *this->dzn_locator;
      dzn::runtime &runtime = *this->dzn_runtime;
      dzn::component *component = this->component;
      dzn::component *provide = this->dzn_port_meta->provide.component;
      dzn::component *require = this->dzn_port_meta->require.component;
      bool no_flush_label_p = port_blocked_p (locator, this->port)
        || runtime.performs_flush (nullptr)
        || runtime.defer;

      auto event = [f,require,this,args...]
      {
        if (require)
          {
            trace_qout (*this->os, *this->dzn_port_meta, this->name);
            this->write_state ();
          }
        f (args...);
      };

      auto port_update = [this]
      {
        this->port_update ("<flush>");
        if (this->other_port_update)
          this->other_port_update ("<flush>");
      };

      scoped_activity activity (runtime, locator, -1);

      runtime.deferred_flush (require) = [this,port_update,provide,&locator,&runtime]
        {
          if (!port_blocked_p (locator, this->port)
              && (!runtime.defer || runtime.native (provide)))
            port_update ();
        };
      runtime.deferred (provide) = require;

      if (!require && no_flush_label_p)
        f (args...);
      else
        {
          runtime.queue (require).push (event);
          if (!require
              || (!provide
                  && !runtime.handling (component)
                  && !runtime.performs_flush (component))
              || (provide
                  && !runtime.native (provide)
                  && !runtime.handling (provide)
                  && !runtime.performs_flush (provide)))
            runtime.flush (require, coroutine_id (locator),
                           activity.value == 1);
          if (!provide
              && !no_flush_label_p
              && runtime.blocked (provide)
              && runtime.handling (component))
            port_update ();
        }

      prune_deferred (locator);
    };
    return *this;
  }
  void operator () (Args...args) const
  {
    return this->e (args...);
  }
};
}
}
#endif //DZN_RUNTIME_HH
//version: 2.18.3
