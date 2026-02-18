#include "event_queue.hh"

TaskQueue::TaskQueue()
    : stopped_(false)
{
}

// Push a task to the back.
void TaskQueue::push(Task t)
{
  {
    std::unique_lock<std::mutex> lk(m_);
    q_.push_back(std::move(t));
  }
  cv_.notify_one();
}

// Try executing one task if available (non-blocking).
// Returns true if a task was executed.
bool TaskQueue::try_execute_one()
{
  Task t;
  {
    std::unique_lock<std::mutex> lk(m_);
    if (q_.empty()) return false;
    t = std::move(q_.front());
    q_.pop_front();
  }
  t(); // execute outside the lock
  return true;
}

// Blocking loop: waits for tasks and executes them until stop() is called.
void TaskQueue::run_until_stopped()
{
  while (!stopped_)
  {
    Task t;
    {
      std::unique_lock<std::mutex> lk(m_);
      cv_.wait(lk, [&]{ return stopped_ || !q_.empty(); });
      if (stopped_) break;
      t = std::move(q_.front());
      q_.pop_front();
    }
    t(); // execute outside the lock
  }
}

// Signal the loop to stop. Wakes any waiters.
void TaskQueue::stop()
{
  {
    std::unique_lock<std::mutex> lk(m_);
    stopped_ = true;
  }

  cv_.notify_all();
}

// Convenience: drain everything that’s currently queued (non-blocking).
void TaskQueue::drain()
{
  while (try_execute_one()) {}
}

bool TaskQueue::empty() const
{
  std::unique_lock<std::mutex> lk(m_);
  return q_.empty();
}
