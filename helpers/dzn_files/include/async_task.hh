#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <thread>

class AsyncTask
{
public:
  AsyncTask(std::function<void()> task);
  ~AsyncTask();

  bool IsStarted() const;
  bool IsExecuting() const;

  void Trigger();

  bool IsCurrentThread() const;

private:
  // C++ is sometimes hard to use, need all of this for a reliable thread wake-up
  enum class State
  {
    Waiting,
    UpdateRequested,
    Executing,
    StopRequested
  } mState;

  std::function<void()> mExecute;
  mutable std::mutex mUpdateMutex;
  std::condition_variable mUpdate;
  std::thread mAsyncTask;

  void AsyncLoop() noexcept;
};
