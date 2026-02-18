#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>

#include "a_calarm.hh"
#include "async_task.hh"

class calarm : public skel::calarm
{
public:
  calarm(dzn::locator const& locator);
  ~calarm();

  void api_set(int millis) override;
  void api_reset() override;

  bool WakeUpIn(std::chrono::microseconds time, std::function<void()> callback);
  void Cancel();
  bool IsCancelled() const;
  std::chrono::time_point<std::chrono::high_resolution_clock> StartTime() const;

  // Note that at any point the active status may change
  bool IsActive() const;

  std::chrono::microseconds GetDuration() const;

private:
  enum class AlarmState
  {
    Inactive,
    Active,
    Canceled
  } mAlarmState;

  mutable std::mutex mAlarmStateMutex;

  AsyncTask mAlarmTask;

  std::chrono::time_point<std::chrono::high_resolution_clock> mStart, mEnd;
  std::chrono::microseconds mDuration;
  std::function<void()> mWakeupCallback;

  std::condition_variable mAlarmCond;
  std::condition_variable mAlarmDone;

  void CancelUnlocked(std::unique_lock<std::mutex>& lck);
  void ExecuteTask();

  void WaitUntilDone(std::unique_lock<std::mutex>& lck);
};