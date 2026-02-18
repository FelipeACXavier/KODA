#include "calarm.hh"

#include <chrono>
#include <functional>

calarm::calarm(dzn::locator const& locator)
    : skel::calarm(locator)
    , mAlarmState(AlarmState::Inactive)
    , mAlarmTask(std::bind(&calarm::ExecuteTask, this))
{
}

calarm::~calarm()
{
  Cancel();
}

void calarm::api_set(int millis)
{
  WakeUpIn(std::chrono::milliseconds(millis * 1000), [this]() { api_timeout(); });
}

void calarm::api_reset()
{
  Cancel();
}

bool calarm::WakeUpIn(std::chrono::microseconds time, std::function<void()> callback)
{
  if (time < std::chrono::microseconds(0))
  {
    return false;
  }

  std::unique_lock<std::mutex> lck(mAlarmStateMutex);

  CancelUnlocked(lck);

  mWakeupCallback = callback;
  mDuration = time;
  mStart = std::chrono::high_resolution_clock::now();
  mEnd = mStart + mDuration;
  mAlarmState = AlarmState::Active;

  mAlarmTask.Trigger();

  return true;
}

void calarm::Cancel()
{
  std::unique_lock<std::mutex> lck(mAlarmStateMutex);

  CancelUnlocked(lck);
}

void calarm::CancelUnlocked(std::unique_lock<std::mutex>& lck)
{
  if (mAlarmState != AlarmState::Inactive)
  {
    mAlarmState = AlarmState::Canceled;
    mAlarmCond.notify_all();
  }

  WaitUntilDone(lck);
}

bool calarm::IsActive() const
{
  std::unique_lock<std::mutex> lck(mAlarmStateMutex);
  return mAlarmState == AlarmState::Active;
}

bool calarm::IsCancelled() const
{
  std::unique_lock<std::mutex> lck(mAlarmStateMutex);
  return mAlarmState == AlarmState::Canceled;
}

void calarm::WaitUntilDone(std::unique_lock<std::mutex>& lck)
{
  if (mAlarmTask.IsCurrentThread())
    return;

  mAlarmDone.wait(lck, [this] { return mAlarmState == AlarmState::Inactive; });
}

std::chrono::time_point<std::chrono::high_resolution_clock> calarm::StartTime() const
{
  return mStart;
}

std::chrono::microseconds calarm::GetDuration() const
{
  return mDuration;
}

void calarm::ExecuteTask()
{
  bool timeoutOccurred = true;
  {
    std::unique_lock<std::mutex> lck(mAlarmStateMutex);
    timeoutOccurred = !mAlarmCond.wait_until(lck, mEnd, [this] { return !mAlarmTask.IsExecuting() || mAlarmState != AlarmState::Active; });
    mAlarmState = AlarmState::Inactive;
  }

  if (timeoutOccurred)
  {
    if (mWakeupCallback != nullptr)
      mWakeupCallback();
  }

  // {
  //   std::unique_lock<std::mutex> lck(mAlarmStateMutex);
  //   std::cout << "Set inactive" << std::endl;
  //   mAlarmState = AlarmState::Inactive;
  // }

  mAlarmDone.notify_all();
}
