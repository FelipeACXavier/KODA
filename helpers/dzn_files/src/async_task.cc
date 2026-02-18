#include "async_task.hh"

AsyncTask::AsyncTask(std::function<void()> task)
    : mState(State::Waiting)
    , mExecute(task)
    , mAsyncTask(&AsyncTask::AsyncLoop, this)
{
}

AsyncTask::~AsyncTask()
{
  {
    std::unique_lock<std::mutex> lck(mUpdateMutex);
    mState = State::StopRequested;
  }

  mUpdate.notify_all();

  if (mAsyncTask.joinable())
    mAsyncTask.join();
}

bool AsyncTask::IsStarted() const
{
  std::unique_lock<std::mutex> lck(mUpdateMutex);
  return mState != State::StopRequested;
}

bool AsyncTask::IsExecuting() const
{
  std::unique_lock<std::mutex> lck(mUpdateMutex);
  return mState == State::Executing;
}

bool AsyncTask::IsCurrentThread() const
{
  return std::this_thread::get_id() == mAsyncTask.get_id();
}

void AsyncTask::Trigger()
{
  {
    std::unique_lock<std::mutex> lck(mUpdateMutex);
    if (mState != State::StopRequested)
      mState = State::UpdateRequested;
  }

  mUpdate.notify_all();
}

void AsyncTask::AsyncLoop() noexcept
{
  while (true)
  {
    {
      std::unique_lock<std::mutex> lck(mUpdateMutex);
      mUpdate.wait(lck, [this] { return mState != State::Waiting; });

      if (mState == State::StopRequested)
        return;

      mState = State::Executing;
    }

    if (mExecute != nullptr)
      mExecute();

    {
      std::unique_lock<std::mutex> lck(mUpdateMutex);
      if (mState == State::Executing)
        mState = State::Waiting;
    }
  }
}
