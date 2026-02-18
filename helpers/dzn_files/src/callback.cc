#include "callback.hh"

const int N_RETRY_ATTEMPTS = 5;

Callback::Callback(std::shared_ptr<mqtt::async_client> client, mqtt::connect_options& connOpts)
    : mNRetries(0)
    , mClient(client)
    , mConnOpts(connOpts)
{
}

void Callback::message_arrived(mqtt::const_message_ptr msg)
{
  std::cout << "Message arrived on topic '" << msg->get_topic()
            << "': " << msg->to_string() << std::endl;

  const std::string topic = msg->get_topic();
  if (mHandlers.find(topic) != mHandlers.end())
  {
    std::cout << "Handler found for: " << topic << std::endl;
    auto handler = mHandlers.at(topic);
    auto data = msg->to_string();
    if (handler)
      handler(data);
  }
}

void Callback::register_handler(const std::string& topic, std::function<int(const std::string& message)> handler)
{
  std::cout << "Registering handler: " << topic << std::endl;

  mHandlers.insert({topic, handler});
}

void Callback::connected(const std::string& cause)
{
  std::cout << "\nConnection success" << std::endl;
}

void Callback::bind_handlers()
{
  for (const auto& key : mHandlers)
  {
    std::cout << "Subscribing to: " << key.first << std::endl;
    mClient->subscribe(key.first, 1);
  }
}

void Callback::on_success(const mqtt::token& tok)
{
}

void Callback::on_failure(const mqtt::token& tok)
{
  std::cout << "Connection attempt failed" << std::endl;
  if (++mNRetries > N_RETRY_ATTEMPTS)
    exit(1);

  reconnect();
}

void Callback::reconnect()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  try
  {
    mClient->connect(mConnOpts, nullptr, *this);
  } catch (const mqtt::exception& exc)
  {
    std::cerr << "Error: " << exc.what() << std::endl;
    exit(1);
  }
}

void Callback::connection_lost(const std::string& cause)
{
  std::cout << "\nConnection lost" << std::endl;
  if (!cause.empty())
    std::cout << "\tcause: " << cause << std::endl;

  std::cout << "Reconnecting..." << std::endl;
  mNRetries = 0;
  reconnect();
}