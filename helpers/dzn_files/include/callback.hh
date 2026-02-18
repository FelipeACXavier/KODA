#pragma once

#include <mqtt/async_client.h>

class Callback : public virtual mqtt::callback, public virtual mqtt::iaction_listener
{
public:
  Callback(std::shared_ptr<mqtt::async_client> client, mqtt::connect_options& connOpts);

  void connected(const std::string& cause) override;

  void on_success(const mqtt::token& tok) override;
  void on_failure(const mqtt::token& tok) override;
  void connection_lost(const std::string& cause) override;

  void message_arrived(mqtt::const_message_ptr msg) override;

  void bind_handlers();
  void register_handler(const std::string& topic, std::function<int(const std::string& message)> handler);

private:
  int mNRetries;
  std::shared_ptr<mqtt::async_client> mClient;
  mqtt::connect_options& mConnOpts;

  std::map<std::string, std::function<int(const std::string& message)>> mHandlers;

  void reconnect();
};