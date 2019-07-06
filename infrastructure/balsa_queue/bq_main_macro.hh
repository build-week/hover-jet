#pragma once

#include <signal.h>
#include <unistd.h>
#include "infrastructure/comms/mqtt_comms_factory.hh"
#include "infrastructure/config/config.hh"
//%deps(paho-mqttpp3)

static bool shutdown = false;

void signal_handler(int s) {
  printf("Caught signal %d\n", s);
  shutdown = true;
}

void set_up_signal_handler() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

std::optional<std::string> parse_commandline_arguments(int argc, char* argv[]) {
  if (argc > 1) {
    return argv[1];
  }
  return {};
}

void parse_config(const std::string& config_path, Config& config) {
  try {
    config = YAML::LoadFile(config_path);
  } catch (YAML::BadFile e) {
    throw std::runtime_error("Could not find BQ Config specified: " + config_path);
  }
}

template <typename BQ_TYPE>
void run_bq(const std::string& bq_typename, const std::optional<std::string>& config_path) {
  Config bq_config;
  if (config_path.has_value()) {
    parse_config(config_path.value(), bq_config);
  }

  BQ_TYPE balsa_queue = BQ_TYPE();
  if (bq_config["bq_name"]) {
    balsa_queue.set_name(bq_config["bq_name"].as<std::string>());
    balsa_queue.gonogo().setName(bq_config["bq_name"].as<std::string>());
  } else {
    balsa_queue.set_name(bq_typename);
    balsa_queue.gonogo().setName(bq_typename);
  }
  balsa_queue.gonogo().nogo("init");
  balsa_queue.set_comms_factory(std::make_unique<jet::MqttCommsFactory>());
  balsa_queue.base_init();
  balsa_queue.init(bq_config);
  while (!shutdown) {
    balsa_queue.base_loop();
    balsa_queue.loop();
    usleep(balsa_queue.loop_delay_microseconds);
  }
  balsa_queue.shutdown();
}

#define BALSA_QUEUE_MAIN_FUNCTION(bq_type)                              \
  int main(int argc, char* argv[]) {                                    \
    set_up_signal_handler();                                            \
    run_bq<bq_type>(#bq_type, parse_commandline_arguments(argc, argv)); \
    return 0;                                                           \
  }
