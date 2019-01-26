#pragma once

#include "infrastructure/comms/mqtt_comms_factory.hh"

#include <chrono>
#include <signal.h>
#include <unistd.h>
//%deps(paho-mqttpp3)

static bool shutdown = false;

void signal_handler(int s) {
  printf("Caught signal %d\n", s);
  shutdown = true;
}

#define BALSA_QUEUE_MAIN_FUNCTION(bq_type)                                         \
  int main(int argc, char *argv[]) {                                               \
    struct sigaction sigIntHandler;                                                \
    sigIntHandler.sa_handler = signal_handler;                                     \
    sigemptyset(&sigIntHandler.sa_mask);                                           \
    sigIntHandler.sa_flags = 0;                                                    \
    sigaction(SIGINT, &sigIntHandler, NULL);                                       \
    bq_type balsa_queue = bq_type();                                               \
    balsa_queue.set_comms_factory(std::make_unique<jet::MqttCommsFactory>());      \
    balsa_queue.init(argc, argv);                                                  \
    std::chrono::high_resolution_clock::time_point start_time;                     \
    while (!shutdown) {                                                            \
      start_time = std::chrono::high_resolution_clock::now();                      \
      balsa_queue.loop();                                                          \
      uint loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(  \
        std::chrono::high_resolution_clock::now() - start_time).count();           \
      if (balsa_queue.loop_delay_microseconds > loop_duration) {                   \
        usleep(balsa_queue.loop_delay_microseconds - loop_duration);               \
      }                                                                            \
    }                                                                              \
    balsa_queue.shutdown();                                                        \
    return 0;                                                                      \
  }
