#pragma once

#include <signal.h>
#include <unistd.h>

//%deps(paho-mqttpp3)

static bool shutdown = false;

void signal_handler(int s) {
  printf("Caught signal %d\n", s);
  shutdown = true;
}

#define BALSA_QUEUE_MAIN_FUNCTION(bq_type)         \
  int main(int argc, char *argv[]) {               \
    struct sigaction sigIntHandler;                \
    sigIntHandler.sa_handler = signal_handler;     \
    sigemptyset(&sigIntHandler.sa_mask);           \
    sigIntHandler.sa_flags = 0;                    \
    sigaction(SIGINT, &sigIntHandler, NULL);       \
    bq_type balsa_queue = bq_type();               \
    balsa_queue.init(argc, argv);                            \
    while (!shutdown) {                            \
      balsa_queue.loop();                          \
      usleep(balsa_queue.loop_delay_microseconds); \
    }                                              \
    balsa_queue.shutdown();                        \
    return 0;                                      \
  }
