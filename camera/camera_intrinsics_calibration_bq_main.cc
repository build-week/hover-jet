#include "camera_intrinsics_calibration_bq.hh"
#include <signal.h>
#include <unistd.h>

//%deps(paho-mqttpp3)

static bool shutdown = false;

void signal_handler(int s){
  printf("Caught signal %d\n",s);
  shutdown = true;
}

int main() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  jet::CameraIntrincicsCalibrationBq subscriber_demo_bq = jet::CameraIntrincicsCalibrationBq();
  subscriber_demo_bq.init();
  while (!shutdown)
  {
    subscriber_demo_bq.loop();
    usleep(subscriber_demo_bq.loop_delay_microseconds);
  }
  subscriber_demo_bq.shutdown();
  return 0;
}

