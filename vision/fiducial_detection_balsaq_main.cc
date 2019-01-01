#include "vision/fiducial_detection_balsaq.hh"

#include <signal.h>

//%deps(paho-mqttpp3)

#include <unistd.h>

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

	jet::FidicualDetectionBq fiducial_detection_bq = jet::FidicualDetectionBq();
	fiducial_detection_bq.init();
	while (!shutdown)
	{
		fiducial_detection_bq.loop();
		usleep(jet::FidicualDetectionBq::loop_delay_microseconds);
	}
	fiducial_detection_bq.shutdown();
	return 0;
}
