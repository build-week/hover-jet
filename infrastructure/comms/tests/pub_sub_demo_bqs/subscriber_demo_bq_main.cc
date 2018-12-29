#include "subscriber_demo_bq.hh"

#include <signal.h>
#include <unistd.h>

#include <iostream>

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

	jet::SubscriberDemoBq subscriber_demo_bq = jet::SubscriberDemoBq();
	subscriber_demo_bq.init();
	while (!shutdown)
	{
		subscriber_demo_bq.loop();
		usleep(1000000);
		std::cout << "duh" << std::endl;
	}
	subscriber_demo_bq.shutdown();
	return 0;
}
