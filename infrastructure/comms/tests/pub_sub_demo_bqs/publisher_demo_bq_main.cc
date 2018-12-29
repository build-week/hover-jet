#include "publisher_demo_bq.hh"

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

	jet::PublisherDemoBq publisher_demo_bq = jet::PublisherDemoBq();
	publisher_demo_bq.init();
	while (!shutdown)
	{
		publisher_demo_bq.loop();
		usleep(1000000);
	}
	publisher_demo_bq.shutdown();
	return 0;
}
