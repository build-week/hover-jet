#include "message_logger_bq.hh"

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

	jet::MessageLoggerBQ message_logger_bq = jet::MessageLoggerBQ();
	message_logger_bq.init();
	while (!shutdown)
	{
		message_logger_bq.loop();
		usleep(1);
	}
	message_logger_bq.shutdown();
	return 0;
}
