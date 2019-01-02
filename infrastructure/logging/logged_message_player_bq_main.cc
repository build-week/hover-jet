#include "logged_message_player_bq.hh"

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

	jet::LoggedMessagePlayerBQ logged_message_player_bq = jet::LoggedMessagePlayerBQ();
	logged_message_player_bq.init();
	while (!shutdown)
	{
		logged_message_player_bq.loop();
		usleep(5000);
	}
	logged_message_player_bq.shutdown();
	return 0;
}
