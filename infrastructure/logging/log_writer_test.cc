#include "log_writer.hh"

#include "infrastructure/comms/schemas/demo_message.hh"

int main() {
	jet::LogWriter log_writer("/logs/log1/", {"channel_0", "channel_1", "pose"});

	jet::DemoMessage demo_message;
	demo_message.content = "how's it going?";
	demo_message.header.sequence_number = 51;
	demo_message.header.timestamp_ns = 4514782913789;

	std::string serialized_demo_message;
	demo_message.serialize(serialized_demo_message);

	log_writer.write_message(0, serialized_demo_message);
	return 0;
}
