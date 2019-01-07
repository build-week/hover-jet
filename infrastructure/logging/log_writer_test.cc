#include "log_reader.hh"
#include "log_writer.hh"

#include "infrastructure/comms/schemas/demo_message.hh"

int main() {
	std::vector<std::string> channels = {"channel_0", "channel_1", "pose"};
	std::string log_path;

	jet::DemoMessage demo_message_write;
	demo_message_write.content = "how's it going?";
	demo_message_write.header.sequence_number = 51;
	demo_message_write.header.timestamp_ns = 4514782913789;

	// Write test
	{
		jet::LogWriter log_writer("/logs/log1/", {"channel_0", "channel_1", "pose"});
		std::string serialized_demo_message;
		demo_message_write.serialize(serialized_demo_message);
		log_writer.write_message(channels[0], serialized_demo_message);
		log_path = log_writer.get_log_path();
		std::cout << "Wrote to log: " << log_path << std::endl;
	}

	// Simple read test
	{
		jet::LogReader log_reader(log_path, channels);
		jet::DemoMessage demo_message_read;
		if (!log_reader.read_next_message(channels[0], demo_message_read)) {
			std::cerr << "Failed to read message from log." << std::endl;
		} else {
			if (demo_message_read.content == demo_message_write.content) {
				std::cout << "[Simple Read Test] Written message and read mesasge have same content." << std::endl;
			} else {
				std::cerr << "[Simple Read Test] Failure: Content of read message does not match content of written message." << std::endl;
			}

			if (demo_message_write.header.timestamp_ns == demo_message_read.header.timestamp_ns) {
				std::cout << "[Simple Read Test] Written message and read mesasge have same timestamps." << std::endl;
			} else {
				std::cerr << "[Simple Read Test] Failure: Timestamp of read message does not match timestamp of written message." << std::endl;
			}
		}
	}

	// Polymorphic read test
	{
		jet::LogReader log_reader(log_path, channels);
		jet::Message base_message;
		if (!log_reader.read_next_message(channels[0], base_message)) {
			std::cerr << "Failed to read message from log." << std::endl;
		} else {
			if (demo_message_write.header.timestamp_ns == base_message.header.timestamp_ns) {
				std::cout << "[Polymorphic Read Test] Written message and read mesasge have same timestamps." << std::endl;
			} else {
				std::cerr << "[Polymorphic Read Test] Failure: Timestamp of read message does not match timestamp of written message." << std::endl;
			}
			if (demo_message_write.header.sequence_number == base_message.header.sequence_number) {
				std::cout << "[Polymorphic Read Test] Written message and read mesasge have same sequence numbers." << std::endl;
			} else {
				std::cerr << "[Polymorphic Read Test] Failure: Sequence number of read message does not match sequence number of written message." << std::endl;
			}
		}
	}

	return 0;
}
