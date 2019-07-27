//%bin(log_summarizer)

//%deps(yaml-cpp)
#include <optional>
#include "infrastructure/logging/log_summary.hh"
#include "infrastructure/config/config.hh"

std::optional<std::string> parse_commandline_arguments(int argc, char* argv[]) {
  if (argc > 1) {
    return argv[1];
  }
  return {};
}

int main(int argc, char* argv[]) {
  const std::optional<std::string> log_path = parse_commandline_arguments(argc, argv);
  Config log_config;
  if (!log_path.has_value()) {
    std::cout << "Please enter a log path" << std::endl;
    return 0;
  }

  jet::LogSummary summary = jet::LogSummary(*log_path);
  summary.print();
  
  return 0;
}

