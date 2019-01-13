/*
///%deps(jet_serial)
//#include <serial/serial.h>
*/

#include <boost/asio.hpp> 
#include <boost/asio/serial_port.hpp> 

#include <memory>

namespace jet {

class LoadCellReceiver {
 public:
  LoadCellReceiver();
  void receive();

 private:
  std::unique_ptr<boost::asio::serial_port> serial_port_;

  //std::unique_ptr<serial::Serial> serial_port_;

  void tokenize(const std::string& str,
                std::vector<float>& tokens,
                char delim);
};

}  // namespace jet
