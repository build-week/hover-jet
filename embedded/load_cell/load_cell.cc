#include "load_cell.hh"

#include <iostream>
#include <sstream>

//%deps(${Boost_LIBRARIES}, pthread)

namespace jet {
    LoadCellReceiver::LoadCellReceiver() {
        const std::string SERIAL_PORT        = "/dev/ttyACM0";
        const unsigned int SERIAL_BAUD_RATE  = 38400;
        const unsigned int SERIAL_TIMEOUT_MS = 200;
           
        boost::asio::io_service io; 
        
        serial_port_ = std::make_unique<boost::asio::serial_port>(io);
        serial_port_->open(SERIAL_PORT);
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(SERIAL_BAUD_RATE));
    }

    void LoadCellReceiver::receive() {
        constexpr unsigned BUFFER_BYTE_SIZE = 42U;
        char read_msg_[BUFFER_BYTE_SIZE];
        boost::asio::read(*serial_port_, boost::asio::buffer(&read_msg_, BUFFER_BYTE_SIZE));
        std::cout << read_msg_ << std::endl;
    }

} // namespace jet 

