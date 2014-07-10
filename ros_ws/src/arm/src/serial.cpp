#include "serial.hpp"

//Serial::Serial() : Serial::io(), Serial::serial(Serial::io), Serial::wait_time(Serial::io) {}

Serial::Serial(unsigned int baud_rate, const std::string serial_port, boost::asio::serial_port_base::parity opt_parity, boost::asio::serial_port_base::character_size opt_size, boost::asio::serial_port_base::flow_control opt_flow, boost::asio::serial_port_base::stop_bits opt_stop) :io(), serial(Serial::io), wait_time(Serial::io)
{
    Serial::open(baud_rate, serial_port, opt_parity, opt_size, opt_flow, opt_stop);
}

void Serial::open(unsigned int baud_rate, const std::string serial_port, boost::asio::serial_port_base::parity opt_parity, boost::asio::serial_port_base::character_size opt_size, boost::asio::serial_port_base::flow_control opt_flow, boost::asio::serial_port_base::stop_bits opt_stop)
{
    if(Serial::isOpen()) Serial::close();

    Serial::serial.open(serial_port);
    Serial::serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    Serial::serial.set_option(opt_parity);
    Serial::serial.set_option(opt_size);
    Serial::serial.set_option(opt_flow);
    Serial::serial.set_option(opt_stop);
}

bool Serial::isOpen()
{
    return Serial::serial.is_open();
}

void Serial::close()
{
    Serial::serial.close();
}

void Serial::begin()
{
    Serial::print("begin");

    Serial::wait_time.expires_from_now(boost::posix_time::seconds(1));
    Serial::wait_time.wait();
}

void Serial::print(std::string some_string)
{
    boost::asio::write(Serial::serial, boost::asio::buffer(some_string.c_str(), some_string.size()));
}

void Serial::write(const unsigned char *data, size_t size)
{
    boost::asio::write(Serial::serial, boost::asio::buffer(data, size));
}

void Serial::wait_ms(unsigned short time)
{
    Serial::wait_time.expires_from_now(boost::posix_time::milliseconds(time));
    Serial::wait_time.wait();
}

void Serial::wait_s(unsigned short time)
{
    Serial::wait_time.expires_from_now(boost::posix_time::seconds(time));
    Serial::wait_time.wait();
}

void Serial::wait_min(unsigned short time)
{
    Serial::wait_time.expires_from_now(boost::posix_time::minutes(time));
    Serial::wait_time.wait();
}

void Serial::wait_h(unsigned short time)
{
    Serial::wait_time.expires_from_now(boost::posix_time::hours(time));
    Serial::wait_time.wait();
}
