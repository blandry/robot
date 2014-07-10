#ifndef SERIAL_H
#define SERIAL_H

#include <iostream>
#include <boost/asio.hpp>


class Serial
{

public:

    /*!CONSTRUTOR
     * Recebe como parâmetros baud_rate (velocidade de transimissão dos dados) e o nome
     * da porta serial
     * ---windows: COM1, COM2, COM3 etc
     * ---unix/linux: /dev/ttyACM0, /dev/ttyUSB0 etc
     *
     * também inicializa os objetos boost::asio::io_service e boost::asio::serial_port,
     * necessários para comunicação com hardware.
    */

    Serial();

    Serial(unsigned int baud_rate, const std::string serial_port,
           boost::asio::serial_port_base::parity opt_parity = boost::asio::serial_port_base::parity(), //none
           boost::asio::serial_port_base::character_size opt_size = boost::asio::serial_port_base::character_size(8),
           boost::asio::serial_port_base::flow_control opt_flow = boost::asio::serial_port_base::flow_control(), //none
           boost::asio::serial_port_base::stop_bits opt_stop = boost::asio::serial_port_base::stop_bits()
           );

    void open(unsigned int baud_rate, const std::string serial_port,
              boost::asio::serial_port_base::parity opt_parity = boost::asio::serial_port_base::parity(), //none
              boost::asio::serial_port_base::character_size opt_size = boost::asio::serial_port_base::character_size(8),
              boost::asio::serial_port_base::flow_control opt_flow = boost::asio::serial_port_base::flow_control(), //none
              boost::asio::serial_port_base::stop_bits opt_stop = boost::asio::serial_port_base::stop_bits()
              );

    void close();

    bool isOpen();


    /*!begin().
    * serve para iniciar a comunicação serial, visto a propriedade
    *de auto-reset do arduino.
    */


    void begin();

    //funções membros

    /*!
     * \brief print
     * \param some_string < sends a string through the serial port
     */

    void print(std::string some_string);

    void write(const unsigned char* data, size_t size);
    void write(std::vector<char>& data);

    void wait_ms(unsigned short int time);
    void wait_s(unsigned short int time);
    void wait_min(unsigned short int time);
    void wait_h(unsigned short int time);

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    boost::asio::deadline_timer wait_time;

};

#endif // SERIAL_H
