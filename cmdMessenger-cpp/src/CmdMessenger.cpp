/*!
 * \file CmdMessenger.cpp
 * \author Rodrigues Filho
 * \version 0.1
 *
 * \section LICENSE
 * \todo this license
 * 
 * \section Description
 * This file implements the main class of this project, the cmd::CmdMessenger class.
 */

#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>

#include "CmdSend.h"
#include "CmdMessenger.h"

using namespace cmd; 

/*----------CTOR | DTOR----------*/

CmdMessenger::CmdMessenger(const std::string &port,
    uint32_t baudrate,
    const char field_separator,
    const char cmd_separator,
    const char esc_character,
    Timeout timeout,
    bytesize_t bytesize,
    parity_t parity,
    stopbits_t stopbits,
    flowcontrol_t flowcontrol)
  :serial_port_(port, baudrate, timeout, bytesize, parity, stopbits, flowcontrol),
  field_separator_(field_separator),
  cmd_separator_(cmd_separator),
  esc_character_(esc_character)
{
  default_callback_ = 0;
}

CmdMessenger::~CmdMessenger()
{
}

/*----------CMD MESSENGER SPECIFIC----------*/

bool CmdMessenger::send(const Cmd &command, bool ack, int ack_id, int simple_timeout)
{
  std::stringstream ss; //stream to hold the command to be sent.

  if(!command.getNumArgs()) 
    ss << command.getId() << cmd_separator_; // if there are no arguments to be sent, the stream receives only the command id and the command separator.
  else
  {
    //otherwise, if there are some arguments, we will have to iterate over the std::vector args_    
    ss << command.getId() << field_separator_;    

    for(unsigned long i=0; i<command.args_.size() - 1; i++){ //Note that we go from i = 0, to i = args_.size() - 1, because after the last argument a cmd_separator must be added.
      ss << command.args_[i] << field_separator_; //adds an argument and the field separator to the stream.
    }

    ss << command.args_.back() << cmd_separator_; //finishes the command. Now it is ready to be sent.

  } // else

  serial_port_.write(ss.str()); //sends the command!
  
  if(ack){ // if ack is required 
    return CmdMessenger::waitCmd(ack_id, simple_timeout); //Verifies if the command arrived or not and returns true if received (false otherwise).
  } 

  return true; //if ack is not required, the function returns true anyway.
}

bool CmdMessenger::waitCmd(int cmd_id, int simple_timeout)
{
}

void CmdMessenger::attach(CallBack callback)
{
  default_callback_ = callback;
}

void CmdMessenger::attach(int cmd_id, CallBack callback)
{
  callbacks_[cmd_id] = callback;
}

void CmdMessenger::feedInSerialData(int num_commands)
{
  std::vector<std::string> raw_commands = serial_port_.readlines(num_commands, ";"); //Gets the 'raw commands' in the buffer.

  for(unsigned long i = 0; i<raw_commands.size(); i++){ //fills in the commands vector
    CmdReceived command(raw_commands[i], field_separator_, cmd_separator_, esc_character_);

    std::cout << raw_commands[i] << std::endl;

    if(callbacks_.count(command.getId())){

    if(command.isValid())
      callbacks_[command.getId()](command);

    }
    else
    {
      if(default_callback_) default_callback_(command);
    }
  } 
}
/*----------SERIAL SPECIFIC----------*/

//These methods simply call the methods in the serial::Serial serial_port_ object. For more information about each one
//refer to the serial::Serial documentation!

void CmdMessenger::open()
{
  serial_port_.open();
}

bool CmdMessenger::isOpen() const
{
  return serial_port_.isOpen();
}

void CmdMessenger::close()
{
  serial_port_.close();
}

/*----------SETTERS AND GETTERS----------*/

void CmdMessenger::setFieldSep(const char field_separator)
{
  field_separator_ = field_separator;
}

char CmdMessenger::getFieldSep() const
{
  return field_separator_;
}

void CmdMessenger::setCmdSep( const char cmd_separator )
{
  cmd_separator_ = cmd_separator;
}

char CmdMessenger::getCmdSep() const
{
  return cmd_separator_;
}

void CmdMessenger::setEscChar( const char esc_character )
{
  esc_character_ = esc_character;
}

char CmdMessenger::getEscChar() const
{
  return esc_character_;
}

void CmdMessenger::setPort(const std::string &port)
{
  serial_port_.setPort(port);
}

void CmdMessenger::setTimeout(Timeout &timeout)
{
  serial_port_.setTimeout(timeout);
}

void CmdMessenger::setTimeout(uint32_t inter_byte_timeout, uint32_t read_timeout_constant, uint32_t read_timeout_multiplier, uint32_t write_timeout_constant, uint32_t write_timeout_multiplier)
{
  serial_port_.setTimeout( inter_byte_timeout, read_timeout_constant, read_timeout_multiplier, write_timeout_constant, write_timeout_multiplier);
}

std::string CmdMessenger::getPort() const
{
  return serial_port_.getPort();
}

void CmdMessenger::setBaudRate(uint32_t baudrate)
{
  serial_port_.setBaudrate(baudrate);
}

uint32_t CmdMessenger::getBaudrate() const
{
  return serial_port_.getBaudrate();
}

void CmdMessenger::setByteSize(bytesize_t bytesize)
{
  serial_port_.setBytesize(bytesize);
}

bytesize_t CmdMessenger::getByteSize() const
{
  return serial_port_.getBytesize();
}

void CmdMessenger::setParity(parity_t parity)
{
  serial_port_.setParity(parity);
}

parity_t CmdMessenger::getParity() const
{
  return serial_port_.getParity();
}

void CmdMessenger::setStopBits(stopbits_t stopbits)
{
  serial_port_.setStopbits(stopbits);
}

stopbits_t CmdMessenger::getStopBits() const
{
  return serial_port_.getStopbits();
}

void CmdMessenger::setFlowControl(flowcontrol_t flowcontrol)
{
  serial_port_.setFlowcontrol(flowcontrol);
}

flowcontrol_t CmdMessenger::getFlowControl() const
{
  return serial_port_.getFlowcontrol();
} 
