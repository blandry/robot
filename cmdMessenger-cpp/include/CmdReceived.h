/*!
 * \file CmdReceived.h
 *
 * \author Rodrigues Filho
 * \version 0.1
 *
 * \section LICENSE
 *
 * The MIT License
 *
 * Copyright (c) 2014 Francisco Edno de Moura Rodrigues Filho
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be
 *  included in all copies or substantial portions of the Software.
 *   
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 *  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 *  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 *  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 * This is the class responsible to hold the incoming command and for parsing the arguments.  
 */

#ifndef CMD_RECEIVED_H
#define CMD_RECEIVED_H

#include <iostream>
#include <queue>

#include "CmdBase.h"

namespace cmd{

  class CmdReceived : public CmdBase
  {
    public:

      /*----------CTOR | DTOR----------*/

      CmdReceived(std::string command = "", const char field_separator = ',', const char cmd_separator = ';', const char esc_character = '/');
      virtual ~CmdReceived();

      /*----------PARSERS----------*/

      bool parseBool();
      void parseBool(bool& arg);

      int parseInt();
      void parseInt(int& arg);

      char parseChar();
      void parseChar(char& arg);

      float parseFloat();
      void parseFloat(float& arg);

      double parseDouble();
      void parseDouble(double& arg);
      
      std::string parseString();
      void parseString(std::string& arg);

      /*----------UTILITY----------*/

      bool isValid();
      int getNumArgs();

    private:
      std::queue<std::string> args_; //holds the arguments to be processed.
      bool validity_; //says if this is a valid command or, if it is incomplete.

      char field_separator_;
      char cmd_separator_;
      char esc_character_;

      void setId(int id);
  };

}

#endif
