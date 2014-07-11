#include <iostream>

#include "CmdMessenger.h"

using namespace std;

enum
{
  ksetled,
  ksend
};

class callback
{
  public:

  callback(){}
  ~callback(){}

  static void method(cmd::CmdReceived& command)
  {
    cout << "\n---Command received: " << command.getId() << " Argument: " << command.parseInt() << endl;
  }
};

void CB(cmd::CmdReceived& command)
{
  cout << "\n----Command received: " << command.getId() << " Argument: " << command.parseInt() << endl;
}

int main(int argc, char *argv[])
{
  if(argc == 2){

    callback cb;

    char op;
    cmd::CmdMessenger arduino(argv[1], 9600);
    cmd::Cmd led(ksetled);

    cmd::Timeout timeout = cmd::Timeout::simpleTimeout(1000);
    arduino.setTimeout(timeout);

    arduino.attach(ksend, &CB);
    arduino.attach(callback::method);

    cmd::Cmd testing(4);

    while(true)
    {
      cout << "'l' liga, 'd' desliga, 'q' sai: ";
      cin >> op;
      cout << endl;

      switch(op)
      {
        case 'l':
          led << true << CmdEnd();
          arduino.send(led);
          led.clear();
          break;

        case 'd':
          led << false << CmdEnd();
          arduino.send(led);
          led.clear();
          break;

        case 'q':
          break;
      }

      if(op == 'q') break;

      cout << "feeding in serial data\n";

      arduino.feedInSerialData(10);

    }

  }
  else
  {
    cout << "Uso: led 'port'\n";
  }

  return 0;
}
