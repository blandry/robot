#include <iostream>
#include "serial/serial.h"

using namespace std;
using namespace serial;

int main(int argc, char *argv[])
{
  if(argc == 2){
    
    Serial arduino;

    arduino.setBaudrate(9600);
    arduino.setPort(argv[1]);
    arduino.open();

    Timeout timeout = Timeout::simpleTimeout(1000);

    arduino.setTimeout(timeout);

    char c = 'a';
    while(true)
    {
      cout << "Digite 'r' para ler e 'q' para sair: ";
      cin >> c;

      if(c == 'r') {
        
        std::string str;

        str = arduino.readline(65535, ";");

        cout << "\nString lida tem tamanho de " << str.size() << " seu conteúdo é: " << str << endl;
      }
      else if(c == 'q') break;
    }

  }
  else
  {
    cout << "Uso: teste 'port'\n";
  }

  return 0; 
}
