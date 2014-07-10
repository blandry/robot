#file CMakeLists.txt

cmake_minimum_required(VERSION 2.6)

#Project Name
project( cmdMessenger-cpp )

#include
include_directories( ./include/ )

#executáveis
#add_executable( hello ./src/hello.cpp )
#add_executable( interbyte ./src/interbyte.cpp )
#add_executable( teste ./src/teste.cpp )
#add_executable( token ./src/token.cpp )
#add_executable( led ./src/led.cpp )
#add_executable( parse ./src/parser.cpp )

#bibliotecas
add_library(messenger2 SHARED ./src/CmdMessenger.cpp ./src/CmdBase.cpp ./src/CmdSend.cpp ./src/CmdReceived.cpp)
target_link_libraries(messenger2 serial)

#target_link_libraries( hello serial )
#target_link_libraries( interbyte serial )
#target_link_libraries( teste serial )
#target_link_libraries( led messenger2 )
