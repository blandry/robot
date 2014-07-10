#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include "limiter.hpp"
#include "tracker.hpp"
#include "serial.hpp"
#include "libconfig.h++"

using namespace std;
using namespace libconfig;

Config smart_conf;

int main(int argc, char** argv)
{
    try
    {
        smart_conf.readFile("smart.conf");
    }
    catch(const libconfig::FileIOException &fioex)
    {
        std::cerr << "Erro de I/O" << endl;
        return EXIT_FAILURE;
    }
    catch(const libconfig::ParseException &pex)
    {
        std::cout << "erro de Parser no arquivo " << pex.getFile() << ", linha:" << pex.getLine() << " - " << pex.getError() << endl;

        return EXIT_FAILURE;
    }


    if(argc == 2){

        if(argv[1] == std::string("mouse")){

            int lh, ls, lv, hh, hs, hv;
            int pos_x;
            int pos_y;

            int l_pos_x = 0; 
            int l_pos_y = 0;
            
            std::stringstream ss;
            std::string s;

            try
            {
                Setting &limits = smart_conf.lookup( "limits" );
                
                lh = limits.operator[](0);
                ls = limits.operator[](1);
                lv = limits.operator[](2);

                hh = limits.operator[](3);
                hs = limits.operator[](4);
                hv = limits.operator[](5);

            }
            catch( const libconfig::SettingNotFoundException &snfex )
            {
                cout << "nome de configuração '" << snfex.getPath() << "' não encontrado, edite o arquivo 'smart.conf'" << endl;

                return EXIT_FAILURE;
            }

            tracker track;

            track.getCapture(0);
            track.setRanges(cvScalar(lh, ls, lv), cvScalar(hh, hs, hv));

            while(1)
            {
                track.getFrame();
                track.threshold();
                track.visual_track();

                pos_x = (int) (1366*track.getXpos())/640;
                pos_y = (int) (768*track.getYpos())/480;

                if(pos_x < 0) pos_x = l_pos_x;
                else if(pos_x > 1366) pos_x = l_pos_x;

                if(pos_y < 0) pos_y = l_pos_y;
                else if(pos_y > 768) pos_y = l_pos_y;

                ss << "xdotool mousemove " << pos_x << " " << pos_y;
                s = ss.str();

                system(s.c_str());

                l_pos_x = pos_x;
                l_pos_y = pos_y;

                if(cv::waitKey(30) == 27) break;
            }

        }

        //MODO COMUNICAÇÃO SERIAL
        
        else if(argv[1] == std::string("serial")){

            unsigned char angleX[1];  //Variável responsàvel por armazenar a posição no eixo X.  
            unsigned char angleY[1];  //Variável responsável por armazenar a posição no eixo Y.

            unsigned char flagX[1] = {200}; //Variável utilizada para informar ao 'receptor Serial' que a mensagem a ser enviada se trata da posição no eixo X
            unsigned char flagY[1] = {100}; //Variável utilizada para informar ao 'receptor Serial' que a mensagem a ser enviada se trata da posição no exio Y

            try
            {
                std::string port = smart_conf.lookup( "serial_port" );
                cout << "a porta serial é: " << port << endl;
                Serial receptor(9600, port);

               tracker track; //Objeto 'tracker', responsável por rastrear o ponto central do objeto reconhecido na etapa da definição dos limites de threshold

               int lh, ls, lv, hh, hs, hv;

               Setting &limits = smart_conf.lookup( "limits" );

               lh = limits.operator[](0);
               ls = limits.operator[](1);
               lv = limits.operator[](2);

               hh = limits.operator[](3);
               hs = limits.operator[](4);
               hv = limits.operator[](5);

               track.getCapture(0);
               track.setRanges( cvScalar(lh, ls, lv), cvScalar(hh, hs, hv) );

               while(1)
               {
                   track.getFrame();
                   track.threshold();
                   track.visual_track();

                   if(track.getChangedXstate()){

                           angleX[0] = (unsigned char)3*track.getXpos()/8;

                           receptor.write( flagX, sizeof(flagX) );
                           receptor.write( angleX, sizeof(angleX) ); 
                    }

                   if(track.getChangedYstate()){

                       angleY[0] = (unsigned char)3*track.getYpos()/8;

                       receptor.write( flagY, sizeof(flagY) );
                       receptor.write( angleY, sizeof(angleY) );
                   }

                   if(cv::waitKey(30) == 27) break;
               }

               track.~tracker();
               receptor.~Serial();
            }
            catch(const libconfig::SettingNotFoundException &snfex)
            {
                std::cerr << "nome de configuração " <<  snfex.getPath() << " não encontrado, edite o arquivo 'smart.conf'" << endl;
                return EXIT_FAILURE;
            }
            

        }else if(argv[1] == std::string("app")){

            int lh, ls, lv, hh, hs, hv;
            const char* up, *down, *right, *left;
            int flagUp = 0; 
            int flagDown = 0; 
            int flagLeft = 0; 
            int flagRight = 0;

            try
            {
                Setting &limits = smart_conf.lookup( "limits" );

                lh = limits.operator[](0);
                ls = limits.operator[](1);
                lv = limits.operator[](2);

                hh = limits.operator[](3);
                hs = limits.operator[](4);
                hv = limits.operator[](5);

                const char* UP = (const char*) smart_conf.lookup( "apps.UP" );
                const char* DOWN = smart_conf.lookup( "apps.DOWN" );
                const char* RIGHT = smart_conf.lookup( "apps.RIGHT" );
                const char* LEFT = smart_conf.lookup( "apps.LEFT" );

                up = UP;
                down = DOWN;
                right = RIGHT;
                left = LEFT;
            }
            catch( const libconfig::SettingNotFoundException &snfex )
            {
                cout << "nome de configuração " << snfex.getPath() << " não encontrado, edite o arquivo 'smart.conf'" << endl;
                return EXIT_FAILURE;
            }

            tracker track;

            track.getCapture(0);
            track.setRanges( cvScalar(lh, ls, lv), cvScalar(hh, hs, hv) );

            while(1)
            {
                track.getFrame();
                track.threshold();
                track.visual_track();

                int quad = track.getQuadrant();

                switch(quad)
                {
                    case 1:
                        flagUp++; 
                        flagDown = 0;
                        flagLeft = 0;
                        flagRight = 0;

                        if(flagUp == x_times){

                           system(up); 

                        }

                        break;

                    case 2:
                        flagUp = 0;
                        flagDown++;
                        flagLeft = 0;
                        flagRight = 0;

                        if(flagDown == x_times){

                            system(down);

                        }

                        break;

                    case 3:
                        flagUp = 0;
                        flagDown = 0;
                        flagLeft++;
                        flagRight = 0;

                        if(flagLeft == x_times){

                            system(left);

                        }

                        break;

                    case 4:
                        flagUp = 0;
                        flagDown = 0;
                        flagLeft = 0;
                        flagRight++;

                        if(flagRight == x_times){

                            system(right);

                        }

                        break;

                }

                if(cv::waitKey(30) == 27) break;
            }


        }else{

            cout << "Modo de execução '" << argv[1] << "' não encontrado" << endl;

            return EXIT_FAILURE;
        }

    }else if(argc == 3){

        if(argv[1] == std::string("set")){

            if(argv[2] == std::string("limits")){

                limiter range;
                int lh, ls, lv, uh, us, uv;

                range.setWindowParams();
                range.get_capture(0);
                range.run();

                range.getLimits(lh, ls, lv, uh, us, uv);

                try
                {
                Setting &limits = smart_conf.lookup("limits");

                limits.operator[](0) = lh;
                limits.operator[](1) = ls;
                limits.operator[](2) = lv;
                
                limits.operator[](3) = uh;
                limits.operator[](4) = us;
                limits.operator[](5) = uv;

                smart_conf.writeFile("smart.conf");
                }

                catch(const libconfig::SettingNotFoundException &snfex)
                {
                    std::cerr << "Nome de configuração " << snfex.getPath() <<  " não foi encontrado, edite o arquivo 'smart.conf'" << endl;
                    
                    return EXIT_FAILURE;
                }

            }

        }

    }

    return 0;

}
