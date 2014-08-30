#include <MotoShield.h>
MotoShield motor;

#define encoder 21

/*Variavel counter*
 *Tipo volatile unsigned int
 *Descricao: Utilizada para verificar a variacao angular.
  Note o uso do modificador de tipo "volatile". Como se trata de uma variavel global
  que sera incrementada em uma ISR (Interrupt Signal Routine), e aconselhavel utiliza-la como volatile,
  pois desse modo ela nao sera armazenada em registradores, mas sim na ram.
*/
 volatile unsigned int counter = 0;
 unsigned long t0, tf = 0; // Tempos finais e iniciais, para calculo da variacao temporal 
 unsigned int firstCount = 0, lastCount = 0; //variaveis que armazenam valores da variavel counter antes e depois de um pequeno delay
 double angle = 0.12566; //angulo entre faixas ou furos do disco 

void setup() {
  // put your setup code here, to run once:
  pinMode(encoder, INPUT); // Define o pino encoder (2) como entrada.
  attachInterrupt(2, rpm_counter, RISING); //relaciona a interrupcao 0 (pino 2 no arduino mega 2560), a funcao rpm_counter, que incrementa um contador
  Serial.begin(9600); // Inicia a comunicacao serial, para exibicao de dados
  motor.begin(15);
}

//Incrementa contador
void rpm_counter()
{
  counter++; 
}

//Obs.: A funcao millis() retorna o tempo em milissegundos desde que o atual programa comecou a ser executado
void loop() {
  motor.go(M2, CCW, 255);
  t0 = millis(); //atualiza o tempo inicial
  firstCount = counter; //Pega atual valor do contador
  
  delay(100); // delay de 100 milissegundos

  lastCount = counter; // pega valor do contador
  tf = millis(); //atualiza tempo final
  
  // Interrompe as interrupcoes para realizar calculos matematicos
  // uma vez que as interrupcoes podem prejudicar a precisao dos calculos
  noInterrupts();
  
  /*
  Aqui e calculado a variacao angular ocorrida, multiplicado a variacao dos valores do contador
  com o valor angular entre as faixas ou furos do disco de encoder
  */
  float deltaTheta = (float) (lastCount - firstCount) * angle;
  float deltaT = (float) (tf - t0)/1000;  //calculo da variacao temporal
  
  if(deltaTheta > 0){ //Caso a variacao angular seja maior que zero
  
  float angular_vel = deltaTheta/deltaT; //calculo da velocidade angular
  
  Serial.print("A velocidade angular e: ");
  Serial.print(angular_vel);
  Serial.println("rad/s");  
  } // fim do if
  
  interrupts();  
}
