#define entrada 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(entrada, INPUT);
  pinMode(13, OUTPUT);
 }


void loop() {

  // put your main code here, to run repeatedly:
  int numBytes = Serial.available();
  if(numBytes){
    Serial.println("Eu recebi do buffer: ");
    Serial.print("--> ");
    for(int i = 0; i < numBytes; i++){
      char c = Serial.read();
      Serial.print(c);
    }
    Serial.println("");
    delay(500);
  }
  
  if(digitalRead(entrada)){
    Serial.println("agora eu vou testar simplesmente o envio e recebimento de strings muito grandes so mesmo para zuar com os nego hahahahahahahhahahahahahahahaha... E aew serial, tanca msm?");
  }
}
