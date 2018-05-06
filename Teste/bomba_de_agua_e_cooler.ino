//------------Código para a Bomba e Cooler
//----------- se pressionar botão liga e se soltar desliga
//-- Foi feito teste separadamente, cada componete da cada vez
//Por isso estão com os mesmo pinos 

//Pino do botão
const int pin_botao = 2; 
//var para o receber o valor do botão
int buttonState;
//pino do relê
const int porta_rele = 3; 

void setup() {
  // put your setup code here, to run once:
  // Define pino o botão
  pinMode(pin_botao, INPUT);
  //Define pino para o rele como saida
  pinMode(porta_rele, OUTPUT); 
  // para o Print
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  buttonState = digitalRead(2);
  Serial.println(buttonState);

  if (buttonState == 0){ //Desligar bomba
    digitalWrite(porta_rele, LOW);
    Serial.println("valor: 0");
  }
  else{ //Ligar bomba
    digitalWrite(porta_rele, HIGH);
    Serial.println("valor: 1");
  }

  Serial.println(buttonState);
  delay(300); 

}
