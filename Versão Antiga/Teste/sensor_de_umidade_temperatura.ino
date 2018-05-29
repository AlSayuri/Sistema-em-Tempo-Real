//Biblioteca para o sensor de temperatura e umidade do ar
//https://github.com/adafruit/DHT-sensor-library
#include <DHT.h>

//para o sensor de temperatura e umidade do ar
#define DHTTYPE DHT11   // DHT 22  (AM2302), AM2321

//Pino do sensor de umidade de solo
#define pino_umidade_solo A5

//Pino do sensor de umidade e r=temperatura do ar
#define DHTPIN 2 //Pino do sensor de temperatura e umidade
#define DHTTYPE DHT11 // DHT 11
//para o sensor de temperatura e umidade
DHT dht(DHTPIN, DHTTYPE);

//var para o receber o valor do sensor de umidade de solo
int v_umidade_solo;

//var para o receber a porcentagem da umidade do solo
int umidade_percentual;

//var para o receber o valor da umidade do ar
float v_umidade_ar;

//var para o receber o valor da temperatura do ar
float v_temperatura_ar;

void setup() {
  // put your setup code here, to run once:
  //Print
  Serial.begin(9600); 
  //Define pino para sensor de umidade de solo
  pinMode(pino_umidade_solo, INPUT);
  //para o sensor de temperatura e umidade
  dht.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  //umidade de solo
    v_umidade_solo = analogRead(pino_umidade_solo);
    umidade_percentual = 100 * ((978-(float)v_umidade_solo) / 978);
    Serial.print("Umidade do solo: ");
    Serial.print(v_umidade_solo);
    Serial.print("       ");
    Serial.print("Umidade do solo(%): ");
    Serial.print(umidade_percentual);
    Serial.println("%"); 
  //fim umidade de solo

  //temperatura e umidade do ar
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    Serial.print("Umidade ar: ");
    Serial.print(h);
    Serial.print("       ");
    Serial.print("Temperatura ar: ");
    Serial.print(t);
    Serial.println(" *C"); 
  //fim temperatura e umidade do ar
  Serial.println("--------------------------//----------------");
  delay(5000);
}
