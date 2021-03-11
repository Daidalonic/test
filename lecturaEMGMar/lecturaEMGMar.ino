#include <Wire.h>

//Definimos las variables


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);//Función de la librería serial. Inicializamos la señal

}

void loop() {
  // put your main code here, to run repeatedly:
int senal = abs(analogRead(A0)-235);

Serial.println(senal); //La escribimos
delay(3);
}
