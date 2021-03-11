
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> //Usamos un driver
//Definimos las variables
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40); //

/*CONEXIONES:
 * ARDUINO -------- DRIVER
 * ------------------------
 * 3.3V    -------- Vcc    
 * 5V      -------- V+
 * GND     -------- GND
 * A4      -------- SDA
 * A5      -------- SCL
 * 
 * 
 */
 //Como mínimo usamos una frecuencia de muestreo el doble de la frecuencia de la onda que queremos muestrear
 //Tenemos un conversor de analógico digital de 10 bits, nos permiten registrar enteros de 3 cifras (1024 valores.
 //Nos viene la señal amplificada y filtrado
//int senal;
//int senal_ant[5]; //señal de 5 valores
//senal_ant=[0, 0, 0, 0, 0];
int i;
unsigned int pos0=172; // ancho de pulso en cuentas para pocicion 0°. No tenemos volta
unsigned int pos180=565; // ancho de pulso en cuentas para la pocicion 180°
//De 0 a 1024 tengo para dividir 5V por ejemplo. En 1024 tengo un voltaje medio de 5V. Si quiero 2,5V (designado con 562) estará mitad encendido y acabado
void setup() {
  //Definimos la función de inicialización (solo va a ocurrir una vez al principio)
  Serial.begin(9600);//Función de la librería serial. Inicializamos la señal
  servos.begin();  
  servos.setPWMFreq(60); //Frecuecia PWM de 60Hz o T=16,66ms //Ponemos una frecuencia de funcionamiento de los servos
}

void setServo(uint8_t n_servo, int angulo) { //Quiere un entero que va a ser el número de servo y el ángulo de servo
  int duty;//define la relación entre el tiempo de encendido y apagado. Mitad encendido, mitad acabado
  duty=map(angulo,0,180,pos0, pos180);//la función escala. El ángulo que va de 0 a 180 tienes que escalármelo entre 0 y 180.
  //Cando sea 0 quiero que sea pos0(172), el duty te dice 0,5 (quiero la mitad de voltaje 50%on y 50% off)
  servos.setPWM(n_servo, 0, duty); //función para poner el servo   
}

void loop() {
    senal = analogRead(A0); //Leemos la señal
    for (i=3;i=0,i++){
      senal_ant(i+1)=senal_ant[1];
    }
    senal_ant[0]=senal; //el primer elemento es la señal
    Serial.println(senal); //La escribimos

    if(senal > 350){
    
    setServo(0,0);//Pones los dos servos en la posición 0. numero de ervo y ángulo
    setServo(1,0); //función propia
    delay(1000); //Esperas unos segundos
    setServo(0,180);//Los pones a 180
    setServo(1,180);
    }
    delay(3);
    
}
