/*
 Sensor Input Smoothing
 Demonstrates smoothing of a sensor input via various methods.


 Created by Matthew Fryer

 This example code is in the public domain.

 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Smoothed.h> 	// Include the library

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

unsigned int pos0=172; // ancho de pulso en cuentas para pocicion 0°. 
unsigned int pos180=565; // ancho de pulso en cuentas para la pocicion 180°
 
#define SENSOR_PIN A0    // The input pin for the sensor. In this example we are reading from an Arduino analogue pin. 
// If you don't have a sensor you can still see the effect if the analogue pin is left floating its value will vary wildly.

// Create two instances of the class to use. 
Smoothed <float> mySensor; 
float basal=0;


void setup() {
	Serial.begin(9600);
 servos.begin();  
  servos.setPWMFreq(60); //Frecuecia PWM de 60Hz o T=16,66ms //Ponemos una frecuencia de funcionamiento de los servos

	// Initialise the first sensor value store. We want this to be the simple average of the last 10 values.
	// Note: The more values you store, the more memory will be used.
	mySensor.begin(SMOOTHED_AVERAGE, 100);	

    // Although it is unnecessary here, the stored values can be cleared if needed.
    mySensor.clear();

}

void setServo(uint8_t n_servo, int angulo) { //Quiere un entero que va a ser el número de servo y el ángulo de servo
  int duty;//define la relación entre el tiempo de encendido y apagado. Mitad encendido, mitad acabado
  duty=map(angulo,0,180,pos0, pos180);//la función escala. El ángulo que va de 0 a 180 tienes que escalármelo entre 0 y 180.
  //Cando sea 0 quiero que sea pos0(172), el duty te dice 0,5 (quiero la mitad de voltaje 50%on y 50% off)
  servos.setPWM(n_servo, 0, duty); //función para poner el servo   
}

void loop() {
    // Read the value from the sensor
    float currentSensorValue = abs(analogRead(SENSOR_PIN)-240 );
   
    mySensor.add(currentSensorValue);
    // Get the smoothed values
    float smoothedSensorValueAvg = mySensor.get(); 
    if(smoothedSensorValueAvg> 40)
    {
      digitalWrite(LED_BUILTIN,HIGH); 
   setServo(0,180);//Los pones a 180
    setServo(1,180);
    }
    else
    {
      digitalWrite(LED_BUILTIN,LOW);
        setServo(0,0);//Los pones a 180
    setServo(1,0);
    }
    
Serial.println(smoothedSensorValueAvg);
   

    delay(3);
}
