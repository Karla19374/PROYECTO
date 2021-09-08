//********************************************************************************
//Universidad del Valle de Guatemala
//BE3015 Electrónica Digital 2
//Karla Melissa López
//Proyecto 1
//********************************************************************************

//********************************************************************************
// Librerías
//********************************************************************************
#include <Arduino.h>
#include <ESP32Servo.h>
#include "s7.h"

Servo miServo;

//********************************************************************************
// Definicion de pines
//********************************************************************************
#define btn 23       // pushbutton
#define delayBtn 500 // delay del pushbutton
#define pinAnalog 34 // GPIO para el sensor lm35

// selección de parámetros de la señal PWM
#define pwmChannel 5 // 16 canales 0-15
#define freqPWM 5000 // frecuencia en Hz
#define resolution 8 // 8 bits de resolución

#define pinPWMr 25 // led roja
#define pinPWMn 26 // led naranja
#define pinPWMv 27 // led verde

#define pinServo 33 // señal para el servo

#define segA 21 // pines para los displays
#define segB 19
#define segC 18
#define segD 22
#define segE 17
#define segF 16
#define segG 4
#define segdp 2

#define display1 14 //transistor para los displays
#define display2 12
#define display3 13

//********************************************************************************
// Prototipos de funciones
//********************************************************************************
void mediaMovilADC(void); // funcion para Filtro media Móvil
void emaADC(void);        // funcion para Filtro media Móvil xponencial EMA
void configurarPWM(void); // funcion para configurar el semáforo

//********************************************************************************
// Variables globales
//********************************************************************************
int adcRaw = 0;    // valor adc entero
float voltaje = 0; // valor del voltaje obtenida de la temperatura del sensor

int numeroLecturas = 5;     // numero de lecturas para el promedio
int sumaLecturas = 0;       // inicio 0 para la suma de lecturas
float promedioLecturas = 0; // numero racional para lecturas

int numLecturas = 5;     // número de muestras de la lectura de la señal
float bufferLecturas[5]; // buffer de lecturas
int indexLecturas = 0;   // indice de muestras
long mAvgSuma = 0;       // valor de adc media movil
long adcFiltrado = 0;    // valor de adc filtrado

double adcFiltradoEMA = 0; // S(0) = Y(0)
double alpha = 0.09;       // factor de suavizado (0.9)

int i = 0;        // interrupcion del pushbutton
int btnPress = 0; // valor al presionar el boton
int lastTime = 0; // valor para evitar el anti rebote del pushbutton

int valDecena = 0;
int valUnidad = 0;
int valDecima = 0;

//********************************************************************************
// ISR
//********************************************************************************
void ISR() // funcion de interrupcion para el pushbutton
{
  if (digitalRead(btn == HIGH)) // si el pushbutton está en estado alto valor es 1
    btnPress = 1;
  else // si el pushbutton no está presionado el valore es 0
    btnPress = 0;
}

//********************************************************************************
// Configuración
//********************************************************************************
void setup()
{
  Serial.begin(115200);

  miServo.attach(pinServo);
  miServo.write(0);

  pinMode(btn, INPUT_PULLUP);
  attachInterrupt(btn, ISR, HIGH);

  configurarPWM();

   configurarDisplay(segA, segB, segC, segD, segE, segF, segG, segdp);

    pinMode(display1, OUTPUT);
    pinMode(display2, OUTPUT);
    pinMode(display3, OUTPUT);

    digitalWrite(display1, HIGH);
    digitalWrite(display2, HIGH);
    digitalWrite(display3, HIGH);
    desplegarSeg(0);

  pinMode(pinPWMv, OUTPUT);
  pinMode(pinPWMr, OUTPUT);
  pinMode(pinPWMn, OUTPUT);

  digitalWrite(pinPWMr, LOW);
  digitalWrite(pinPWMv, LOW);
  digitalWrite(pinPWMn, LOW);
}

//********************************************************************************
// Loop principal
//********************************************************************************
void loop()
{
  while (btnPress == 1) // si el pushbutton se presiona se ejecuta el bucle
  {
    if (millis() - lastTime > delayBtn) // si lastTime es menor al delay del boton, se ejecuta el if, para evitar el anti rebote
    {

      adcRaw = analogRead(pinAnalog); // valor del lm35

      mediaMovilADC(); // ejecurta el modulo mediaMovilADC
      emaADC();        // ejecuta el modulo emaADC

      Serial.print('\t');
      Serial.println(voltaje); // se muesta el valor del voltaje en serial monitor

      configurarPWM(); // ejecutar el modulo de PWM para el semáforo

      for (int dutycycle = 0; dutycycle < 20; dutycycle++) // señal para el PWM
      {
        ledcWrite(pwmChannel, dutycycle);
        delay(5);
      }
      for (int dutycycle = 19; dutycycle > 0; dutycycle--)
      {
        ledcWrite(pwmChannel, dutycycle);
        delay(5);
      }
    }
    btnPress = 0; // si el pushbutton no se presiona no se ejecuta nada más
  }

  if (millis() - lastTime7seg >= sampleTime7seg ){
        lastTime7seg = millis();
        digitalWrite(display1, HIGH);
      digitalWrite(display2, LOW);
      digitalWrite(display3, LOW);
      desplegarSeg(valDecena);
      desplegarPun(0);
      delay(5);

      digitalWrite(display1, LOW);
      digitalWrite(display2, HIGH);
      digitalWrite(display3, LOW);
      desplegarSeg(valUnidad);
      desplegarPun(1);
      delay(5);

      digitalWrite(display1, LOW);
      digitalWrite(display2, LOW);
      digitalWrite(display3, HIGH);
      desplegarSeg(valDecima);
      desplegarPun(0);
      delay(5);

    }
}

//****************************************************************
// Filtro media Móvil
//****************************************************************
void mediaMovilADC(void)
{
  adcRaw = analogRead(pinAnalog);                      // valor adcRaw igual al valor del lm35
  mAvgSuma = mAvgSuma - bufferLecturas[indexLecturas]; // Se resta el último valor y se suma el nuevo valor
  mAvgSuma = mAvgSuma + adcRaw;
  bufferLecturas[indexLecturas] = adcRaw; // Se actualiza el valor
  indexLecturas++;                        // Se incrementa el índice
  if (indexLecturas >= numLecturas)
  {
    indexLecturas = 0;
  }
  adcFiltrado = mAvgSuma / numLecturas;
}
//****************************************************************
// Filtro media Móvil exponencial EMA
//****************************************************************
void emaADC(void)
{
  adcRaw = analogRead(pinAnalog);                                       // valor adcRaw igual al valor del lm35
  adcFiltradoEMA = (alpha * adcRaw) + ((1.0 - alpha) * adcFiltradoEMA); // filtrado EMA
  voltaje = ((adcFiltradoEMA * 5000 / 4095.0) / 10);                    // formula para pasar de adcRaw a grados
  valDecena = (voltaje / 10);
    valUnidad = (voltaje - (valDecena * 10));
    valDecima = ((voltaje * 10) - (valDecena * 100 + valUnidad * 10));
}

//****************************************************************
// Configuracion PWM para LEDS
//****************************************************************
void configurarPWM(void)
{
    ledcSetup(pwmChannel, freqPWM, resolution);

    if (voltaje < 10.00)
    {
        ledcAttachPin(pinPWMv, pwmChannel);
        Serial.print("verder");
        miServo.write(30);
    }

    if ((voltaje > 10.00) && (voltaje < 13.50))
    {
        ledcAttachPin(pinPWMn, pwmChannel);
        Serial.print("naranja");
        miServo.write(90);
    }

    if (voltaje >= 13.50)
    {
        ledcAttachPin(pinPWMr, pwmChannel);
        Serial.print("rojo");
        miServo.write(150);
        
    }
}
