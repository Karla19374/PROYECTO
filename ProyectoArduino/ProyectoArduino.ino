//********************************************************************************
//Universidad del Valle de Guatemala
//BE3015 Electrónica Digital 2
//Karla Melissa López
//Proyecto
//********************************************************************************

//********************************************************************************
// Librerías
//********************************************************************************
#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"

Servo miServo;

//********************************************************************************
// Definicion de pines
//********************************************************************************

#define btn 23 // pushbutton
#define delayBtn 500 // delay del pushbutton

#define pinAnalog 34 // GPIO para el sensor lm35

#define IO_USERNAME "Karla19374"
#define IO_KEY "aio_jdhP11FUf4GYIZldZl6K5i8Jk6nd"

#define WIFI_SSID "TIGO-3281"
#define WIFI_PASS "2NB144207118"

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

// selección de parámetros de la señal PWM
#define pwmChannel 5 // 16 canales 0-15
#define freqPWM 5000 // frecuencia en Hz
#define resolution 8 // 8 bits de resolución

#define pinPWMr 25 // led roja
#define pinPWMn 26 // led naranja
#define pinPWMv 27 // led verde

#define pinServo 33 // señal para el servo

int lm35 = 0;

AdafruitIO_Feed *nivelFeed = io.feed("  nivel-de-temperatura");

extern uint8_t pinA, pinB, pinC, pinD, pinE, pinF, pinG, pindp;
uint8_t pinA, pinB, pinC, pinD, pinE, pinF, pinG, pindp;

void configurarDisplay(uint8_t A, uint8_t B, uint8_t C, uint8_t D, uint8_t E, uint8_t F, uint8_t G, uint8_t dp);
void desplegarSeg(uint8_t digito);
void desplegarPun(boolean punto);

//********************************************************************************
// Prototipos de funciones
//********************************************************************************
void mediaMovilADC(void); // funcion para Filtro media Móvil
void emaADC(void); // funcion para Filtro media Móvil xponencial EMA
void configurarPWM(void); // funcion para configurar el semáforo
//void display7(void);

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

int valDecena = 0;
int valUnidad = 0;
int valDecima = 0;

int i = 0;        // interrupcion del pushbutton
int btnPress = 0; // valor al presionar el boton
int lastTime = 0; // valor para evitar el anti rebote del pushbutton

unsigned long lastTime7seg;
unsigned int sampleTime7seg = 25;

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

  // wait for serial monitor to open
  while (!Serial)
    ;

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while (io.status() < AIO_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

  lastTime7seg = millis();

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
  if (millis() - lastTime7seg >= sampleTime7seg)
  {
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
  while (btnPress == 1) // si el pushbutton se presiona se ejecuta el bucle
  {
    if (millis() - lastTime > delayBtn) // si lastTime es menor al delay del boton, se ejecuta el if, para evitar el anti rebote
    {
      adcRaw = analogRead(pinAnalog);

      mediaMovilADC(); // ejecurta el modulo mediaMovilADC
      emaADC(); // ejecuta el modulo emaADC
      io.run();

      // save count to the 'counter' feed on Adafruit IO
      Serial.print("sending -> ");
      Serial.print(voltaje);
      nivelFeed->save(voltaje);
      
      Serial.print('\t');
      Serial.print(voltaje); // se muesta el valor del voltaje en serial monitor
      configurarPWM(); // ejecutar el modulo de PWM para el semáforo

      for (int dutycycle = 0; dutycycle < 20; dutycycle++)// señal para el PWM
      {
        ledcWrite(pwmChannel, dutycycle);
        delay(5);
      }
      for (int dutycycle = 19; dutycycle > 0; dutycycle--)
      {
        ledcWrite(pwmChannel, dutycycle);
        delay(5);
      }
      delay(3000);
    }
    btnPress = 0;
  }

  
}

//****************************************************************
// Filtro media Móvil
//****************************************************************
void mediaMovilADC(void)
{
  adcRaw = analogRead(pinAnalog);

  // Se resta el último valor y se suma el nuevo valor
  mAvgSuma = mAvgSuma - bufferLecturas[indexLecturas];
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

  adcRaw = analogRead(pinAnalog);
  adcFiltradoEMA = (alpha * adcRaw) + ((1.0 - alpha) * adcFiltradoEMA);
  voltaje = ((adcFiltradoEMA * 5000 / 4095.0) / 10);
  valDecena = (voltaje / 10);
  valUnidad = (voltaje - (valDecena * 10));
  valDecima = ((voltaje * 10) - (valDecena * 100 + valUnidad * 10));
}

//****************************************************************
// Filtro media Móvil exponencial EMA
//****************************************************************
void configurarPWM(void)
{
  ledcSetup(pwmChannel, freqPWM, resolution);

  if (voltaje < 10.00)
  {
    ledcAttachPin(pinPWMv, pwmChannel);
    Serial.println("verder");
    digitalWrite(pinPWMn, 0);
    digitalWrite(pinPWMr, 0);
    miServo.write(30);
  }

  if ((voltaje > 10.00) && (voltaje < 13.50))
  {
    ledcAttachPin(pinPWMn, pwmChannel);
    Serial.println("naranja");
    digitalWrite(pinPWMr, 0);
    digitalWrite(pinPWMv, 0);
    miServo.write(90);
  }

  if (voltaje >= 13.50)
  {
    ledcAttachPin(pinPWMr, pwmChannel);
    Serial.println("rojo");
    digitalWrite(pinPWMv, 0);
    digitalWrite(pinPWMn, 0);
    miServo.write(150);
  }
}

void configurarDisplay(uint8_t A, uint8_t B, uint8_t C, uint8_t D, uint8_t E, uint8_t F, uint8_t G, uint8_t dp)
{
  pinA = A;
  pinB = B;
  pinC = C;
  pinD = D;
  pinE = E;
  pinF = F;
  pinG = G;
  pindp = dp;

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  pinMode(pinD, OUTPUT);
  pinMode(pinE, OUTPUT);
  pinMode(pinF, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pindp, OUTPUT);

  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);
  digitalWrite(pinC, LOW);
  digitalWrite(pinD, LOW);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, LOW);
  digitalWrite(pinG, LOW);
  digitalWrite(pindp, LOW);
}

void desplegarSeg(uint8_t digito)
{
  switch (digito)
  {
  case 0:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, LOW);
    //digitalWrite(pindp, HIGH);
    break;

  case 1:
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, LOW);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, LOW);
    // digitalWrite(pindp, LOW);
    break;

  case 2:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, LOW);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, HIGH);
    // digitalWrite(pindp, LOW);
    break;

  case 3:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, HIGH);
    // digitalWrite(pindp, LOW);
    break;

  case 4:
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, LOW);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    // digitalWrite(pindp, LOW);
    break;

  case 5:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    // digitalWrite(pindp, LOW);
    break;

  case 6:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    // digitalWrite(pindp, LOW);
    break;

  case 7:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, LOW);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, LOW);
    // digitalWrite(pindp, LOW);
    break;

  case 8:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    // digitalWrite(pindp, LOW);
    break;
  case 9:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    // digitalWrite(pindp, LOW);
    break;

  default:
    break;
  }
}

void desplegarPun(boolean punto)
{
  if (punto == 1)
  {
    digitalWrite(pindp, HIGH);
  }
  else
  {
    digitalWrite(pindp, LOW);
  }
}
