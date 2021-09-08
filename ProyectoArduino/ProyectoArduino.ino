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
#include "confi.h"

Servo miServo;

//********************************************************************************
// Definicion de pines
//********************************************************************************

#define btn 23
#define delayBtn 500

#define pinAnalog 34

#define IO_USERNAME "Karla19374"
#define IO_KEY "aio_jdhP11FUf4GYIZldZl6K5i8Jk6nd"

#define WIFI_SSID "TIGO-3281"
#define WIFI_PASS "2NB144207118"

#define segA 21
#define segB 19
#define segC 18
#define segD 22
#define segE 17
#define segF 16
#define segG 4
#define segdp 2

#define display1 14
#define display2 12
#define display3 13

// Paso 1: selección de parámetros de la señal PWM
#define pwmChannel 0
#define pwmChanneln 1
#define pwmChannelv 3
#define pwmChannelr 5 // 16 canales 0-15
#define freqPWM 5000  // Frecuencia en Hz
#define resolution 8  // 1-16 bits de resolución

#define pinPWMr 25
#define pinPWMn 26
#define pinPWMv 32
#define pinPWM 27

#define pinServo 33

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
void mediaMovilADC(void);
void emaADC(void);
void configurarPWM(void);
//void display7(void);

//********************************************************************************
// Variables globales
//********************************************************************************
int adcRaw = 0;
float voltaje = 0;

int numeroLecturas = 2;
int sumaLecturas = 0;
float promedioLecturas = 0;

int numLecturas = 2;     // Número de muestras
float bufferLecturas[2]; // Buffer de lecturas
int indexLecturas = 0;   // índice de muestras
long mAvgSuma = 0;       // Valor de adc media movil
long adcFiltrado = 0;    // Valor de adc filtrado

double adcFiltradoEMA = 0; // S(0) = Y(0)
double alpha = 0.09;       // Factor de suavizado (0-1)

int valDecena = 0;
int valUnidad = 0;
int valDecima = 0;

int i = 0;
int btnPress = 0;
int lastTime = 0;

unsigned long lastTime7seg;
unsigned int sampleTime7seg = 25;

//********************************************************************************
// ISR
//********************************************************************************
void ISR()
{
    if (digitalRead(btn == HIGH))
        btnPress = 1;
    else
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

    configurarDisplay(segA, segB, segC, segD, segE, segF, segG, segdp);

    pinMode(display1, OUTPUT);
    pinMode(display2, OUTPUT);
    pinMode(display3, OUTPUT);

    digitalWrite(display1, HIGH);
    digitalWrite(display2, HIGH);
    digitalWrite(display3, HIGH);
    desplegarSeg(0);

    configurarPWM();
    ledcSetup(pwmChannel, freqPWM, resolution);
    ledcAttachPin(pinPWM, pwmChannel);

    ledcSetup(pwmChanneln, freqPWM, resolution);
    ledcAttachPin(pinPWMn, pwmChanneln);

    ledcSetup(pwmChannelr, freqPWM, resolution);
    ledcAttachPin(pinPWMr, pwmChannelr);

    ledcSetup(pwmChannelv, freqPWM, resolution);
    ledcAttachPin(pinPWMv, pwmChannelv);
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
    while (btnPress == 1)
    {
        if (millis() - lastTime > delayBtn)
        {
            adcRaw = analogRead(pinAnalog);

            mediaMovilADC();
            emaADC();
            
            io.run();

            // save count to the 'counter' feed on Adafruit IO
            Serial.print("sending -> ");
            Serial.print(voltaje);
            nivelFeed->save(voltaje);

            Serial.print('\t');
            Serial.print(voltaje);
           
            delay(3000);
            configurarPWM();
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

    if ((voltaje > 5.00) && (voltaje < 10.00))
    {
        for (int dutycycle = 0; dutycycle <= 50; dutycycle++)
        {
            ledcWrite(pwmChannelv, dutycycle);
            delay(5);
        }
        for (int dutycycle = 49; dutycycle >= 0; dutycycle--)
        {
            ledcWrite(pwmChannelv, dutycycle);
            delay(5);
        }

        Serial.print("verde");
        miServo.write(30);
    }

    if ((voltaje > 10.00) && (voltaje < 13.50))
    {
        for (int dutycycle = 0; dutycycle < 50; dutycycle++)
        {
            ledcWrite(pwmChanneln, dutycycle);
            delay(5);
        }
        for (int dutycycle = 49; dutycycle > 0; dutycycle--)
        {
            ledcWrite(pwmChanneln, dutycycle);
            delay(5);
        }

        Serial.print("naranja");
        miServo.write(90);
    }

    if (voltaje > 13.50)
    {
        for (int dutycycle = 0; dutycycle < 50; dutycycle++)
        {
            ledcWrite(pwmChannelr, dutycycle);
            delay(5);
        }
        for (int dutycycle = 49; dutycycle > 0; dutycycle--)
        {
            ledcWrite(pwmChannelr, dutycycle);
            delay(5);
        }

        Serial.print("rojo");
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
