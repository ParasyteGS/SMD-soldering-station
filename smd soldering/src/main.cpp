
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <TFT_ILI9163C.h>
#include <SPI.h>
#include <Wire.h>

// Color definitions
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

// Definición de pines para la pantalla TFT
#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 8
#define TFT_SCLK 13
#define TFT_MOSI 11
#define T0 298.15 // [K]
#define E 2.71828 // [#]

// PINs
#define PIN_button 6
#define PIN_ssr 3
#define PIN_thermistor A7
#define PIN_fan 2
#define PIN_led1 A0
#define PIN_led2 A1

// Parámetros del NTC
#define BETA 3950       // Coeficiente Beta
#define R_NTC_25 100000 // Resistencia del NTC a 25°C
#define R_PULLUP 4700   // 4.7kΩ

TFT_ILI9163C tft = TFT_ILI9163C(TFT_CS, TFT_DC, TFT_RST);

// Temperature variables
double Setpoint;
volatile float measured_temp = 0;
volatile float measured_resistance = 0;
volatile int adc_raw = 0;

void SetTitle();
void MeasureTemp();

void setup()
{
  pinMode(PIN_button, INPUT);
  pinMode(PIN_ssr, OUTPUT);
  pinMode(PIN_fan, OUTPUT);
  pinMode(PIN_led1, OUTPUT);
  pinMode(PIN_led2, OUTPUT);
  pinMode(PIN_button, INPUT_PULLUP);
  digitalWrite(PIN_button, HIGH);

  SPI.begin();
  tft.begin();

  SetTitle();
  delay(5000);
  tft.fillScreen(BLACK);
}

void loop()
{
}

void SetTitle()
{
  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 20);
  tft.print("SMD");
  tft.setCursor(20, 50);
  tft.print("Soldering");
}

void MeasureTemp()
{
  adc_raw = analogRead(PIN_thermistor);
  // Calcular la resistencia del NTC
  measured_resistance = R_PULLUP / ((1023.00 / adc_raw) - 1);
  // Aplicar ecuación de Steinhart-Hart
  measured_temp = BETA / log(measured_resistance / (R_NTC_25 * pow(E, -BETA / T0))) - 273.15;
}
