
#include <Arduino.h>
#include <thermistor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Wire.h>

// Definición de pines para la pantalla TFT
#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 8
#define TFT_SCLK 13
#define TFT_MOSI 11

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Variables
volatile float measure_temp = 0.00;
volatile float measure_resistance = 0.00;
volatile int adc_raw = 0;

// Inputs/Outputs
int button = 3;
int ssr = 6;
int thermistor_pin = A7;
int fan = 2;
int led1 = A0;
int led2 = A1;

unsigned int millis_before, millis_before_2;
unsigned int millis_now = 0;
float refresh_rate = 500;
float pid_refres_rate = 50;
float seconds = 0;
int running_mode = 0;
int selected_mode = 0;
int max_modes = 3;
float temperatura = 0;
float preheat_setoint = 140;
float soak_setpoint = 150;
float reflow_setpoint = 200;
float temp_setpoint = 0;
float pwm_value = 255;
float min_pid_value = 0;
float max_pid_value = 180;
float cooldown_temp = 40;
bool button_state = true;

float Kp = 2;      // Mine was 2
float Ki = 0.0025; // Mine was 0.0025
float Kd = 9;      // Mine was 9
float PID_Output = 0;
float PID_P, PID_I, PID_D;
float PID_ERROR, PREV_ERROR;

thermistor therm(A7, 0);

void setup()
{
  pinMode(ssr, OUTPUT);
  digitalWrite(ssr, HIGH);
  pinMode(fan, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(button, INPUT_PULLUP);
  pinMode(thermistor_pin, INPUT);
  Serial.begin(9600);

  SPI.begin();
  tft.initR(INITR_144GREENTAB);
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);

  tft.setCursor(10, 10);
  tft.println("SMD Soldering");
  Serial.println("Sistema de soldadura SMD iniciado");

  millis_before = millis();
  millis_now = millis();
}

void loop()
{
  millis_now = millis();
  if (millis_now - millis_before_2 > pid_refres_rate)
  {
    millis_before_2 = millis_now;

    temperatura = therm.analog2temp();

    if (running_mode == 1)
    {
      if (temperatura < preheat_setoint)
      {
        temp_setpoint = seconds * 1.666;
      }
      if (temperatura > preheat_setoint && seconds < 90)
      {
        temp_setpoint = soak_setpoint;
      }
      else if (seconds > 90 && seconds < 110)
      {
        temp_setpoint = reflow_setpoint;
      }

      // PID
      PID_ERROR = temp_setpoint - temperatura;
      PID_P = Kp * PID_ERROR;
      PID_I = PID_I + (Ki * PID_ERROR);
      PID_D = Kd * (PID_ERROR - PREV_ERROR);
      PID_Output = PID_P + PID_I + PID_D;

      if (PID_Output > max_pid_value)
        PID_Output = max_pid_value;
      else if (PID_Output < min_pid_value)
        PID_Output = min_pid_value;

      pwm_value = 255 - PID_Output;
      analogWrite(ssr, pwm_value);
      PREV_ERROR = PID_ERROR;

      if (seconds > 130)
      {
        digitalWrite(ssr, HIGH);
        running_mode = 10; // Pasar a modo cooldown
      }
    }

    if (running_mode == 10)
    {
      tft.fillScreen(ST7735_BLACK);
      tft.setCursor(10, 50);
      tft.println("   Completado   ");
      Serial.println("Proceso completado");

      seconds = 0;
      running_mode = 11;
      delay(3000);
    }
  }
  // Refresco de pantalla TFT
  if (millis_now - millis_before > refresh_rate)
  {
    millis_before = millis();
    seconds += (refresh_rate / 1000);

    tft.fillScreen(ST7735_BLACK); // Borrar la pantalla

    Serial.print("T: ");
    Serial.print(temperatura, 1);
    Serial.print("°C | ");

    if (running_mode == 0)
    {
      digitalWrite(ssr, HIGH);
      tft.setCursor(10, 10);
      tft.print("T: ");
      tft.print(temperatura, 1);
      tft.setCursor(10, 30);
      tft.print("SSR OFF");
      tft.setCursor(10, 50);
      tft.print("Presiona para iniciar");

      Serial.println("Modo inactivo - Presiona botón para iniciar");
    }
    else if (running_mode == 11)
    {
      if (temperatura < cooldown_temp)
      {
        running_mode = 0;
      }
      digitalWrite(ssr, HIGH);
      tft.setCursor(10, 10);
      tft.print("T: ");
      tft.print(temperatura, 1);
      tft.setCursor(10, 30);
      tft.print("SSR OFF");
      tft.setCursor(10, 50);
      tft.print("    COOLDOWN    ");

      Serial.println("Modo cooldown...");
    }
    else if (running_mode == 1)
    {
      tft.setCursor(10, 10);
      tft.print("T: ");
      tft.print(temperatura, 1);
      tft.setCursor(10, 30);
      tft.print("SSR ON");
      tft.setCursor(10, 50);
      tft.print("S");
      tft.print(temp_setpoint, 0);
      tft.setCursor(10, 70);
      tft.print("PWM: ");
      tft.print(pwm_value, 0);
      tft.setCursor(10, 90);
      tft.print("Tiempo: ");
      tft.print(seconds, 0);
      tft.print("s");

      Serial.print("Setpoint: ");
      Serial.print(temp_setpoint, 0);
      Serial.print("°C | PWM: ");
      Serial.print(pwm_value, 0);
      Serial.print(" | Tiempo: ");
      Serial.print(seconds, 0);
      Serial.println("s");
    }
  }

  // Detección del único botón
  if (digitalRead(button) && !button_state)
  {
    button_state = true;

    if (running_mode == 1)
    {
      // Si está corriendo, cancela el proceso
      digitalWrite(ssr, HIGH);
      running_mode = 0;
      Serial.println("Proceso cancelado.");
    }
    else if (running_mode == 0)
    {
      // Si está apagado, inicia el proceso
      running_mode = 1;
      seconds = 0;
      Serial.println("Proceso iniciado.");
    }
  }
  else if (!digitalRead(button) && button_state)
  {
    button_state = false;
  }
}