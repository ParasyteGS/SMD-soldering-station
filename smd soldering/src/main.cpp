
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

#define T0 298.15 // [K]
#define E 2.71828 // [#]

// PINs
#define PIN_switch 6
#define PIN_ssr 3
#define PIN_thermistor A7
#define PIN_fan 2
#define PIN_led1 A0
#define PIN_led2 A1

// Parámetros del NTC
#define BETA 3950       // Coeficiente Beta
#define R_NTC_25 100000 // Resistencia del NTC a 25°C
#define R_PULLUP 4700   // 4.7kΩ

LiquidCrystal_I2C lcd(0x3F, 16, 2);

// Temperature variables

volatile float measured_temp = 0;
volatile float measured_resistance = 0;
volatile int adc_raw = 0;

// PID Varibles

double Setpoint, Input, Output;
double Kp = 2, Ki = 0.0025, Kd = 9;
PID PidSSR(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long startTime;
int etapa = 0;

static bool wasOn = false;

void MeasureTemp();
void ledsVentilador_State_temperatura();
void ViewTemp_States();
void Switch_on_off();
void ViewTemp_States();
void AjustarSetpoint();
void setup()
{
  pinMode(PIN_switch, INPUT);
  pinMode(PIN_ssr, OUTPUT);
  pinMode(PIN_fan, OUTPUT);
  pinMode(PIN_led1, OUTPUT);
  pinMode(PIN_led2, OUTPUT);
  pinMode(PIN_switch, INPUT_PULLUP);
  digitalWrite(PIN_ssr, LOW);
  digitalWrite(PIN_fan, LOW);
  digitalWrite(PIN_led1, LOW);
  digitalWrite(PIN_led2, LOW);
  digitalWrite(PIN_switch, HIGH);

  Setpoint = 150;
  PidSSR.SetMode(AUTOMATIC);
  PidSSR.SetOutputLimits(0, 255);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SMD Soldering");

  delay(2000);
}

void loop()
{

  MeasureTemp();
  Input = measured_temp;
  ViewTemp_States();
  Switch_on_off();
  delay(100);
}

void Switch_on_off()
{
  if (digitalRead(PIN_switch) == LOW)
  {
    if (!wasOn)
    {
      startTime = millis();
      wasOn = true;
    }

    AjustarSetpoint();
    PidSSR.Compute();
    analogWrite(PIN_ssr, Output);
    ledsVentilador_State_temperatura();
  }
  else
  {
    digitalWrite(PIN_ssr, LOW);
    digitalWrite(PIN_led1, LOW);
    digitalWrite(PIN_led2, LOW);

    if (measured_temp >= 30)
    {
      digitalWrite(PIN_fan, HIGH);
      etapa = 4;
    }
    else
    {
      digitalWrite(PIN_fan, LOW);
      etapa = 0;
    }
    Output = 0;
    wasOn = false;
  }
}

void AjustarSetpoint()
{
  unsigned long elapsed = (millis() - startTime) / 1000;

  if (elapsed < 90)
  {
    Setpoint = map(elapsed, 0, 90, measured_temp, 150);
    etapa = 1;
  }

  else if (elapsed < 150)
  {
    Setpoint = 180;
    etapa = 2;
  }

  else if (elapsed < 180)
  {
    Setpoint = map(elapsed, 150, 180, 180, 240);
    etapa = 3;
  }

  else
  {
    Setpoint = 25;
    etapa = 4;
  }
}

void ledsVentilador_State_temperatura()
{
  if (measured_temp >= 30)
  {
    digitalWrite(PIN_led1, HIGH);
  }
  else
  {
    digitalWrite(PIN_led1, LOW);
  }

  if (etapa == 4)
  {
    digitalWrite(PIN_led2, HIGH);
    digitalWrite(PIN_fan, HIGH);
  }
  else
  {
    digitalWrite(PIN_led2, LOW);
  }
}

void ViewTemp_States()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.print(measured_temp);
  lcd.print("C");

  lcd.setCursor(9, 0);
  lcd.print("S: ");
  lcd.print(Setpoint);
  lcd.print("C");

  if (etapa == 0)
  {
    lcd.setCursor(0, 1);
    lcd.print("Wait");
  }
  else if (etapa == 1)
  {
    lcd.setCursor(0, 1);
    lcd.print("Hot");
  }
  else if (etapa == 2)
  {
    lcd.setCursor(0, 1);
    lcd.print("Soak");
  }
  else if (etapa == 3)
  {
    lcd.setCursor(0, 1);
    lcd.print("Reflow");
  }
  else if (etapa == 4)
  {
    lcd.setCursor(0, 1);
    lcd.print("CoolDown");
  }
}

void MeasureTemp()
{
  adc_raw = analogRead(PIN_thermistor);
  // Calcular la resistencia del NTC
  measured_resistance = R_PULLUP / ((1023.00 / adc_raw) - 1);
  // Aplicar ecuación de Steinhart-Hart
  measured_temp = BETA / log(measured_resistance / (R_NTC_25 * pow(E, -BETA / T0))) - 273.15;
}
