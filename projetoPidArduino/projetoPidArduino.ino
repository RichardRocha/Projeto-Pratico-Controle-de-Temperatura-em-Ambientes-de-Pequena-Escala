#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1_bc.h> 

// Definições
#define ONE_WIRE_BUS 2
#define PWM_PIN 9
#define TEMP_THRESHOLD 32.0        // Temperatura para ligar
#define MIN_PWM_EFFECTIVE 245      // PWM mínimo para ventoinha girar

// Faixa exibida no display
#define DISPLAY_RPM_MIN 1200
#define DISPLAY_RPM_MAX 1500

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Variáveis PID
double tempAtual, pwmPID, tempSetpoint = 32.0;

// Ganhos PID
double Kp = 140.0;
double Ki = 0.6;
double Kd = 3.0;

// PID em modo reverso
PID controlPID(&tempAtual, &pwmPID, &tempSetpoint, Kp, Ki, Kd, REVERSE);

// Estado da ventoinha
bool fanOn = false;

// Variável para suavização do RPM mostrado
static float rpmSuave = 0; 

void setup() {
  Serial.begin(9600);

  sensors.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Temp: --.- C");
  lcd.setCursor(0, 1);
  lcd.print("RPM: ----");

  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);

  controlPID.SetMode(AUTOMATIC);
  controlPID.SetOutputLimits(0, 255);
  controlPID.SetSampleTime(400);
}

void loop() {

  // Ler temperatura
  sensors.requestTemperatures();
  tempAtual = sensors.getTempCByIndex(0);

  if (tempAtual == DEVICE_DISCONNECTED_C) {
    Serial.println("Sensor desconectado!");
    lcd.setCursor(0, 0); lcd.print("Temp: ERRO ");
    lcd.setCursor(0, 1); lcd.print("RPM: ----   ");
    analogWrite(PWM_PIN, 0);
    fanOn = false;
    rpmSuave = 0;
    delay(1000);
    return;
  }


  // HISTERSE 32°C liga / 31°C desliga
  if (!fanOn && tempAtual >= TEMP_THRESHOLD)
      fanOn = true;

  if (fanOn && tempAtual <= (TEMP_THRESHOLD - 1.0))
      fanOn = false;

  // CONTROLE DO PWM (PID)
  int pwmValue = 0;

  if (fanOn) {
    controlPID.Compute();

    if (pwmPID < MIN_PWM_EFFECTIVE)
        pwmValue = MIN_PWM_EFFECTIVE;
    else
        pwmValue = constrain((int)pwmPID, MIN_PWM_EFFECTIVE, 255);

  } else {
    pwmValue = 0;
  }

  analogWrite(PWM_PIN, pwmValue);

  // RPM PROGRESSIVO
  int rpmAlvo = 0;

  if (fanOn && pwmValue >= MIN_PWM_EFFECTIVE) {
      rpmAlvo = map(
          pwmValue,
          MIN_PWM_EFFECTIVE, 255,
          DISPLAY_RPM_MIN, DISPLAY_RPM_MAX
      );
  } else {
      rpmAlvo = 0;  // abaixo de 31°C, RPM = 0
  }


  if (!fanOn) {
      // Quando desligado → queda rápida
      rpmSuave += (rpmAlvo - rpmSuave) * 0.45;
  } else {
      // Quando ligado → subida suave
      rpmSuave += (rpmAlvo - rpmSuave) * 0.30;
  }

  int rpm = (int)rpmSuave;

  // -----------------------
  // LCD
  // -----------------------
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(tempAtual, 1);
  lcd.print(" C   ");

  lcd.setCursor(0, 1);
  lcd.print("RPM: ");
  lcd.print(rpm);
  lcd.print("   ");

  Serial.print("Temp: ");
  Serial.print(tempAtual, 2);
  Serial.print(" | pwmPID: ");
  Serial.print(pwmPID, 2);
  Serial.print(" | PWM usado: ");
  Serial.print(pwmValue);
  Serial.print(" | RPM alvo: ");
  Serial.print(rpmAlvo);
  Serial.print(" | RPM suave: ");
  Serial.println(rpm);

  delay(300);
}
