,fkefkenke,f,
#include <Arduino.h> 
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_LIS3MDL.h> 

// === Définition des identifiants Wi-Fi ===
const char* ssid = "TON_SSID";
const char* password = "TON_MOT_DE_PASSE";

// === Déclaration et initialisation des capteurs ===
Adafruit_LSM6DS3 gyro;
Adafruit_LIS3MDL magneto;

// === Variables globales pour le suivi de l'orientation ===
float angle_reference = 0.0;
float angle_actuel = 0.0;
float offset_gyro_z = 0.0;

// === Définition des broches ===
#define LEDU1 25
#define LEDU2 26
#define EN_D 23
#define EN_G 4
#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16
#define SDA_PIN 21
#define SCL_PIN 22
#define ADDR_IMU 0x6B
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27
#define CTRL2_G  0x11
#define CTRL3_C  0x12

// === Définition des broches encodeurs ===
#define ENC_G_CH_A 32
#define ENC_D_CH_A 27

// === PWM configuration ===
#define PWM_CH_IN1_G 0
#define PWM_CH_IN2_G 1
#define PWM_CH_IN1_D 2
#define PWM_CH_IN2_D 3
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

volatile long ticks_d = 0;
volatile long ticks_g = 0;

// Calibration ticks
#define TICKS_PAR_20CM 5000

void IRAM_ATTR countTicksD() {
  ticks_d++;
}

void IRAM_ATTR countTicksG() {
  ticks_g++;
}

void controlerMoteurGauche(int8_t vitesse) {
  vitesse = constrain(vitesse, -255, 255);
  if (vitesse < 0) {
    ledcWrite(PWM_CH_IN1_G, -vitesse);
    ledcWrite(PWM_CH_IN2_G, 0);
  } else {
    ledcWrite(PWM_CH_IN1_G, 0);
    ledcWrite(PWM_CH_IN2_G, vitesse);
  }
}

void controlerMoteurDroit(int8_t vitesse) {
  vitesse = constrain(vitesse, -255, 255);
  if (vitesse < 0) {
    ledcWrite(PWM_CH_IN1_D, 0);
    ledcWrite(PWM_CH_IN2_D, -vitesse);
  } else {
    ledcWrite(PWM_CH_IN1_D, vitesse);
    ledcWrite(PWM_CH_IN2_D, 0);
  }
}

void calibrerGyroscope() {
  sensors_event_t acc_event, gyro_event;
  const int mesures = 500;
  float somme_offset = 0;

  Serial.println("Calibrage du gyroscope en cours... Ne pas bouger le robot.");
  for (int i = 0; i < mesures; i++) {
    gyro.getAccelerometerSensor()->getEvent(&acc_event);
    gyro.getGyroSensor()->getEvent(&gyro_event);
    somme_offset += gyro_event.gyro.z;
    delay(10);
  }
  offset_gyro_z = somme_offset / mesures;
  Serial.print("Offset du gyroscope Z : ");
  delay(300);
  Serial.println(offset_gyro_z, 6);
  delay(300);
  Serial.println("Calibrage terminé !");
  delay(300);
}

void avancer(float distance_cm) {
  long ticks_cible = (TICKS_PAR_20CM * distance_cm) / 20;
  ticks_d = 0;
  ticks_g = 0;
  angle_actuel = 0;
  angle_reference = 0;

  unsigned long dernier_temps = millis();

  while (true) {
    unsigned long temps_actuel = millis();
    float delta_t = (temps_actuel - dernier_temps) / 1000.0;
    dernier_temps = temps_actuel;

    sensors_event_t acc_event, gyro_event;
    gyro.getAccelerometerSensor()->getEvent(&acc_event);
    gyro.getGyroSensor()->getEvent(&gyro_event);

    float vitesse_z = gyro_event.gyro.z - offset_gyro_z;
    float delta_angle = vitesse_z * delta_t * 180.0 / PI;
    angle_actuel += delta_angle;

    float erreur_angle = angle_reference - angle_actuel;
    float k = 5.0;
    int correction = erreur_angle * k;

    int vitesse_base = 155;
    int vitesse_gauche = constrain(vitesse_base - correction, -255, 255);
    int vitesse_droite = constrain(vitesse_base + correction, -255, 255);

    controlerMoteurGauche(vitesse_gauche);
    controlerMoteurDroit(vitesse_droite);

    long moyenne_ticks = (ticks_d + ticks_g) / 2;
    float distance_parcourue = (moyenne_ticks * 20.0) / TICKS_PAR_20CM;

    Serial.print("Ticks D: ");
    Serial.print(ticks_d);
    Serial.print(" | Ticks G: ");
    Serial.print(ticks_g);
    Serial.print(" | Moyenne: ");
    Serial.print(moyenne_ticks);
    Serial.print(" | Distance parcourue (cm): ");
    Serial.println(distance_parcourue);

    if (moyenne_ticks >= ticks_cible) {
      controlerMoteurGauche(0);
      controlerMoteurDroit(0);
      Serial.println("Distance atteinte. Robot arrêté.");
      break;
    }

    delay(100);
  }
}

void setup() {
  Serial.begin(115200);

  // Pins moteurs
  pinMode(IN_1_D, OUTPUT);
  pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT);
  pinMode(IN_2_G, OUTPUT);
  pinMode(EN_D, OUTPUT);
  pinMode(EN_G, OUTPUT);
  digitalWrite(EN_D, HIGH);
  digitalWrite(EN_G, HIGH);

  // PWM setup
  ledcSetup(PWM_CH_IN1_G, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_IN2_G, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_IN1_D, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_IN2_D, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(IN_1_G, PWM_CH_IN1_G);
  ledcAttachPin(IN_2_G, PWM_CH_IN2_G);
  ledcAttachPin(IN_1_D, PWM_CH_IN1_D);
  ledcAttachPin(IN_2_D, PWM_CH_IN2_D);

  // Encodeurs
  pinMode(ENC_D_CH_A, INPUT_PULLUP);
  pinMode(ENC_G_CH_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), countTicksD, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), countTicksG, RISING);

  // Init capteurs
  if (!gyro.begin_I2C(0x6B)) {
    Serial.println("Erreur : LSM6DS3 non détecté !");
    while (1);
  }
  if (!magneto.begin_I2C(0x1E)) {
    Serial.println("Erreur : LIS3MDL non détecté !");
    while (1);
  } 

  calibrerGyroscope();
}

void loop() {
  avancerDistance(1000.0);  // ← ← ← modifie ici pour changer la distance (en cm)
  while (1);  // Empêche de relancer la fonction en boucle
}
