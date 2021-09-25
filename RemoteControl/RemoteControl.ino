#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

enum EtatTurbo {
  idle,
  actif,
  recharge
};

// Definitions des pins
#define PIN_JOY_X A0
#define PIN_JOY_Y A1
#define PIN_ACCEL A2
#define PIN_TURBO 4 

// Pins pour le nRF24L01
#define NRF_CE  7
#define NRF_CSN 8

// Definition de valeurs
#define DEADZONE 0.1f
#define DUREE_TURBO 2000
#define DUREE_RECHARGE 2000

// Declaration des variables globales
float joyX = 0.0f;
float joyY = 0.0f;
float accl = 0.0f;
bool btnTurbo = false;

uint32_t debutTurbo;
uint32_t debutRecharge;
EtatTurbo etatTurbo = idle;


RF24 radio(NRF_CE, NRF_CSN);

const byte address[6] = "test1";

// Prototypes des fonctions
void update_input_values();
void update_etat_turbo();
void send_motor_and_turbo_values();

void setup(){
  pinMode(PIN_TURBO, INPUT_PULLUP);
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop(){
  update_input_values();
  update_etat_turbo();
  send_motor_and_turbo_values();
}

// Une version de "map()" pour floats.
float mapf(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float remapDeadzone(float value, float deadzone){
  if(value >  deadzone) return mapf(value,  deadzone, 1.0f, 0.0f, 1.0f);
  if(value < -deadzone) return mapf(value, -1.0f, -deadzone, -1.0f, 0.0f);

  return 0.0f;
}

// Map une valeur entre "fromLower-fromUpper" vers "toLower-toUpper", avec une deadzone entre "(-deadzone)-(+deadzone)"
float normalizePlusDeadzone(float value, float fromLower, float fromUpper, float toLower, float toUpper, float deadzone){
  return remapDeadzone(mapf(value, fromLower, fromUpper, toLower, toUpper), deadzone);
}

void update_input_values(){
  int rawJoyX = analogRead(PIN_JOY_X);
  int rawJoyY = analogRead(PIN_JOY_Y);
  int rawAccl = analogRead(PIN_ACCEL);

  joyX = normalizePlusDeadzone(rawJoyX, 0, 1023, -1, 1, 0.1f);
  joyY = normalizePlusDeadzone(rawJoyY, 0, 1023, -1, 1, 0.1f);
  accl = normalizePlusDeadzone(rawAccl, 0, 1023, 0, 1, 0.1f);
  btnTurbo = digitalRead(PIN_TURBO) == LOW;
}

void update_etat_turbo(){
  switch(etatTurbo){
    case idle:
      if(btnTurbo){
        etatTurbo = actif;
        debutTurbo = millis();
      }
      break;

    case actif:
      if(millis() - debutTurbo > DUREE_TURBO){
        etatTurbo = recharge;
        debutRecharge = millis();
      }
      break;
    
    case recharge:
      if(millis() - debutRecharge > DUREE_RECHARGE){
        etatTurbo = idle;
      }
  }
}

void send_motor_and_turbo_values(){
  float gauche = constrain( joyX + joyY, -1, 1) * accl;
  float droite = constrain(-joyX + joyY, -1, 1) * accl;

  // Pour la telecommande on pourrait envoyer les données pures avec Serial.write()
  // Pour faire simple on peut faire avec 9 octets 4+4 pour les deux floats du moteurs + 1 pour le turbo.
  // Serial.println(String(gauche) + ", " + droite + ", " + etatTurbo);

  // Créer une union entre les donnés "utiles" et un tableau d'octets de meme longueur, permettant de réinterpréter les données en octets.
  union {
    struct {
      float gauche;
      float droite;
      bool turbo;
    } donnees;
    byte bytes[sizeof(donnees)];
  } u;
  
  u.donnees.gauche = gauche;
  u.donnees.droite = droite;
  u.donnees.turbo = (etatTurbo == actif);

  // Serial.write(u.bytes, sizeof(u.bytes));
  radio.write(u.bytes, sizeof(u.bytes));
}