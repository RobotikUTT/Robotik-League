#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

enum TurboState {
  idle,
  active,
  recharging
};

// Input pin definitions
#define PIN_JOY_X A3
#define PIN_JOY_Y A2
#define PIN_ACCEL A4
#define PIN_TURBO 4

// nRF24L01 pins
#define NRF_CE  7
#define NRF_CSN 8

// Fixed values
#define DEADZONE 0.1f
#define TURBO_ACTIVE_TIME 2000
#define TURBO_RECHARGE_TIME 2000

// Global variables
float joyX = 0.0f;
float joyY = 0.0f;
float accl = 0.0f;
bool turbo_btn_state = false;

uint32_t turbo_active_start;
uint32_t turbo_recharge_start;
TurboState turbo_state = idle;


RF24 radio(NRF_CE, NRF_CSN);

const byte address[6] = "test1";

// Function prototypes
void update_input_values();
void update_turbo_state();
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
  update_turbo_state();
  send_motor_and_turbo_values();
}

// A version of "map()" for floats.
float mapf(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float remap_deadzone(float value, float deadzone){
  if(value >  deadzone) return mapf(value,  deadzone, 1.0f, 0.0f, 1.0f);
  if(value < -deadzone) return mapf(value, -1.0f, -deadzone, -1.0f, 0.0f);

  return 0.0f;
}

// Maps the first range to the second range, with an added deadzone from -deadzone to +deadzone.
float normalize_plus_deadzone(float value, float from_lower, float from_upper, float to_lower, float to_upper, float deadzone){
  return remap_deadzone(mapf(value, from_lower, from_upper, to_lower, to_upper), deadzone);
}

void update_input_values(){
  int rawJoyX = analogRead(PIN_JOY_X);
  int rawJoyY = analogRead(PIN_JOY_Y);
  int rawAccl = analogRead(PIN_ACCEL);

  joyX = normalize_plus_deadzone(rawJoyX, 0, 1023, -1, 1, 0.1f);
  joyY = normalize_plus_deadzone(rawJoyY, 0, 1023, -1, 1, 0.1f);
  accl = normalize_plus_deadzone(rawAccl, 0, 1023, 0, 1, 0.1f);
  turbo_btn_state = digitalRead(PIN_TURBO) == LOW;
}

// Updates the turbo state. Depends on the current state, how long the turbo has been in the state, and if the turbo button is being pressed.
void update_turbo_state(){
  switch(turbo_state){
    case idle:
      if(turbo_btn_state){
        turbo_state = active;
        turbo_active_start = millis();
      }
      break;

    case active:
      if(millis() - turbo_active_start > TURBO_ACTIVE_TIME){
        turbo_state = recharging;
        turbo_recharge_start = millis();
      }
      break;
    
    case recharging:
      if(millis() - turbo_recharge_start > TURBO_RECHARGE_TIME){
        turbo_state = idle;
      }
      break;
  }
}

void send_motor_and_turbo_values(){
  float left = constrain( joyX + joyY, -1, 1) * accl;
  float right = constrain(-joyX + joyY, -1, 1) * accl;

  // Create a union of a struct containing the data we want to send, and an array of the same length, allowing the data to be reinterpreted directly as bytes.
  union {
    struct {
      float left;
      float right;
      bool turbo;
    } data;
    byte bytes[sizeof(data)];
  } u;
  
  u.data.left = left;
  u.data.right = right;
  u.data.turbo = (turbo_state == active);

  radio.write(u.bytes, sizeof(u.bytes));
}