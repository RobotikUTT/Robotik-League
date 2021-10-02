#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define LEFT_PWM_PIN      3
#define LEFT_FORWARD_PIN  A4
#define LEFT_BACKWARD_PIN A5

#define RIGHT_PWM_PIN      6
#define RIGHT_FORWARD_PIN  5
#define RIGHT_BACKWARD_PIN 4

// nRF24L01 pins
#define NRF_CE  8
#define NRF_CSN 10

#define BAT_VOLT_PIN A6

// A struct containing all the data we want to send to the robot.
typedef struct {
  float left;
  float right;
  bool turbo;
} RCData;

RF24 radio(NRF_CE, NRF_CSN); // CE, CSN

RCData rc_data;

const byte address[6] = "test1";

byte buffer[32];

float bat_volt;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);

  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  digitalWrite(LEFT_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_FORWARD_PIN, HIGH);
}

void loop() {
  read_battery_voltage();
  update_rc_data();
  update_motor_outputs();
  delay(10);
}

void update_rc_data(){
  union {
    RCData data;
    byte bytes[32];
  } u;

  if (radio.available()) {
    radio.read(u.bytes, sizeof(u.bytes));
    
    rc_data = u.data;
  }
}

// A version of "map()" for floats.
float mapf(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void update_motor_outputs(){

  float output_coef = 6/bat_volt;
  if(output_coef > 1) output_coef = 1.0f;


  digitalWrite(LEFT_BACKWARD_PIN, rc_data.left < 0);
  digitalWrite(LEFT_FORWARD_PIN, rc_data.left > 0);

  digitalWrite(RIGHT_BACKWARD_PIN, rc_data.right < 0);
  digitalWrite(RIGHT_FORWARD_PIN, rc_data.right > 0);

  analogWrite(RIGHT_PWM_PIN, abs(rc_data.right * 255 * output_coef));
  analogWrite(LEFT_PWM_PIN, abs(rc_data.left * 255 * output_coef));

  Serial.print("left: ");
  Serial.print(rc_data.left);
  Serial.print("; right: ");
  Serial.println(rc_data.right);
}

void read_battery_voltage(){
  int bat_pin_in = analogRead(BAT_VOLT_PIN);
  bat_volt = mapf(bat_pin_in, 0.0f, 1023.0f, 0.0f, 15.0f);
}