//Transmitter.ino
//#include <SPI.h>
#include <RF24.h>

#define IRQ_PIN		2
#define CE_PIN		A0
#define CSN_PIN		10

#define LED_CONNECTION_PIN	3
#define LED_TURBO_PIN 0
#define LED_POWER_PIN A5

#define RADIO_UPDATE_FREQUENCY_HZ	100
#define RADIO_UPDATE_PERIOD_US		1000000 / RADIO_UPDATE_FREQUENCY_HZ
#define LED_BLINK_HALF_PERIOD_MS	350
#define TURBO_LED_BLINK_HALF_PERIOD_MS 100

#define TURBO_ACTIVE_DURATION_MS   3000
#define TURBO_RECHARGE_DURATION_MS 3000

#define MASK_DIR_RIGHT	0b00000001
#define MASK_DIR_LEFT	  0b00000010
#define MASK_TURBO		  0b00000100

#define DIP_PIN_1 9
#define DIP_PIN_2 8
#define DIP_PIN_3 7
#define DIP_PIN_4 6

#define JOY_X_PIN A3
#define JOY_Y_PIN A2
#define ACCEL_PIN A4
#define TURBO_PIN 4
#define TURBO_PIN_2 5

typedef struct {
	uint8_t	right_speed;
	uint8_t	left_speed;
	uint8_t	bits_params;
} packet_t;

enum TurboState {
	disabled,
	active,
	recharging
};


// Instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

uint8_t address_transmitter[6] = "Rem00";
uint8_t address_receiver[6] = "Rob00";
uint8_t selected_channel;	// From 0 to 16

bool connected = false;
bool led_state = false;
uint32_t last_led_blink_ms;

float right_speed;
float left_speed;
TurboState turbo_state = disabled;
uint32_t turbo_active_start_ms;
uint32_t turbo_recharge_start_ms;

volatile uint32_t last_radio_update_us;

uint8_t get_dip_switches_selection();
void interruptHandler();

void setup() {
	Serial.begin(115200);

	// Initialize the transceiver on the SPI bus
	if (!radio.begin()) {
		Serial.println(F("Radio hardware is not responding!"));
		while (1) {} // Hold in infinite loop; Maybe put a red led blinking
	}
  
	pinMode(LED_CONNECTION_PIN, OUTPUT);
	pinMode(LED_TURBO_PIN, OUTPUT);
	pinMode(LED_POWER_PIN, OUTPUT);

	digitalWrite(LED_POWER_PIN, HIGH);

  // Initialize the DIP switch pins
  pinMode(DIP_PIN_1, INPUT_PULLUP);
  pinMode(DIP_PIN_2, INPUT_PULLUP);
  pinMode(DIP_PIN_3, INPUT_PULLUP);
  pinMode(DIP_PIN_4, INPUT_PULLUP);

	// Initialize the turbo button
	// Here we only use one of them, but we could require the user to press both to activate the turbo.
	pinMode(TURBO_PIN, INPUT_PULLUP);
	pinMode(TURBO_PIN_2, INPUT_PULLUP);

	// Configure interrupt
	// pinMode(IRQ_PIN, INPUT);
	// attachInterrupt(digitalPinToInterrupt(IRQ_PIN), interruptHandler, FALLING);

	// Get selected channel from the dip switches
	selected_channel = get_dip_switches_selection();

	// 126 1MHz radio channels available: from 0 to 126 == 2400MHz to 2526MHz
	radio.setChannel(80 + 3 * selected_channel);	// Range: 2480;2483;2486;2489;2492;...;2525 MHz
	radio.setPALevel(RF24_PA_HIGH);
	radio.setDataRate(RF24_250KBPS);
	radio.setPayloadSize(sizeof(packet_t));

	// Radio address = "Rem"+str(selected_channel) and "Rob"+str(selected_channel) (ex: "Rem07")
	address_transmitter[4] = address_receiver[4] = '0' + (selected_channel / 10) % 10;		// Tens digit
	address_transmitter[5] = address_receiver[5] = '0' + selected_channel % 10;			// Unit digit

	// Open writing pipe on the receiver address
	radio.openWritingPipe(address_receiver);	// always uses pipe 0

	// Open reading pipe on the transmitter address
	radio.openReadingPipe(1, address_transmitter);	// using pipe 1
}

void loop() {
	// 1) Read inputs
	/* SERIAL DEBUG */
	// if (Serial.available()) {
	// 	uint8_t cmd = Serial.read();
	// 	switch (cmd) {
	// 		case 'T':
	// 			turbo_ON ^= 1;
	// 			Serial.println(turbo_ON ? "Turbo ON !": "Turbo OFF");
	// 			break;
	// 		case 'R':
	// 			right_speed = Serial.parseFloat();
	// 			Serial.println(right_speed);
	// 			break;
	// 		case 'L':
	// 			left_speed = Serial.parseFloat();
	// 			Serial.println(left_speed);
	// 			break;
	// 		default:
	// 			// Fault: empty Rx buffer
	// 			while(Serial.available()) Serial.read();
	// 	}
	// }
	/* END SERIAL DEBUG */

  /* START READ JOYSTICKS */	
  int rawJoyX = analogRead(JOY_X_PIN);
  int rawJoyY = analogRead(JOY_Y_PIN);
  int rawAccl = analogRead(ACCEL_PIN);

  float joyX_unnormalized = remap_with_deadzone(rawJoyX, 1023, 0, -1, 1, 0.1f);
  float joyY_unnormalized = remap_with_deadzone(rawJoyY, 0, 1023, -1, 1, 0.1f);

  float joy_modulus = sqrtf(joyX_unnormalized * joyX_unnormalized + joyY_unnormalized * joyY_unnormalized);

  float joyX = joyX_unnormalized / joy_modulus;
  float joyY = joyY_unnormalized / joy_modulus;
	
  float accl = remap_with_deadzone(rawAccl, 1023, 0, 0, 1, 0.1f);
  bool turbo_btn_state = !digitalRead(TURBO_PIN) or !digitalRead(TURBO_PIN_2);

  left_speed  = constrain( joyX + joyY, -1, 1) * accl;
  right_speed = constrain(-joyX + joyY, -1, 1) * accl;
  /* END READ JOYSTICKS */

	// 2) Blink the connection status LED (green) if not connected
	if (!connected && millis() - last_led_blink_ms > LED_BLINK_HALF_PERIOD_MS) {
		last_led_blink_ms = millis();
		digitalWrite(LED_CONNECTION_PIN, led_state ^= 1);
	}

	if(connected) digitalWrite(LED_CONNECTION_PIN, HIGH); // Not efficient

	// 3) Determine if the turbo should be active, recharging or disabled
	switch (turbo_state)
	{
	case disabled:
		digitalWrite(LED_TURBO_PIN, HIGH);


		if(turbo_btn_state){
			turbo_state = active;
			turbo_active_start_ms = millis();
		}
		break;
	case active:
		digitalWrite(LED_TURBO_PIN, (millis() - turbo_recharge_start_ms) % (2*TURBO_LED_BLINK_HALF_PERIOD_MS) > TURBO_LED_BLINK_HALF_PERIOD_MS);

		if(millis() - turbo_active_start_ms > TURBO_ACTIVE_DURATION_MS){
			turbo_state = recharging;
			turbo_recharge_start_ms = millis(); // We could also do turbo_active_start_ms + TURBO_ACTIVE_DURATION
		}
		break;
	case recharging:
		digitalWrite(LED_TURBO_PIN, LOW);
		if(millis() - turbo_recharge_start_ms > TURBO_RECHARGE_DURATION_MS){
			turbo_state = disabled;
		}
		break;
	default:
		break;
	}

	// 4) Periodically send the speeds' informations through radio
	if (micros() - last_radio_update_us > RADIO_UPDATE_PERIOD_US) {
		last_radio_update_us = micros();

		packet_t spd_data = {0, 0, 0};

		if (right_speed >= 0)
			spd_data.bits_params |= MASK_DIR_RIGHT;
		if (left_speed >= 0)
			spd_data.bits_params |= MASK_DIR_LEFT;
		if (turbo_state == active)
			spd_data.bits_params |= MASK_TURBO;

		spd_data.right_speed = abs(right_speed) * 255;
		spd_data.left_speed = abs(left_speed) * 255;

		connected = radio.write(&spd_data, sizeof(packet_t));
	}
}

uint8_t get_dip_switches_selection() {
	uint8_t dip_value = 0;
  dip_value |= digitalRead(DIP_PIN_1) << 0;
  dip_value |= digitalRead(DIP_PIN_2) << 1;
  dip_value |= digitalRead(DIP_PIN_3) << 2;
  dip_value |= digitalRead(DIP_PIN_4) << 3;
  Serial.println(dip_value);
	return dip_value;
}

// void interruptHandler() {
// 	bool tx_ds, tx_df, rx_dr;					// declare variables for IRQ masks
// 	radio.whatHappened(tx_ds, tx_df, rx_dr);	// get values for IRQ masks

// 	if (tx_ds) {
// 		connected = true;
// 		// digitalWrite(LED_CONNECTION_PIN, HIGH);
// 	}
// 	if (tx_df)
// 		connected = false;
// }

// A version of "map()" for floats.
float mapf(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Maps the first range to the second range, with an added deadzone from -deadzone to +deadzone.
float remap_with_deadzone(float value, float from_lower, float from_upper, float to_lower, float to_upper, float deadzone){

  // Remap the the value from the first range to the second range.
  float remapped_value = mapf(value, from_lower, from_upper, to_lower, to_upper);

  // Remap the ranges on either side of the deadzone to start from 0 on the edge of the deadzone.
  if(remapped_value >  deadzone) return mapf(remapped_value,  deadzone, 1.0f, 0.0f, 1.0f);
  if(remapped_value < -deadzone) return mapf(remapped_value, -1.0f, -deadzone, -1.0f, 0.0f);

  // If the value is within the deadzone, return 0
  return 0.0f;
}