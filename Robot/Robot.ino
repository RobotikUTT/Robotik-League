//Receiver.ino
//#include <SPI.h>
#include <RF24.h>

#define IRQ_PIN		2
#define CE_PIN		8
#define CSN_PIN		10

#define VBAT_PIN	A6
#define LED_CONNECTION_PIN	7
#define LED_TURBO_PIN   9

#define EN_G 3
#define EN_D 6
#define IN1  5
#define IN2  4
#define IN3  A4
#define IN4  A5

#define LED_BLINK_HALF_PERIOD_MS	350
#define CONNECTION_TIMEOUT_MS		  200
#define NOMINAL_MOTOR_V 6

#define TURBO_LED_BLINK_HALF_PERIOD_MS 100

#define TURBO_ACTIVE_DURATION_MS   2500 // Must be longer than the duration on the remote.
#define TURBO_RECHARGE_DURATION_MS 2500 // Must be shorter than the duration on the remote.

#define MASK_DIR_RIGHT	0b00000001
#define MASK_DIR_LEFT	  0b00000010
#define MASK_TURBO		  0b00000100
#define SHIFT_DIR_RIGHT	0
#define SHIFT_DIR_LEFT	1
#define SHIFT_TURBO		2

#define DIP_PIN_1 A3
#define DIP_PIN_2 A2
#define DIP_PIN_3 A1
#define DIP_PIN_4 A0

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


volatile uint8_t right_speed = 0;		// 0->255 <-> 0%->100%
volatile uint8_t left_speed = 0;		// 0->255 <-> 0%->100%
volatile bool right_dir_forward = true;	// true = forward; false = reverse
volatile bool left_dir_forward = true;	// true = forward; false = reverse

TurboState turbo_state = disabled;
bool turbo_btn_state = false;

uint32_t turbo_active_start_ms;
uint32_t turbo_recharge_start_ms;

volatile uint8_t current_pwm_right = 0;
volatile uint8_t current_pwm_left = 0;

volatile bool battery_low = false;
float battery_low_threshold = 6.4;	// Battery cell is too low at 3.2V; 2S (2 cells) battery -> 6.4V

volatile uint32_t last_packet_received;
volatile bool connected = false;
bool led_state = false;
uint32_t last_led_blink_ms;

uint8_t get_dip_switches_selection();
float get_battery_voltage();
uint8_t calc_pwm(uint8_t pwm_in, bool is_turning);
void update_motors(uint8_t right_pwm, uint8_t left_pwm, bool right_forward, bool left_forward);
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
	
	pinMode(EN_G, OUTPUT);
	pinMode(EN_D, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

	// Initialize the DIP switch pins
  pinMode(DIP_PIN_1, INPUT_PULLUP);
  pinMode(DIP_PIN_2, INPUT_PULLUP);
  pinMode(DIP_PIN_3, INPUT_PULLUP);
  pinMode(DIP_PIN_4, INPUT_PULLUP);

	// Configure interrupt
	pinMode(IRQ_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(IRQ_PIN), interruptHandler, FALLING);

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

	// Open writing pipe on the transmitter address
	radio.openWritingPipe(address_transmitter);	// always uses pipe 0

	// Open reading pipe on the receiver address
	radio.openReadingPipe(1, address_receiver);	// using pipe 1

	radio.startListening();
}

void loop() {
	// 1) Check voltage (if below a threshold (like 6.4V), put battery_low to true and stop the motors)


	// 2) Check connection timeout and blink the connection status LED (green) if not connected
	if (!connected) {
		if (millis() - last_led_blink_ms > LED_BLINK_HALF_PERIOD_MS) {
			last_led_blink_ms = millis();
			digitalWrite(LED_CONNECTION_PIN, led_state ^= 1);
		}
	}
	else if (millis() - last_packet_received > CONNECTION_TIMEOUT_MS) {
		connected = false;
		update_motors(0, 0, true, true);
	}

	// 3) Check turbo ON time and update the turbo LED
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

		// If the turbo has been active long enough, or the remote is no longer requesing turbo
		if(millis() > turbo_active_start_ms + TURBO_ACTIVE_DURATION_MS || !turbo_btn_state){
			turbo_state = recharging;
			turbo_recharge_start_ms = millis();
		}
		break;

	case recharging:
		// analogWrite(LED_TURBO_PIN, (millis()-turbo_recharge_start_ms) * 255 / TURBO_RECHARGE_DURATION_MS);
		digitalWrite(LED_TURBO_PIN, LOW);
		if(millis() > turbo_recharge_start_ms +  TURBO_RECHARGE_DURATION_MS){
			turbo_state = disabled;
		}
		break;
	}

	/* SERIAL DEBUG */
	static uint32_t last_print_ms;
	if (millis() - last_print_ms > 100) {
		last_print_ms = millis();

		if (!right_dir_forward) Serial.write('-');
		Serial.println(current_pwm_left);
		if (!left_dir_forward) Serial.write('-');
		Serial.println(current_pwm_right);
		Serial.println(turbo_state == active ? "Turbo ON !": "Turbo OFF");
	}
	/* END SERIAL DEBUG */
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

float get_battery_voltage() {
	// Voltage dividor: 20kohm & 10kohm => volatge divided by 3
	// raw_value: 0 -> 1023
	// measured_voltage = raw_value / 1023 * 5: 0 -> 5V
	// real_volatge = measured_voltage * 3: 0 -> 15V
	uint16_t raw_value = analogRead(VBAT_PIN);
	return (float) raw_value * 3.0 * 5.0 / 1023.0;
}

uint8_t calc_pwm(uint8_t pwm_in, bool is_turning) {
	// Return PWM ajusted to the battery voltage
	// - nominal motor voltage is 6V
	// - fully charged battery gives 8.4V
	
	// if(is_turning) return pwm_in;
	if(turbo_state == active) return pwm_in;

	float bat_volt = get_battery_voltage();
	float pwm_coef = constrain(NOMINAL_MOTOR_V / bat_volt, 0, 1);
	return pwm_in * pwm_coef;

	// Maybe adjust the fact that the final speed of a motor is not linear (proportional) to its PWM
	// Or later
}

void update_motors(uint8_t right_pwm, uint8_t left_pwm, bool right_forward, bool left_forward) {
	// Apply pwm and direction on both L293D
	if (right_pwm != current_pwm_right) {
		current_pwm_right = right_pwm;
		analogWrite(EN_D, right_pwm);
		digitalWrite(IN1, right_forward);
		digitalWrite(IN2, !right_forward);
	}
	if (left_pwm != current_pwm_left) {
		current_pwm_left = left_pwm;
		analogWrite(EN_G, left_pwm);
		digitalWrite(IN3, !left_forward);
		digitalWrite(IN4, left_forward);
	}

	return;
}

void interruptHandler() {
	bool tx_ds, tx_df, rx_dr;					// declare variables for IRQ masks
	radio.whatHappened(tx_ds, tx_df, rx_dr);	// get values for IRQ masks
	// whatHappened() clears the IRQ masks also. This is required for
	// continued TX operations when a transmission fails.
	// clearing the IRQ masks resets the IRQ pin to its inactive state (HIGH)

	if (rx_dr) {
		packet_t rcv_data;
		while (radio.available()) {
			radio.read(&rcv_data, sizeof(packet_t));
		}

		right_speed = rcv_data.right_speed;
		left_speed = rcv_data.left_speed;
		right_dir_forward = (rcv_data.bits_params & MASK_DIR_RIGHT) >> SHIFT_DIR_RIGHT;
		left_dir_forward = (rcv_data.bits_params & MASK_DIR_LEFT) >> SHIFT_DIR_LEFT;
		turbo_btn_state = (rcv_data.bits_params & MASK_TURBO) >> SHIFT_TURBO;

		// Check battery low
		// if (! battery_low)
		bool is_turning = (right_dir_forward != left_dir_forward);
		update_motors(calc_pwm(right_speed, is_turning), calc_pwm(left_speed, is_turning), right_dir_forward, left_dir_forward);

		last_packet_received = millis();
		connected = true;
		digitalWrite(LED_CONNECTION_PIN, HIGH);
	}

	return;
}
