//Receiver.ino
//#include <SPI.h>
#include <RF24.h>

#define IRQ_PIN		3
#define CE_PIN		8
#define CSN_PIN		9
#define VBAT_PIN	A7
#define LED_GREEN_PIN	2

#define LED_BLINK_HALF_PERIOD_MS	350
#define CONNECTION_TIMEOUT_MS		200

#define MASK_DIR_RIGHT	0b00000001
#define MASK_DIR_LEFT	0b00000010
#define MASK_TURBO		0b00000100
#define SHIFT_DIR_RIGHT	0
#define SHIFT_DIR_LEFT	1
#define SHIFT_TURBO		2

typedef struct {
	uint8_t	right_speed;
	uint8_t	left_speed;
	uint8_t	bits_params;
} packet_t;

// Instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

uint8_t address_transmitter[6] = "Rem00";
uint8_t address_receiver[6] = "Rob00";
uint8_t selected_channel;	// From 0 to 16


volatile uint8_t right_speed = 0;		// 0->255 <-> 0%->100%
volatile uint8_t left_speed = 0;		// 0->255 <-> 0%->100%
volatile bool right_dir_forward = true;	// true = forward; false = reverse
volatile bool left_dir_forward = true;	// true = forward; false = reverse
volatile bool turbo_ON = false;
uint32_t turbo_start_ms;

volatile bool battery_low = false;
float battery_low_threshold = 6.4;	// Battery cell is too low at 3.2V; 2S (2 cells) battery -> 6.4V

volatile uint32_t last_packet_received;
volatile bool connected = false;
bool led_state = false;
uint32_t last_led_blink_ms;

uint8_t get_dip_switches_selection();
float get_battery_voltage();
uint8_t calc_pwm(uint8_t pwm_in);
void update_motors(uint8_t right_pwm, uint8_t left_pwm);
void interruptHandler();

void setup() {
	Serial.begin(115200);

	// Initialize the transceiver on the SPI bus
	if (!radio.begin()) {
		Serial.println(F("Radio hardware is not responding!"));
		while (1) {} // Hold in infinite loop; Maybe put a red led blinking
	}

	pinMode(LED_GREEN_PIN, OUTPUT);

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
			digitalWrite(LED_GREEN_PIN, led_state ^= 1);
		}
	}
	else if (millis() - last_packet_received > CONNECTION_TIMEOUT_MS) {
		connected = false;
		update_motors(0, 0);
	}

	// 3) Check turbo ON time
	// if tubo ON time > threshold: turbo = false

	/* SERIAL DEBUG */
	static uint32_t last_print_ms;
	if (millis() - last_print_ms > 100) {
		last_print_ms = millis();

		if (!right_dir_forward) Serial.write('-');
		Serial.println(right_speed);
		if (!left_dir_forward) Serial.write('-');
		Serial.println(left_speed);
		Serial.println(turbo_ON ? "Turbo ON !": "Turbo OFF");
	}
	/* END SERIAL DEBUG */
}


uint8_t get_dip_switches_selection() {
	// 4 switches -> 2^4 = 16 combinations -> 0 -> 15
	// TODO
	return 12;
}

float get_battery_voltage() {
	// Voltage dividor: 20kohm & 10kohm => volatge divided by 3
	// raw_value: 0 -> 1023
	// measured_voltage = raw_value / 1023 * 5: 0 -> 5V
	// real_volatge = measured_voltage * 3: 0 -> 15V
	uint16_t raw_value = analogRead(VBAT_PIN);
	return (float) raw_value * 3.0 * 5.0 / 1023.0;
}

uint8_t calc_pwm(uint8_t pwm_in) {
	// Return PWM ajusted to the battery voltage
	// - nominal motor voltage is 6V
	// - fully charged battery gives 8.4V

	// Maybe adjust the fact that the final speed of a motor is not linear (proportional) to its PWM
	// Or later

	return pwm_in;
}

void update_motors(uint8_t right_pwm, uint8_t left_pwm) {
	// Apply pwm and direction on both L293D

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
		turbo_ON = (rcv_data.bits_params & MASK_TURBO) >> SHIFT_TURBO;

		// Check battery low
		// if (! battery_low)
		update_motors(calc_pwm(right_speed), calc_pwm(left_speed));

		last_packet_received = millis();
		connected = true;
		digitalWrite(LED_GREEN_PIN, HIGH);
	}

	return;
}
