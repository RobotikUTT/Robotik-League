//Transmitter.ino
//#include <SPI.h>
#include <RF24.h>

#define IRQ_PIN		3
#define CE_PIN		8
#define CSN_PIN		9

#define LED_GREEN_PIN	2

#define RADIO_UPDATE_FREQUENCY_HZ	100
#define RADIO_UPDATE_PERIOD_US		1000000 / RADIO_UPDATE_FREQUENCY_HZ
#define LED_BLINK_HALF_PERIOD_MS	350

#define MASK_DIR_RIGHT	0b00000001
#define MASK_DIR_LEFT	0b00000010
#define MASK_TURBO		0b00000100

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

volatile bool connected = false;
bool led_state = false;
uint32_t last_led_blink_ms;

float right_speed;
float left_speed;
bool turbo_ON;

uint32_t last_radio_update_us;

uint8_t get_dip_switches_selection();
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

	// Open writing pipe on the receiver address
	radio.openWritingPipe(address_receiver);	// always uses pipe 0

	// Open reading pipe on the transmitter address
	radio.openReadingPipe(1, address_transmitter);	// using pipe 1
}

void loop() {
	// 1) Read inputs
	/* SERIAL DEBUG */
	if (Serial.available()) {
		uint8_t cmd = Serial.read();
		switch (cmd) {
			case 'T':
				turbo_ON ^= 1;
				Serial.println(turbo_ON ? "Turbo ON !": "Turbo OFF");
				break;
			case 'R':
				right_speed = Serial.parseFloat();
				Serial.println(right_speed);
				break;
			case 'L':
				left_speed = Serial.parseFloat();
				Serial.println(left_speed);
				break;
			default:
				// Fault: empty Rx buffer
				while(Serial.available()) Serial.read();
		}
	}
	/* END SERIAL DEBUG */

	// 2) Blink the connection status LED (green) if not connected
	if (!connected && millis() - last_led_blink_ms > LED_BLINK_HALF_PERIOD_MS) {
		last_led_blink_ms = millis();
		digitalWrite(LED_GREEN_PIN, led_state ^= 1);
	}

	// 3) Periodically send the speeds' informations through radio
	if (micros() - last_radio_update_us > RADIO_UPDATE_PERIOD_US) {
		last_radio_update_us = micros();

		packet_t spd_data = {0, 0, 0};

		if (right_speed >= 0)
			spd_data.bits_params |= MASK_DIR_RIGHT;
		if (left_speed >= 0)
			spd_data.bits_params |= MASK_DIR_LEFT;
		if (turbo_ON)
			spd_data.bits_params |= MASK_TURBO;

		spd_data.right_speed = abs(right_speed) * 255;
		spd_data.left_speed = abs(left_speed) * 255;

		radio.write(&spd_data, sizeof(packet_t));
	}
}

uint8_t get_dip_switches_selection() {
	// 4 switches -> 2^4 = 16 combinations -> 0 -> 15
	// TODO
	return 12;
}

void interruptHandler() {
	bool tx_ds, tx_df, rx_dr;					// declare variables for IRQ masks
	radio.whatHappened(tx_ds, tx_df, rx_dr);	// get values for IRQ masks

	if (tx_ds) {
		connected = true;
		digitalWrite(LED_GREEN_PIN, HIGH);
	}
	if (tx_df)
		connected = false;
}
