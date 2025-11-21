/** m16946 - ATSAMR34 LoRa P2P Simple Application -
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * This is a bare minimum user application template.
 *
 * For documentation of the board, go \ref group_common_boards "here" for a link
 * to the board-specific documentation.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# Basic usage of on-board LED and button
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include "sio2host.h"
#include "stdio_serial.h"
#include "radio_interface.h"
#include "radio_driver_hal.h"
#include "aes_engine.h"
#include "sw_timer.h"
#include "system_init.h"
#include "lorawan.h"
#include "display.h"
#include <ctype.h>

// Trim trailing \r or \n in-place (returns new length)
static uint8_t trim_crlf(uint8_t *p, uint8_t len)
{
	while (len && (p[len - 1] == '\r' || p[len - 1] == '\n')) len--;
	return len;
}

// Quick ASCII print (good for NMEA which has no embedded NUL)
static void print_payload_ascii(const uint8_t *p, uint8_t len)
{
	const size_t MAX = 256;
	char buf[MAX + 1];
	if (len > MAX) len = MAX;
	memcpy(buf, p, len);
	buf[len] = '\0';
	printf("ASCII: %s\r\n", buf);
}

// Safer version (shows non-printables as <XX>)
static void print_payload_ascii_safe(const uint8_t *p, uint8_t len)
{
	printf("ASCII(safe): ");
	for (uint8_t i = 0; i < len; i++)
	{
		uint8_t c = p[i];
		if (c == '\r')       { printf("\\r"); }
		else if (c == '\n')  { printf("\\n"); }
		else if (isprint(c)) { putchar(c); }
		else                 { printf("<%02X>", c); }
	}
	printf("\r\n");
}


/************************** macro definition ***********************************/
#define APP_DEBOUNCE_TIME   50	// button debounce time in ms
#define BUFFER_SIZE			50

/************************** Global variables ***********************************/
uint8_t buttonPressed = 0 ;
uint8_t buttonCounter = 0 ;
uint8_t buffer[BUFFER_SIZE] ;
// Software timer id for periodic GPS-like messages
uint8_t gpsTimerId = SWTIMER_INVALID;
// Software timer id for beep GPIO action (3 second pulse)
uint8_t beepTimerId = SWTIMER_INVALID;
// Software timer id for buzzer/vibrate (pulsing PWM effect)
uint8_t buzzerTimerId = SWTIMER_INVALID;
uint8_t buzzerToggleCount = 0;  /* Track on/off cycles for buzzer */
// Software timer id for heartbeat prints
uint8_t heartbeatTimerId = SWTIMER_INVALID;

/****************************** PROTOTYPES *************************************/
static void init(void) ;
static void configure_led(void) ;
static void configure_extint(void) ;
static void configure_radio(void) ;
static void configure_eic_callback(void) ;
static void extint_callback(void) ;
void print_menu(void) ;
void serial_data_handler(void) ;
void radio_transmit_uplink(uint8_t *data, uint16_t len) ;
void radio_enter_receive_mode(void) ;
void radio_exit_receive_mode(void) ;
SYSTEM_TaskStatus_t APP_TaskHandler(void) ;
void appData_callback(void *appHandle, appCbParams_t *appdata) ;
void print_array(uint8_t *array, uint8_t length) ;
static void gps_timer_cb(void *param);
static void heartbeat_cb(void *param);
static void decode_frame(uint8_t *frame, uint8_t len);
static void beep_timer_cb(void *param);
static void buzzer_timer_cb(void *param);
static void configure_buzzer(void);

/****************************** FUNCTIONS *************************************/
static void init(void)
{
	system_init() ;
	delay_init() ;
	board_init() ;
	configure_led() ;
	sio2host_init() ;
	INTERRUPT_GlobalInterruptEnable() ;
	// LoRaWAN Stack driver init
	HAL_RadioInit() ;
	AESInit() ;
	SystemTimerInit() ;
	Stack_Init() ;
	LORAWAN_Init(appData_callback, NULL) ;

	/* Create and start periodic GPS-like timer (3 seconds) */
	if (SwTimerCreate(&gpsTimerId) == LORAWAN_SUCCESS)
	{
		SwTimerStart(gpsTimerId, MS_TO_US(3000), SW_TIMEOUT_RELATIVE, (void *)gps_timer_cb, NULL);
	}

	/* Create beep timer (will be started on beep command) */
	SwTimerCreate(&beepTimerId);
	/* Create buzzer timer (will be started on vibrate command) */
	SwTimerCreate(&buzzerTimerId);

	/* Create and start heartbeat timer (2 seconds) */
	SwTimerCreate(&heartbeatTimerId);
	SwTimerStart(heartbeatTimerId, MS_TO_US(2000), SW_TIMEOUT_RELATIVE, (void *)heartbeat_cb, NULL);

	/* Initialize display */
	//display_init();
}

void print_menu(void)
{
	printf("\r\n-- ATSAMR34 LoRa P2P Simple Application --\r\n") ;	
	printf("- Press SW0 button to transmit counter value [%d]\r\n", buttonCounter) ;
	printf("- Type any character to transmit over LoRa Radio\r\n") ;
}

int main (void)
{
	init() ;	
	configure_led() ;
	configure_buzzer() ;
	configure_extint() ;
	configure_eic_callback() ;
	
	configure_radio() ;
	radio_enter_receive_mode() ;
	
	print_menu() ;
	while (1) 
	{
		serial_data_handler() ;
		SYSTEM_RunTasks() ;
	}
}

static void configure_led(void)
{
	struct port_config pin_conf ;
	port_get_config_defaults(&pin_conf) ;
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT ;
	port_pin_set_config(LED_0_PIN, &pin_conf) ;
	port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE) ;
}

static void configure_buzzer(void)
{
	/* Configure PA16 as output for buzzer/vibrate control */
	struct port_config pin_conf ;
	port_get_config_defaults(&pin_conf) ;
	pin_conf.direction = PORT_PIN_DIR_OUTPUT ;
	port_pin_set_config(PIN_PA16, &pin_conf) ;
	port_pin_set_output_level(PIN_PA16, false) ;  /* Start LOW */
}

static void configure_extint(void)
{
	struct extint_chan_conf eint_chan_conf;
	extint_chan_get_config_defaults(&eint_chan_conf);
	eint_chan_conf.gpio_pin           = BUTTON_0_EIC_PIN;
	eint_chan_conf.gpio_pin_mux       = BUTTON_0_EIC_MUX;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_FALLING;
	eint_chan_conf.filter_input_signal = true;
	extint_chan_set_config(BUTTON_0_EIC_LINE, &eint_chan_conf);
}

static void configure_eic_callback(void)
{
	extint_register_callback(
		extint_callback,
		BUTTON_0_EIC_LINE,
		EXTINT_CALLBACK_TYPE_DETECT
	);
	extint_chan_enable_callback(BUTTON_0_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);
}

static void extint_callback(void)
{
	/* Read the button level */
	if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE)
	{
		/* Wait for button debounce time */
		delay_ms(APP_DEBOUNCE_TIME);
		/* Check whether button is in default state */
		while(port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE)
		{
			delay_ms(100);
		}
		buttonPressed = true;
		/* Post task to application handler on button press */
		SYSTEM_PostTask(APP_TASK_ID);
	}
}

SYSTEM_TaskStatus_t APP_TaskHandler(void)
{
	if (buttonPressed == true)
	{
		buttonPressed = false ;
		buttonCounter++ ;
		if (buttonCounter > 255) buttonCounter = 0 ;
		printf("Button pressed %d times\r\n", buttonCounter) ;

		// exit receive mode
		radio_exit_receive_mode() ;
		
		// prepare and transmit buttonCounter
		buffer[0] = buttonCounter ;
		radio_transmit_uplink(buffer, 1) ;
	}
	return SYSTEM_TASK_SUCCESS;
}

void serial_data_handler(void)
{
	int rxChar ;
	char serialData ;
	/* verify if there was any character received*/
	if((-1) != (rxChar = sio2host_getchar_nowait()))
	{
		serialData = (char)rxChar;
		if((serialData != '\r') && (serialData != '\n') && (serialData != '\b'))
		{
			printf("\r\n") ;
			// exit receive mode
			radio_exit_receive_mode() ;
			// prepare and transmit character received
			buffer[0] = serialData ;
			radio_transmit_uplink(buffer, 1) ;
		}
	}
}

/* Configure LoRa Radio for P2P */
static void configure_radio(void)
{
	// 1) reset & pause LoRaWAN, like you already did
	LORAWAN_Reset(ISM_EU868);
	uint32_t time_ms = LORAWAN_Pause();
	printf("MAC Pause %ld\r\n", time_ms);

	// 2) frequency 868.1 MHz
	uint32_t freq = 868100000UL;
	RADIO_SetAttr(CHANNEL_FREQUENCY, &freq);

	// 3) LoRa modulation params
	int16_t sf = SF_7;                  // was SF_12
	RADIO_SetAttr(SPREADING_FACTOR, &sf);

	RadioLoRaBandWidth_t bw = BW_125KHZ;
	RADIO_SetAttr(BANDWIDTH, &bw);

	RadioErrorCodingRate_t cr = CR_4_5;
	RADIO_SetAttr(ERROR_CODING_RATE, &cr);

	// 4) preamble 8 symbols
	uint16_t preamble = 8;
	RADIO_SetAttr(PREAMBLE_LEN, &preamble);

	// 5) enable CRC
	uint8_t crc_on = 1;
	RADIO_SetAttr(CRC_ON, &crc_on);

	// 6) sync word 0x34 (same as RN2483 default / LoRaWAN)
	uint8_t sw = 0x34;
	RADIO_SetAttr(LORA_SYNC_WORD, &sw);

	// 7) power
	int16_t outputPwr = 14;             // or 15
	RADIO_SetAttr(OUTPUT_POWER, &outputPwr);

	// 8) watchdog (as your code)
	uint32_t wdt = 60000;
	RADIO_SetAttr(WATCHDOG_TIMEOUT, &wdt);

	printf("Radio configured for P2P 868.1MHz SF7/BW125/CR4_5\r\n");
}


/* Transmit LoRa Radio Uplink */
void radio_transmit_uplink(uint8_t *data, uint16_t len)
{
	printf("[Transmit Uplink] ") ;
	print_array(data, len) ;
	RadioError_t radioStatus ;
	RadioTransmitParam_t RadioTransmitParam ;
	RadioTransmitParam.bufferLen = len ;
	RadioTransmitParam.bufferPtr = data ;
	radioStatus = RADIO_Transmit(&RadioTransmitParam) ;
	switch(radioStatus)
	{
		case ERR_NONE:
			printf("Radio Transmit Success \r\n") ;
			print_menu() ;
			break;
		case ERR_DATA_SIZE:
			//do nothing, status already set to invalid
			break;
		default:
			printf("Radio busy \r\n") ;
	}
}

/* LoRa Radio enter into Receive Mode */
void radio_enter_receive_mode(void)
{
	RadioReceiveParam_t radioReceiveParam ;
	uint32_t rxTimeout = 0 ;	// forever
	radioReceiveParam.action = RECEIVE_START ;
	radioReceiveParam.rxWindowSize = rxTimeout ;
	if (RADIO_Receive(&radioReceiveParam) == 0)
	{
		printf("Radio in Receive mode\r\n") ;
	}
}

/* LoRa Radio exit from Receive Mode */
void radio_exit_receive_mode(void)
{
	RadioReceiveParam_t radioReceiveParam ;
	radioReceiveParam.action = RECEIVE_STOP ;
	if (RADIO_Receive(&radioReceiveParam) == 0)
	{
		printf("Radio Exit Receive mode\r\n") ;
	}	
}

/* Uplink/Downlink Callback */
void appData_callback(void *appHandle, appCbParams_t *appdata)
{
	StackRetStatus_t status = LORAWAN_INVALID_REQUEST ;
	
	if (appdata->evt == LORAWAN_EVT_RX_DATA_AVAILABLE)
	{
		// Downlink Event
		status = appdata->param.rxData.status ;
		switch(status)
		{
			case LORAWAN_RADIO_SUCCESS:
			{
				uint8_t dataLength = appdata->param.rxData.dataLength ;
				uint8_t *pData = appdata->param.rxData.pData ;
				if((dataLength > 0U) && (NULL != pData))
				{
					int8_t rssi_value, snr_value ;
					RADIO_GetAttr(PACKET_RSSI_VALUE, &rssi_value) ;
					RADIO_GetAttr(PACKET_SNR, &snr_value) ;
					
					printf(">> Payload received: ") ;
					print_array(pData, dataLength) ;
					uint8_t n = trim_crlf(pData, dataLength);
					print_payload_ascii(pData, n);
					printf("RSSI Value: %d\r\n", rssi_value) ;
					printf("SNR Value: %d", snr_value) ;
					printf("\r\n*******************\r\n") ;
					
					LED_On(LED_0_PIN) ;
					delay_ms(50) ;
					LED_Off(LED_0_PIN) ;

					/* Decode the received frame if it looks like a command */
					decode_frame(pData, dataLength);

					radio_enter_receive_mode() ;
					print_menu() ;
				}
			}
			break ;
			case LORAWAN_RADIO_NO_DATA:
			{
				printf("\n\rRADIO_NO_DATA \n\r");
			}
			break;
			case LORAWAN_RADIO_DATA_SIZE:
				printf("\n\rRADIO_DATA_SIZE \n\r");
			break;
			case LORAWAN_RADIO_INVALID_REQ:
				printf("\n\rRADIO_INVALID_REQ \n\r");
			break;
			case LORAWAN_RADIO_BUSY:
				printf("\n\rRADIO_BUSY \n\r");
			break;
			case LORAWAN_RADIO_OUT_OF_RANGE:
				printf("\n\rRADIO_OUT_OF_RANGE \n\r");
			break;
			case LORAWAN_RADIO_UNSUPPORTED_ATTR:
				printf("\n\rRADIO_UNSUPPORTED_ATTR \n\r");
			break;
			case LORAWAN_RADIO_CHANNEL_BUSY:
				printf("\n\rRADIO_CHANNEL_BUSY \n\r");
			break;
			case LORAWAN_INVALID_PARAMETER:
				printf("\n\rINVALID_PARAMETER \n\r");
			break;
			default:
				printf("UNKNOWN ERROR %d\r\n", status) ;
			break ;
		}
	}
	else if(appdata->evt == LORAWAN_EVT_TRANSACTION_COMPLETE)
	{
		// Uplink Event
		switch(status = appdata->param.transCmpl.status)
		{
			case LORAWAN_SUCCESS:
			case LORAWAN_RADIO_SUCCESS:
				printf("Transmission success\r\n") ;
				radio_enter_receive_mode() ;
				break ;
			case LORAWAN_RADIO_NO_DATA:
				printf("\r\nRADIO_NO_DATA\r\n") ;
				break ;
			case LORAWAN_RADIO_BUSY:
				printf("\r\nRADIO_BUSY\r\n") ;
				break ;
			default:
				break ;
		}
	}
}

/*********************************************************************//*
 \brief      Function to Print array of characters
 \param[in]  *array  - Pointer of the array to be printed
 \param[in]   length - Length of the array
 ************************************************************************/
void print_array(uint8_t *array, uint8_t length)
{
	printf("0x") ;
	for (uint8_t i = 0; i < length; i++)
	{
		printf("%02x", *array) ;
		array++ ;
	}
	printf("\n\r") ;
}

/* Decode command frame: [Group][Remote ID][Dog ID][Action][Param]
   Actions: 0x01=beep, 0x02=vibrate, 0x03=mute, 0x04=volume setting
*/
static void decode_frame(uint8_t *frame, uint8_t len)
{
	if (len < 5)
	{
		printf("Frame too short, ignoring\r\n");
		return;
	}

	uint8_t group = frame[0];
	uint8_t remote_id = frame[1];
	uint8_t dog_id = frame[2];
	uint8_t action = frame[3];
	uint8_t param = frame[4];

	printf("Decoded frame: Group=%02x, Remote=%02x, Dog=%02x, Action=%02x, Param=%02x\r\n",
	       group, remote_id, dog_id, action, param);

	/* Execute action based on command byte */
	switch (action)
	{
		case 0x01:  /* BEEP */
			printf("Action: BEEP - GPIO HIGH for 3 seconds\r\n");
			/* Set GPIO high */
			port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
			/* Start timer to turn it off after 3 seconds */
			if (beepTimerId != SWTIMER_INVALID)
			{
				SwTimerStop(beepTimerId);
			}
			SwTimerStart(beepTimerId, MS_TO_US(3000), SW_TIMEOUT_RELATIVE, (void *)beep_timer_cb, NULL);
			break;

		case 0x02:  /* VIBRATE */
			printf("Action: VIBRATE (PA16 buzz) - Duration %d param\r\n", param);
			/* Set buzzer on and start pulsing timer (100ms pulses) */
			port_pin_set_output_level(PIN_PA16, true);
			buzzerToggleCount = 0;
			if (buzzerTimerId != SWTIMER_INVALID)
			{
				SwTimerStop(buzzerTimerId);
			}
			/* Start 100ms timer for buzzing effect */
			SwTimerStart(buzzerTimerId, MS_TO_US(100), SW_TIMEOUT_RELATIVE, (void *)buzzer_timer_cb, NULL);
			break;

		case 0x03:  /* MUTE */
			printf("Action: MUTE - Not implemented\r\n");
			break;

		case 0x04:  /* VOLUME SETTING */
			printf("Action: VOLUME (param=%02x) - Not implemented\r\n", param);
			break;

		default:
			printf("Unknown action: %02x\r\n", action);
			break;
	}
}

/* Timer callback to turn off beep GPIO after 3 seconds */
static void beep_timer_cb(void *param)
{
	printf("Beep timeout - GPIO LOW\r\n");
	port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);
}

/* Timer callback to pulse buzzer (vibrate effect) - toggles PA16 every 100ms for 3 seconds total (30 cycles) */
static void buzzer_timer_cb(void *param)
{
	buzzerToggleCount++;
	
	/* Toggle PA16 */
	port_pin_toggle_output_level(PIN_PA16);
	
	printf("Buzzer toggle %d\r\n", buzzerToggleCount);
	
	/* Continue buzzing for 30 cycles (3 seconds at 100ms intervals) */
	if (buzzerToggleCount < 30)
	{
		/* Restart timer */
		SwTimerStart(buzzerTimerId, MS_TO_US(100), SW_TIMEOUT_RELATIVE, (void *)buzzer_timer_cb, NULL);
	}
	else
	{
		/* Stop buzzing - ensure GPIO is LOW */
		port_pin_set_output_level(PIN_PA16, false);
		printf("Buzzer off\r\n");
	}
}

/* Heartbeat timer callback: print heartbeat every 2 seconds */
static void heartbeat_cb(void *param)
{
	(void)param;
	printf("HEARTBEAT\r\n");
	/* restart the heartbeat timer for periodic behavior */
	if (heartbeatTimerId != SWTIMER_INVALID)
	{
		(void)SwTimerStart(heartbeatTimerId, MS_TO_US(2000), SW_TIMEOUT_RELATIVE, (void *)heartbeat_cb, NULL);
	}
}

/* Periodic timer callback: send GPS message every 3s.
   Format: [0x01][0x01][0x01][4-byte latitude][4-byte longitude]
   Example: 0x010101491c9c3846001e843a
*/
static void gps_timer_cb(void *param)
{
	/* Example coordinates (replace with actual GPS sensor readings) */
	int32_t lat = 0x491c9c38;  /* Example latitude in 1e-7 degree units or custom format */
	int32_t lon = 0x001e843a;  /* Example longitude in 1e-7 degree units or custom format */

	/* Build binary frame: [Group=01][Remote=01][Dog=01][Lat(4 bytes)][Lon(4 bytes)] */
	buffer[0] = 0x01;  /* Group */
	buffer[1] = 0x01;  /* Remote ID */
	buffer[2] = 0x01;  /* Dog ID */
	
	/* Latitude (4 bytes, big-endian) */
	buffer[3] = (uint8_t)((lat >> 24) & 0xFF);
	buffer[4] = (uint8_t)((lat >> 16) & 0xFF);
	buffer[5] = (uint8_t)((lat >> 8) & 0xFF);
	buffer[6] = (uint8_t)(lat & 0xFF);
	
	/* Longitude (4 bytes, big-endian) */
	buffer[7] = (uint8_t)((lon >> 24) & 0xFF);
	buffer[8] = (uint8_t)((lon >> 16) & 0xFF);
	buffer[9] = (uint8_t)((lon >> 8) & 0xFF);
	buffer[10] = (uint8_t)(lon & 0xFF);

	/* Total payload: 11 bytes */
	uint16_t payload_len = 11;

	/* Transmit over LoRa */
	radio_exit_receive_mode();
	radio_transmit_uplink(buffer, payload_len);

	/* Display GPS info on LCD */
	//display_clear();
	//display_printf(0, 10, "LAT: 0x%08lx", (long)lat);
	//display_printf(0, 22, "LON: 0x%08lx", (long)lon);
	//display_printf(0, 34, "TX: 3sec timer");
	//display_send_buffer();

	/* Restart timer for periodic behavior */
	(void)SwTimerStart(gpsTimerId, MS_TO_US(3000), SW_TIMEOUT_RELATIVE, (void *)gps_timer_cb, NULL);
}
