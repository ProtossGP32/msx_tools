// --------------------------------------------------------------------
//	The MIT License (MIT)
//	
//	Copyright (c) 2021 HRA! (t.hara)
//	
//	Permission is hereby granted, free of charge, to any person obtaining a copy
//	of this software and associated documentation files (the "Software"), to deal
//	in the Software without restriction, including without limitation the rights
//	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//	copies of the Software, and to permit persons to whom the Software is
//	furnished to do so, subject to the following conditions:
//	
//	The above copyright notice and this permission notice shall be included in
//	all copies or substantial portions of the Software.
//	
//	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//	THE SOFTWARE.
// --------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "bsp/board.h"
#include "tusb.h"
#include "hardware/uart.h"

// --------------------------------------------------------------------
//	MSX �� ��A���A���A�E�AA�AB �̃{�^���ɑΉ����� GPIO�s���̊J�n�ԍ�
//
//	Start number of the GPIO pin corresponding to the Up, Down, Left, 
//	Right, A, and B buttons on the MSX.
//
#define MSX_BUTTON_PIN 2

// --------------------------------------------------------------------
//	MSX ���痈�� SEL�M���� GPIO�s���ԍ�
//
//	GPIO pin number of the SEL signal coming from the MSX
//
#define MSX_SEL_PIN 8

// --------------------------------------------------------------------
//	SEL�M���̘_��
//		0: ���_�� (MSX Joymega)
//		1: ���_�� (MegaDrive)
//
//	Logic of SEL signal
//		0: negative logic (MSX Joymega)
//		1: positive logic (MegaDrive)
//
#define MSX_SEL_LOGIC 0

// --------------------------------------------------------------------
//	X,Y�����x
//
//	X,Y axis sensitivity
//
#define	LEFT_THRESHOLD		-64
#define	RIGHT_THRESHOLD		64
#define	TOP_THRESHOLD		-64
#define	BOTTOM_THRESHOLD	64

#define	LEFT_THRESHOLD_JOYSTICK		(LEFT_THRESHOLD + 128)
#define	RIGHT_THRESHOLD_JOYSTICK	(RIGHT_THRESHOLD + 128)
#define	TOP_THRESHOLD_JOYSTICK		(TOP_THRESHOLD + 128)
#define	BOTTOM_THRESHOLD_JOYSTICK	(BOTTOM_THRESHOLD + 128)

// --------------------------------------------------------------------
//	BUTTON MAP
//
#define A_BUTTON			GAMEPAD_BUTTON_X
#define B_BUTTON			GAMEPAD_BUTTON_B
#define C_BUTTON			GAMEPAD_BUTTON_A
#define X_BUTTON			GAMEPAD_BUTTON_Y
#define Y_BUTTON			GAMEPAD_BUTTON_C
#define Z_BUTTON			GAMEPAD_BUTTON_Z
#define START_BUTTON		GAMEPAD_BUTTON_TR
#define MODE_BUTTON			GAMEPAD_BUTTON_TL

// --------------------------------------------------------------------
// GPIO BUTTON MAP
//
#define GPIO_UP_BUTTON		9
#define GPIO_DOWN_BUTTON 	10
#define GPIO_LEFT_BUTTON	11
#define GPIO_RIGHT_BUTTON	12
#define GPIO_A_BUTTON			13
#define GPIO_B_BUTTON			14
#define GPIO_C_BUTTON			15
#define GPIO_X_BUTTON			16
#define GPIO_Y_BUTTON			17
#define GPIO_Z_BUTTON			18
#define GPIO_START_BUTTON	19
#define GPIO_MODE_BUTTON	20

// TODO: Learn to use Bitwise operations to use a single uint32_t value for
// all GPIO state values at once instead of sequentially polling all of them
static uint32_t volatile gpio_buttons_state = 0x00;

static uint8_t volatile joymega_gpio_matrix[5] = {
	0x33, 0x3F, 0x03, 0x3F, 0x3F
};

// --------------------------------------------------------------------
#define DEBUG_UART_ON 0

// --------------------------------------------------------------------
#if MSX_SEL_LOGIC == 0
	#define MSX_SEL_H true
	#define MSX_SEL_L false
#else
	#define MSX_SEL_H false
	#define MSX_SEL_L true
#endif

// --------------------------------------------------------------------
typedef struct TU_ATTR_PACKED {
	uint8_t		x;			//< X position of the gamepad. LEFT:0 ... CENTER:128 ... RIGHT:255
	uint8_t		y;			//< Y position of the gamepad. TOP:0 .... CENTER:128 ... BOTTON:255
	uint32_t	buttons;	//< Buttons of the gamepad.
	int8_t		reserved1;
	int8_t		reserved2;
} my_hid_joystick_report_t;

// --------------------------------------------------------------------
static uint8_t volatile joymega_matrix[5] = {
	0x33, 0x3F, 0x03, 0x3F, 0x3F
};

// --------------------------------------------------------------------
#define MAX_REPORT  4

static uint8_t report_count[ CFG_TUH_HID ];
static tuh_hid_report_info_t report_info_arr[ CFG_TUH_HID ][ MAX_REPORT ];
// TODO: Analyse best values for each mode
static int process_mode = 2;	//	0: joypad_mode, 1: mouse_mode, 2: gpio_mode

// --------------------------------------------------------------------
//	Mouse information

//	USB�}�E�X���瑗���Ă���������W���邽�߂̕ϐ�
static volatile int16_t	mouse_delta_x = 0;
static volatile int16_t	mouse_delta_y = 0;
static volatile int		mouse_resolution = 0;
static int32_t	mouse_button = 0;

//	USB�}�E�X���瑗���Ă����������Ɂu���M�p�v�ɉ��H�����l���i�[����ϐ�
static volatile int32_t	mouse_current_data = 3;
static volatile bool mouse_consume_data = false;

//	����MSX�֑��M���Ă�����e��ێ�����ϐ�
static int32_t	mouse_sending_data = 0;

//	LED Pattern
static int led_state = 0;
static const int led_pattern[6][8] = {
	{ 1, 0, 0, 0, 0, 0, 0, 0 },			//	Joypad mode
	{ 1, 0, 1, 0, 0, 0, 0, 0 },			//	Mouse mode (normal)
	{ 1, 0, 1, 0, 1, 0, 0, 0 },			//	Mouse mode (V half)
	{ 1, 0, 1, 0, 0, 0, 1, 0 },			//	Mouse mode (normal2)
	{ 1, 1, 0, 1, 0, 1, 0, 0 },			//	Mouse mode (V half2)
	{ 1, 1, 1, 1, 1, 1, 0, 0 },			//	GPIO mode - No USB connected
};

// --------------------------------------------------------------------
static void initialization( void ) {
	uint8_t i;

	board_init();
	tusb_init();

	//	GPIO�̐M���̌�����ݒ�
	for( i = 0; i < 6; i++ ) {
		gpio_init( MSX_BUTTON_PIN + i );
		gpio_set_dir( MSX_BUTTON_PIN + i, GPIO_OUT );
	}
	gpio_init( MSX_SEL_PIN );
	gpio_set_dir( MSX_SEL_PIN, GPIO_IN );
	gpio_pull_up( MSX_SEL_PIN );

	// GPIO for native gamepad
	for( i = 0; i < 12; i++) {
		gpio_init( GPIO_UP_BUTTON + i );
		gpio_set_dir( GPIO_UP_BUTTON + i, GPIO_IN );
		gpio_pull_up( GPIO_UP_BUTTON + i);
	}

	// DEBUG: LED for gpio gamepad mode
	gpio_init(21);
	gpio_set_dir(21, GPIO_OUT);
	gpio_put(21, false);
	gpio_init(22);
	gpio_set_dir(22, GPIO_OUT);
	gpio_put(22, false);
}

// --------------------------------------------------------------------
static uint64_t inline my_get_us( void ) {
	return to_us_since_boot( get_absolute_time() );
}

// --------------------------------------------------------------------
static void joypad_mode( void ) {
	static const uint32_t mask = 0x3F << MSX_BUTTON_PIN;
	static const uint64_t sequence_trigger_us = 1100;		//	1100[usec]
	static const uint64_t sequence_finish_us = 1600;		//	1600[usec]
	static uint64_t start_time;

	//	state 0
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_L ) {
		gpio_put_masked( mask, (uint32_t)joymega_matrix[1] << MSX_BUTTON_PIN );
	}
	start_time = my_get_us();

	//	state 1
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_H ) {
		gpio_put_masked( mask, (uint32_t)joymega_matrix[0] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_trigger_us ) {
			break;
		}
	}
	if( (my_get_us() - start_time) > sequence_trigger_us ) {
		return;
	}

	//	state 2
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_L ) {
		gpio_put_masked( mask, (uint32_t)joymega_matrix[1] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_trigger_us ) {
			break;
		}
	}
	if( (my_get_us() - start_time) > sequence_trigger_us ) {
		return;
	}
	//	state 3
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_H ) {
		gpio_put_masked( mask, (uint32_t)joymega_matrix[0] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_trigger_us ) {
			break;
		}
	}
	if( (my_get_us() - start_time) > sequence_trigger_us ) {
		return;
	}
	//	state 4
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_L ) {
		gpio_put_masked( mask, (uint32_t)joymega_matrix[1] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_trigger_us ) {
			break;
		}
	}
	if( (my_get_us() - start_time) > sequence_trigger_us ) {
		return;
	}
	//	state 5
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_H ) {
		gpio_put_masked( mask, (uint32_t)joymega_matrix[2] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_finish_us ) {
			break;
		}
	}
	if( (my_get_us() - start_time) > sequence_finish_us ) {
		return;
	}
	//	state 6
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_L ) {
		gpio_put_masked( mask, (uint32_t)joymega_matrix[3] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_finish_us ) {
			break;
		}
	}
	if( (my_get_us() - start_time) > sequence_finish_us ) {
		return;
	}
	//	state 7
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_H ) {
		gpio_put_masked( mask, (uint32_t)joymega_matrix[4] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_finish_us ) {
			break;
		}
	}
}

// TODO: Unify joypad_mode and gpio_mode functions into a single one
// - They share the same logic except the states matrix
//   to mask and send to the MSX joystick port
static void gpio_mode( void ) {
	static const uint32_t mask = 0x3F << MSX_BUTTON_PIN;
	static const uint64_t sequence_trigger_us = 1100;		//	1100[usec]
	static const uint64_t sequence_finish_us = 1600;		//	1600[usec]
	static uint64_t start_time;

	//	state 0
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_L ) {
		gpio_put_masked( mask, (uint32_t)joymega_gpio_matrix[1] << MSX_BUTTON_PIN );
	}
	start_time = my_get_us();

	//	state 1
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_H ) {
		gpio_put_masked( mask, (uint32_t)joymega_gpio_matrix[0] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_trigger_us ) {
			break;
		}
	}
	if( (my_get_us() - start_time) > sequence_trigger_us ) {
		return;
	}

	//	state 2
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_L ) {
		gpio_put_masked( mask, (uint32_t)joymega_gpio_matrix[1] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_trigger_us ) {
			break;
		}
	}
	if( (my_get_us() - start_time) > sequence_trigger_us ) {
		return;
	}
	//	state 3
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_H ) {
		gpio_put_masked( mask, (uint32_t)joymega_gpio_matrix[0] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_trigger_us ) {
			break;
		}
	}
	if( (my_get_us() - start_time) > sequence_trigger_us ) {
		return;
	}
	//	state 4
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_L ) {
		gpio_put_masked( mask, (uint32_t)joymega_gpio_matrix[1] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_trigger_us ) {
			break;
		}
	}
	if( (my_get_us() - start_time) > sequence_trigger_us ) {
		return;
	}
	//	state 5
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_H ) {
		gpio_put_masked( mask, (uint32_t)joymega_gpio_matrix[2] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_finish_us ) {
			break;
		}
	}
	if( (my_get_us() - start_time) > sequence_finish_us ) {
		return;
	}
	//	state 6
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_L ) {
		gpio_put_masked( mask, (uint32_t)joymega_gpio_matrix[3] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_finish_us ) {
			break;
		}
	}
	if( (my_get_us() - start_time) > sequence_finish_us ) {
		return;
	}
	//	state 7
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_H ) {
		gpio_put_masked( mask, (uint32_t)joymega_gpio_matrix[4] << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > sequence_finish_us ) {
			break;
		}
	}
}

// --------------------------------------------------------------------
static uint32_t inline get_mouse_button_bits( void ) {
	return (mouse_sending_data & 3);
}

// --------------------------------------------------------------------
static uint32_t inline get_mouse_nibble_bits_1st( void ) {

	//	MSX�֑����Ă���Œ��ɍX�V����Ȃ��悤�ɃR�s�[����
	mouse_sending_data = mouse_current_data;

	//	1�ڂ̃f�[�^�͖��Ӗ� (�{�^���̂�)
	return get_mouse_button_bits();
}

// --------------------------------------------------------------------
static uint32_t inline get_mouse_nibble_bits_2nd( void ) {

	return get_mouse_button_bits() | ((mouse_sending_data >> 2) & 0x3C);
}

// --------------------------------------------------------------------
static uint32_t inline get_mouse_nibble_bits_3rd( void ) {

	return get_mouse_button_bits() | ((mouse_sending_data >> 6) & 0x3C);
}

// --------------------------------------------------------------------
static uint32_t inline get_mouse_nibble_bits_4th( void ) {

	return get_mouse_button_bits() | ((mouse_sending_data >> 10) & 0x3C);
}

// --------------------------------------------------------------------
static uint32_t inline get_mouse_nibble_bits_5th( void ) {

	return get_mouse_button_bits() | ((mouse_sending_data >> 14) & 0x3C);
}

// --------------------------------------------------------------------
static uint32_t inline get_mouse_nibble_bits_6th( void ) {

	return get_mouse_button_bits();
}

// --------------------------------------------------------------------
static void mouse_mode( void ) {
	static const uint32_t mask = 0x3F << MSX_BUTTON_PIN;
	static const uint64_t timeout_us = 500;		//	500[usec]
	static uint64_t start_time;

	//	idle
	gpio_put_masked( mask, get_mouse_nibble_bits_1st() << MSX_BUTTON_PIN );
	if( gpio_get( MSX_SEL_PIN ) == MSX_SEL_L ) {
		return;
	}

	mouse_consume_data = true;
	mouse_current_data &= 0xF;
	start_time = my_get_us();

	//	state 1
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_H ) {
		gpio_put_masked( mask, get_mouse_nibble_bits_2nd() << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > timeout_us ) {
			return;
		}
	}

	//	state 2
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_L ) {
		gpio_put_masked( mask, get_mouse_nibble_bits_3rd() << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > timeout_us ) {
			return;
		}
	}

	//	state 3
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_H ) {
		gpio_put_masked( mask, get_mouse_nibble_bits_4th() << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > timeout_us ) {
			return;
		}
	}

	//	state 4
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_L ) {
		gpio_put_masked( mask, get_mouse_nibble_bits_5th() << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > timeout_us ) {
			return;
		}
	}

	//	state 5
	while( gpio_get( MSX_SEL_PIN ) == MSX_SEL_H ) {
		gpio_put_masked( mask, get_mouse_nibble_bits_6th() << MSX_BUTTON_PIN );
		if( (my_get_us() - start_time) > timeout_us ) {
			return;
		}
	}
}

// --------------------------------------------------------------------
void response_core( void ) {

	for( ;; ) {
		if( process_mode == 0 ) {
			// DEGUB: turn on LED on GPIO 22
			gpio_put(21, true);
			gpio_put(22, false);
			joypad_mode();
		}
		else if( process_mode == 1 ) {
			mouse_mode();
		}
		else if( process_mode == 2 ){
			// DEGUB: turn on LED on GPIO 21
			gpio_put(21, false);
			gpio_put(22, true);
			// TODO: unify joypad_mode() and gpio_mode() into a single function with an input parameter (states matrix)
			gpio_mode();
		}
	}
}

//--------------------------------------------------------------------+
void led_blinking_task(void) {
	const uint32_t interval_ms = 250;
	static uint32_t start_ms = 0;

	// Blink every interval ms
	if (board_millis() - start_ms < interval_ms) return; // not enough time
	start_ms += interval_ms;
	if( process_mode == 0 ) {
		board_led_write( led_pattern[ 0 ][ led_state ] );
	}
	else if( process_mode == 1) {
		board_led_write( led_pattern[ mouse_resolution + 1 ][ led_state ] );
	}
	else {
		// GPIO pattern
		board_led_write( led_pattern[ 5 ][ led_state] );
	}
	led_state = (led_state + 1) & 7;
}

// --------------------------------------------------------------------
static void process_gamepad_report( hid_gamepad_report_t const *p_report ) {
	static const uint8_t default_matrix[5] = {
		0x33, 0x3F, 0x03, 0x3F, 0x3F
	};
	uint8_t matrix[5];
	uint8_t button;

	#if DEBUG_UART_ON
		printf( "process_gamepad_report()\n" );
	#endif

	//	            b5,b4,b3,b2,b1,b0
	//	matrix[0] = �� �� �k �k �` �r
	//	matrix[1] = �� �� �� �E �a �b
	//	matrix[2] = �k �k �k �k �` �r
	//	matrix[3] = �y �x �w �l �g �g
	//	matrix[4] = �g �g �g �g �` �r

	memcpy( matrix, default_matrix, sizeof(matrix) );
	if( p_report->x <= LEFT_THRESHOLD ) {
		matrix[1] &= 0x37;
	}
	else if( p_report->x >= RIGHT_THRESHOLD ) {
		matrix[1] &= 0x3B;
	}

	if( p_report->y <= TOP_THRESHOLD ) {
		matrix[0] &= 0x1F;
		matrix[1] &= 0x1F;
	}
	else if( p_report->y >= BOTTOM_THRESHOLD ) {
		matrix[0] &= 0x2F;
		matrix[1] &= 0x2F;
	}

	button = p_report->buttons;
	if( (button & A_BUTTON) != 0 ) {
		matrix[0] &= 0x3D;
		matrix[2] &= 0x3D;
		matrix[4] &= 0x3D;
	}
	if( (button & B_BUTTON) != 0 ) {
		matrix[1] &= 0x3D;
	}
	if( (button & C_BUTTON) != 0 ) {
		matrix[1] &= 0x3E;
	}
	if( (button & X_BUTTON) != 0 ) {
		matrix[3] &= 0x37;
	}
	if( (button & Y_BUTTON) != 0 ) {
		matrix[3] &= 0x2F;
	}
	if( (button & Z_BUTTON) != 0 ) {
		matrix[3] &= 0x1F;
	}
	if( (button & START_BUTTON) != 0 ) {
		matrix[0] &= 0x3E;
		matrix[2] &= 0x3E;
		matrix[4] &= 0x3E;
	}
	if( (button & MODE_BUTTON) != 0 ) {
		matrix[3] &= 0x3B;
	}
	memcpy( (void*) joymega_matrix, matrix, sizeof(matrix) );

	#if DEBUG_UART_ON
		printf( "pos_x = %d\r\n", (int)p_report->x );
		printf( "pos_y = %d\r\n", (int)p_report->y );
		printf( "buttons = 0x%02X\r\n", (int)p_report->buttons );
	#endif
}

// --------------------------------------------------------------------
static void process_joystick_report( my_hid_joystick_report_t const *p_report ) {
	static const uint8_t default_matrix[5] = {
		0x33, 0x3F, 0x03, 0x3F, 0x3F
	};
	uint8_t matrix[5];
	uint8_t button;

	#if DEBUG_UART_ON
		printf( "process_joystick_report()\n" );
	#endif

	//	            b5,b4,b3,b2,b1,b0
	//	matrix[0] = �� �� �k �k �` �r
	//	matrix[1] = �� �� �� �E �a �b
	//	matrix[2] = �k �k �k �k �` �r
	//	matrix[3] = �y �x �w �l �g �g
	//	matrix[4] = �g �g �g �g �` �r

	memcpy( matrix, default_matrix, sizeof(matrix) );
	if( p_report->x <= LEFT_THRESHOLD_JOYSTICK ) {
		matrix[1] &= 0x37;
	}
	else if( p_report->x >= RIGHT_THRESHOLD_JOYSTICK ) {
		matrix[1] &= 0x3B;
	}

	if( p_report->y <= TOP_THRESHOLD_JOYSTICK ) {
		matrix[0] &= 0x1F;
		matrix[1] &= 0x1F;
	}
	else if( p_report->y >= BOTTOM_THRESHOLD_JOYSTICK ) {
		matrix[0] &= 0x2F;
		matrix[1] &= 0x2F;
	}

	button = p_report->buttons;
	if( (button & A_BUTTON) != 0 ) {
		matrix[0] &= 0x3D;
		matrix[2] &= 0x3D;
		matrix[4] &= 0x3D;
	}
	if( (button & B_BUTTON) != 0 ) {
		matrix[1] &= 0x3D;
	}
	if( (button & C_BUTTON) != 0 ) {
		matrix[1] &= 0x3E;
	}
	if( (button & X_BUTTON) != 0 ) {
		matrix[3] &= 0x37;
	}
	if( (button & Y_BUTTON) != 0 ) {
		matrix[3] &= 0x2F;
	}
	if( (button & Z_BUTTON) != 0 ) {
		matrix[3] &= 0x1F;
	}
	if( (button & START_BUTTON) != 0 ) {
		matrix[0] &= 0x3E;
		matrix[2] &= 0x3E;
		matrix[4] &= 0x3E;
	}
	if( (button & MODE_BUTTON) != 0 ) {
		matrix[3] &= 0x3B;
	}
	memcpy( (void*) joymega_matrix, matrix, sizeof(matrix) );

	#if DEBUG_UART_ON
		printf( "pos_x = %d\r\n", (int)p_report->x );
		printf( "pos_y = %d\r\n", (int)p_report->y );
		printf( "buttons = 0x%02X\r\n", (int)p_report->buttons );
	#endif
}

// --------------------------------------------------------------------
// static void process_gpio_joypad() {
// 	static const uint8_t default_matrix[5] = {
// 		0x33, 0x3F, 0x03, 0x3F, 0x3F
// 	};
// 	uint8_t matrix[5];

// 	#if DEBUG_UART_ON
// 		printf( "process_gpio_joypad()\n" );
// 	#endif

// 	//	            b5,b4,b3,b2,b1,b0
// 	//	matrix[0] = �� �� �k �k �` �r
// 	//	matrix[1] = �� �� �� �E �a �b
// 	//	matrix[2] = �k �k �k �k �` �r
// 	//	matrix[3] = �y �x �w �l �g �g
// 	//	matrix[4] = �g �g �g �g �` �r

// 	// DEGUB: turn on LED on GPIO 22
// 	//gpio_put(22, true);

// 	gpio_buttons_state = gpio_get_all();

// 	memcpy( matrix, default_matrix, sizeof(matrix) );

//	// D-PAD
//  if( ((gpio_buttons_state << GPIO_LEFT_BUTTON) & 0x01) != 0 ) {
// 		matrix[1] &= 0x37;
// 	}
// 	else if( ((gpio_buttons_state << GPIO_RIGHT_BUTTON) & 0x01) != 0 ) {
// 		matrix[1] &= 0x3B;
// 	}

// 	if( ((gpio_buttons_state << GPIO_UP_BUTTON) & 0x01) != 0 ) {
// 		matrix[0] &= 0x1F;
// 		matrix[1] &= 0x1F;
// 	}
// 	else if( ((gpio_buttons_state << GPIO_DOWN_BUTTON) & 0x01) != 0 ) {
// 		matrix[0] &= 0x2F;
// 		matrix[1] &= 0x2F;
// 	}

// 	// // Buttons
// 	// if( (gpio_buttons_state & GPIO_A_BUTTON) != 0 ) {
// 	// 	matrix[0] &= 0x3D;
// 	// 	matrix[2] &= 0x3D;
// 	// 	matrix[4] &= 0x3D;
// 	// }
// 	// if( (gpio_buttons_state & GPIO_B_BUTTON) != 0 ) {
// 	// 	matrix[1] &= 0x3D;
// 	// }
// 	// if( (gpio_buttons_state & GPIO_C_BUTTON) != 0 ) {
// 	// 	matrix[1] &= 0x3E;
// 	// }
// 	// if( (gpio_buttons_state & GPIO_X_BUTTON) != 0 ) {
// 	// 	matrix[3] &= 0x37;
// 	// }
// 	// if( (gpio_buttons_state & GPIO_Y_BUTTON) != 0 ) {
// 	// 	matrix[3] &= 0x2F;
// 	// }
// 	// if( (gpio_buttons_state & GPIO_Z_BUTTON) != 0 ) {
// 	// 	matrix[3] &= 0x1F;
// 	// }
// 	// if( (gpio_buttons_state & GPIO_START_BUTTON) != 0 ) {
// 	// 	matrix[0] &= 0x3E;
// 	// 	matrix[2] &= 0x3E;
// 	// 	matrix[4] &= 0x3E;
// 	// }
// 	// if( (gpio_buttons_state & GPIO_MODE_BUTTON) != 0 ) {
// 	// 	matrix[3] &= 0x3B;
// 	// }

// 	memcpy( (void*) joymega_gpio_matrix, matrix, sizeof(matrix) );

// 	#if DEBUG_UART_ON
// 		printf( "buttons = 0x%02X\r\n", (int)gpio_buttons_state );
// 	#endif

// 	// DEGUB: turn off LED on GPIO 22
// 	//gpio_put(22, false);
// }


// TODO: Use Bitwise shift of the gpio_get_all return value instead of sequentially get all of the GPIO values 

static void process_gpio_joypad() {
	static const uint8_t default_matrix[5] = {
		0x33, 0x3F, 0x03, 0x3F, 0x3F
	};
	uint8_t matrix[5];

	#if DEBUG_UART_ON
		printf( "process_gpio_joypad()\n" );
	#endif

	//	            b5,b4,b3,b2,b1,b0
	//	matrix[0] = �� �� �k �k �` �r
	//	matrix[1] = �� �� �� �E �a �b
	//	matrix[2] = �k �k �k �k �` �r
	//	matrix[3] = �y �x �w �l �g �g
	//	matrix[4] = �g �g �g �g �` �r

	memcpy( matrix, default_matrix, sizeof(matrix) );

	if( gpio_get(GPIO_LEFT_BUTTON) != true ) {
		matrix[1] &= 0x37;
	}
	else if( gpio_get(GPIO_RIGHT_BUTTON) != true ) {
		matrix[1] &= 0x3B;
	}

	if( gpio_get(GPIO_UP_BUTTON) != true ) {
		matrix[0] &= 0x1F;
		matrix[1] &= 0x1F;
	}
	else if( gpio_get(GPIO_DOWN_BUTTON) != true ) {
		matrix[0] &= 0x2F;
		matrix[1] &= 0x2F;
	}

	// Buttons
	if( gpio_get(GPIO_A_BUTTON) != true ) {
		matrix[0] &= 0x3D;
		matrix[2] &= 0x3D;
		matrix[4] &= 0x3D;
	}
	if( gpio_get(GPIO_B_BUTTON) != true ) {
		matrix[1] &= 0x3D;
	}
	if( gpio_get(GPIO_C_BUTTON) != true ) {
		matrix[1] &= 0x3E;
	}
	if( gpio_get(GPIO_X_BUTTON) != true ) {
		matrix[3] &= 0x37;
	}
	if( gpio_get(GPIO_Y_BUTTON) != true ) {
		matrix[3] &= 0x2F;
	}
	if( gpio_get(GPIO_Z_BUTTON) != true ) {
		matrix[3] &= 0x1F;
	}
	if( gpio_get(GPIO_START_BUTTON) != true ) {
		matrix[0] &= 0x3E;
		matrix[2] &= 0x3E;
		matrix[4] &= 0x3E;
	}
	if( gpio_get(GPIO_MODE_BUTTON) != true ) {
		matrix[3] &= 0x3B;
	}

	memcpy( (void*) joymega_gpio_matrix, matrix, sizeof(matrix) );

	#if DEBUG_UART_ON
		printf( "buttons = 0x%02X\r\n", (int)gpio_buttons_state );
	#endif

}

// --------------------------------------------------------------------
static void process_mouse_report( hid_mouse_report_t const * report ) {
	int16_t delta_x;
	int16_t delta_y;
	int32_t send_data, last_mouse_button;
	int d1, d2, d3, d4;
	static const int reverse_inv4[] = { 3, 1, 2, 0 };
	static const int reverse16[] = { 0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15 };
	
	if( mouse_consume_data ) {
		mouse_consume_data = false;
		mouse_delta_x = 0;
		mouse_delta_y = 0;
	}
	delta_x = mouse_delta_x - (int16_t) report->x;
	if( delta_x < -127 ) {
		delta_x = -127;
	}
	else if( delta_x > 127 ) {
		delta_x = 127;
	}

	delta_y = mouse_delta_y - (int16_t) report->y;
	if( delta_y < -127 ) {
		delta_y = -127;
	}
	else if( delta_y > 127 ) {
		delta_y = 127;
	}

	last_mouse_button = mouse_button;
	mouse_button = (report->buttons & (MOUSE_BUTTON_RIGHT | MOUSE_BUTTON_LEFT | MOUSE_BUTTON_MIDDLE));

	//	���{�^���������ꂽ��𑜓x��ς���
	if( !(last_mouse_button & MOUSE_BUTTON_MIDDLE) && (mouse_button & MOUSE_BUTTON_MIDDLE) ) {
		mouse_resolution = (mouse_resolution + 1) & 3;
	}

	//	���M�p�f�[�^�����
	send_data = reverse_inv4[ mouse_button & 3 ];
	switch( mouse_resolution ) {
	default:
	case 0:
		d1 = reverse16[((delta_x     ) >> 4) & 0x0F ];
		d2 = reverse16[ (delta_x     )       & 0x0F ];
		d3 = reverse16[((delta_y     ) >> 4) & 0x0F ];
		d4 = reverse16[ (delta_y     )       & 0x0F ];
		break;
	case 1:
		d1 = reverse16[((delta_x     ) >> 4) & 0x0F ];
		d2 = reverse16[ (delta_x     )       & 0x0F ];
		d3 = reverse16[((delta_y >> 1) >> 4) & 0x0F ];
		d4 = reverse16[ (delta_y >> 1)       & 0x0F ];
		break;
	case 2:
		d1 = reverse16[((delta_x >> 1) >> 4) & 0x0F ];
		d2 = reverse16[ (delta_x >> 1)       & 0x0F ];
		d3 = reverse16[((delta_y >> 1) >> 4) & 0x0F ];
		d4 = reverse16[ (delta_y >> 1)       & 0x0F ];
		break;
	case 3:
		d1 = reverse16[((delta_x >> 1) >> 4) & 0x0F ];
		d2 = reverse16[ (delta_x >> 1)       & 0x0F ];
		d3 = reverse16[((delta_y >> 2) >> 4) & 0x0F ];
		d4 = reverse16[ (delta_y >> 2)       & 0x0F ];
		break;
	}
	send_data = send_data | (d1 << 4) | (d2 << 8) | (d3 << 12) | (d4 << 16);

	//	�r������͖ʓ|�Ȃ̂ŏȗ� (^^;
	mouse_delta_x = delta_x;
	mouse_delta_y = delta_y;
	mouse_current_data = send_data;
}

// --------------------------------------------------------------------
int main(void) {

	initialization();
	multicore_launch_core1( response_core );

	for( ;; ) {
		tuh_task();
		led_blinking_task();
		// GPIO gamepad reading
		process_gpio_joypad();
	}
	return 0;
}

// --------------------------------------------------------------------
//	HID���ڑ����ꂽ�Ƃ��ɌĂяo�����R�[���o�b�N
//
//	Callback to be called when a gamepad is connected.
//
void tuh_hid_mount_cb( uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len ) {

	//	Check device type
	uint8_t const interface_protocol = tuh_hid_interface_protocol( dev_addr, instance );

	#if DEBUG_UART_ON
		printf( "tuh_hid_mount_cb( %d, %d, %p, %d );\n", dev_addr, instance, desc_report, desc_len );
	#endif
	report_count[instance] = tuh_hid_parse_report_descriptor( report_info_arr[instance], MAX_REPORT, desc_report, desc_len );

	if( interface_protocol == HID_ITF_PROTOCOL_MOUSE ) {
		process_mode = 1;	//	mouse_mode
		mouse_delta_x = 0;
		mouse_delta_y = 0;
		mouse_button = 0;
		mouse_resolution = 0;
	}
	else {
		process_mode = 0;	//	joypad_mode
	}
}

// --------------------------------------------------------------------
//	HID���ؒf���ꂽ�Ƃ��ɌĂяo�����R�[���o�b�N
//
//	Callback to be called when the gamepad is disconnected.
//
void tuh_hid_umount_cb( uint8_t dev_addr, uint8_t instance ) {

	(void) dev_addr;
	(void) instance;
	#if DEBUG_UART_ON
		printf( "tuh_hid_umount_cb( %d, %d );\n", dev_addr, instance );
	#endif

	process_mode = 2;	//	gpio_mode
}

// --------------------------------------------------------------------
void tuh_hid_report_received_cb( uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len ) {

	(void) dev_addr;

	uint8_t const rpt_count = report_count[ instance ];
	tuh_hid_report_info_t* rpt_info_arr = report_info_arr[ instance ];
	tuh_hid_report_info_t* rpt_info = NULL;

	#if DEBUG_UART_ON
		printf( "tuh_hid_report_received_cb( %d, %d, %p, %d )\n", dev_addr, instance, report, len );
	#endif

	if ( rpt_count == 1 && rpt_info_arr[0].report_id == 0) {
		// Simple report without report ID as 1st byte
		rpt_info = &rpt_info_arr[0];
	} else {
		// Composite report, 1st byte is report ID, data starts from 2nd byte
		uint8_t const rpt_id = report[0];

		// Find report id in the arrray
		for( uint8_t i = 0; i < rpt_count; i++ ) {
			if( rpt_id == rpt_info_arr[i].report_id ) {
				rpt_info = &rpt_info_arr[i];
				break;
			}
		}

		report++;
		len--;
	}

	if( !rpt_info ) {
		#if DEBUG_UART_ON
			printf( "-- rpt_info == NULL\n" );
		#endif
		return;
	}

	#if DEBUG_UART_ON
		printf( "-- rpt_info->usage_page == %d\n", rpt_info->usage_page );
		printf( "-- rpt_info->usage == %d\n", rpt_info->usage );
	#endif
	if( rpt_info->usage_page == HID_USAGE_PAGE_DESKTOP ) {
		// DEBUG: turn on LED on GPIO 21 when an HID device is connected
		gpio_put(21, true);
		gpio_put(22, false);
		if( rpt_info->usage == HID_USAGE_DESKTOP_GAMEPAD ) {
			process_gamepad_report( (hid_gamepad_report_t const*) report );
		}
		else if( rpt_info->usage == HID_USAGE_DESKTOP_JOYSTICK ) {
			process_joystick_report( (my_hid_joystick_report_t const*) report );
		}
		else if( rpt_info->usage == HID_USAGE_DESKTOP_MOUSE ) {
			process_mouse_report( (hid_mouse_report_t const*) report );
		}
	}
}
