/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  *
  * To create a bin file then in post-build steps in properties:
 * arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin"; arm-none-eabi-size "${BuildArtifactFileName}"
 *
 * To create a hex file:
 * arm-none-eabi-objcopy -O ihex "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.hex"; arm-none-eabi-size "${BuildArtifactFileName}"
 *
 * To flash on windows:
 * "C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe" -c SWD -p program.hex -Rst
 *
 ******************************************************************************
*/

#include "platform_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "dev_uart.h"
#include "dev_usb_uart.h"
#include "dev_led.h"
#include "joystick.h"


void usb_rx_parser(uint8_t * recv_buffer, uint16_t recv_len);
void joys_callback(char * str_result, uint8_t str_len);

/* Declare glb struct and initialize buffers */
struct tp_glb glb;

DECLARE_UART_DEV(dbg_uart, USART1, 115200, 256, 10, 1);

/* Declare USB endpoints */
DECLARE_COMM_BUFFER(usb_comm_buffer, 128, 128);
DECLARE_USB_UART_DEV(usb_comm, ENDP1, ENDP1_TXADDR, ENDP3, ENDP3_RXADDR, VIRTUAL_COM_PORT_DATA_SIZE, 10, usb_rx_parser, NULL);

DECLARE_DEV_LED(led_status, PORT_STATUS_LED, PIN_STATUS_LED);

DECLARE_JOYS(joys, joys_callback);

void main_loop(void)
{
	/* 1 ms timer */
	if (glb.tmr_1ms) {
		glb.tmr_1ms = 0;

		if ((glb.tmr_250ms++) >= LED_TICK_MS) {
			glb.tmr_250ms = 0;
			dev_led_update(&led_status);
		}
		dev_uart_update(&dbg_uart);
		USB_update_timers(&usb_comm);

		if ((glb.tmr_joys_update++) >= JOYS_UPDATE_TMR_MS) {
			glb.tmr_joys_update = 0;
			joys_update(glb.adc_axis_x, glb.adc_axis_y);
		}
	}

	/* USB reception is done by polling,
	 * but normally the interrupt reception
	 * is done in EPx_OUT_Callback
	 * */
	if (bDeviceState == CONFIGURED) {
		USB_dev_recv(&usb_comm);
	}
}

int main(void)
{
	if (SysTick_Config(SystemCoreClock / 1000)) {
		/* Capture error */
		while (1);
	}

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

	/* System Clocks Configuration */
	RCC_Configuration();
	/* NVIC Configuration */
	NVIC_Configuration();
	/* GPIO Configuration */
	GPIO_Configuration();
	set_trace_level(
			0
			| TRACE_LEVEL_DEFAULT
			| TRACE_LEVEL_JOYS
			,1);
	dev_uart_add(&dbg_uart);
	USB_Configuration();
	USB_Init();
	dev_led_init(&led_status);	
	
	usb_comm.buffer = &usb_comm_buffer;
	USB_dev_init(&usb_comm);
	/* ADC Configuration */
	ADC_Configuration();
	// /* Initialize joystick */
	joys_init(&joys);

	TRACE(("Application started...\n"));
	dev_led_set_pattern(&led_status, LED_PATTERN_IDLE);

	while(1) {
		main_loop();
	}
}

/* send available data to USB */
void joys_callback(char * str_result, uint8_t str_len)
{
	char usb_str[str_len + 5];
	sprintf(usb_str, "%s%s", "DATA=", str_result);
	USB_dev_send(&usb_comm, usb_str, str_len+5);
}

void usb_rx_parser(uint8_t * recv_buffer, uint16_t recv_len)
{
	TRACE(("USB0_recv: %d\n", recv_len));
}
