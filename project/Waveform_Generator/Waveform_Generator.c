/*
 * usb-vcp.c
 *
 * author: Furkan Cayci
 * description:
 *    USB Virtual COM Port (vcp) implementation.
 *    Uses USB CDC class (Communications Device Class) from libopencm3
 *    After programming, micro-USB will act as a serial port.
 *    Any other libopencm3 library calls are not used, thus not included.
 *    Only the USB stack is included.
 *
 * setup:
 *    uses the micro-USB port for PC communication.
 *    Connect it as you would to a serial port.
 *    (putty, screen, minicom, realterm, etc...)
 *    It will receive a character and print the next character.
 *
 * note:
 *    if you have a linux host, and the virtual port might appear
 *    at /dev/ttyACM0, in this case modemmanager might kick in
 *    and try to communicate with the device (thinking it as a modem)
 *    you can remove modemmanager package to fix this issue
 */


#include "stm32f407xx.h"
#include "system_stm32f4xx.h"
#include "stddef.h"
#include "libopencm3/usb/usbd.h"
#include "libopencm3/usb/cdc.h"
#include "usb-vcp.h"
#include "math.h"

#define sample_rate 50

/*
 * NOTE: This is added since we are not using
 * all of the libopencm3 function calls.
 * On usb_f107.c file, there is a single call to
 * enable OTGFS clock which is already done in
 * our code.
 */
void rcc_periph_clock_enable(x) {};

/*************************************************
* function declarations
*************************************************/

int  main(void);
void Default_Handler(void);
void Systick_Handler(void);
//uint32_t  tentothe(uint8_t p);
//void delay_ms(volatile uint32_t);
void print(usbd_device *usbd_dev);
void init_systick(uint32_t s, uint8_t cen);
void init_timer2(uint32_t psc, uint32_t arr);
void prints(usbd_device *usbd_dev, char m[128], uint8_t n);
void printm(usbd_device *usbd_dev, char *message, uint8_t n);

/*************************************************
* variables
*************************************************/

//static volatile uint32_t tDelay;
uint8_t 	counter=0, process=0, i=0xF;
uint32_t  frequency_int=1000, amplitude_int=2000;
uint16_t  x=0;
uint16_t Signal[sample_rate];
bool request=0, request_1=0, request_2=0;
bool mode=0;
char buf[64];																													
char waveform, frequency[7], amplitude[4];

const char m1[]	=	"\n\rChoose waveform:\n\r[1] Sine\n\r[2] Square\n\r[3] Sawtooth";
const char m2[]	=	"\n\r[4] Triangular\n\r[5] White Noise";
const char m3[] = "\n\rAccepted, press ENTER";
const char m4[] = "\n\rInvalid waveform selection!!!";
const char m5[] = "\n\rUser >";
const char m6[] = "\n\rNumber of digits exceeded limit!!!";
const char m7[] = "\n\rSet frequency";
const char m8[] = "\n\rValue exceeded limits!!!";
const char m9[] = "\n\rSet amplitude (1 to 3,000 mV)";
const char m10[] = "\n\rGenerating Signal <!>";

uint32_t max_freq = 2000000/sample_rate;
char max_freq_string[6];

/*************************************************
* default interrupt handler
*************************************************/

void Default_Handler(void)
{
	for (;;);  // Wait forever
}

/*************************************************
* default systick interrupt handler
*************************************************/

void Systick_Handler(void){											// Controls LED for indication of processes
	
	
	if(mode == 0){	
  		GPIOD->ODR = (uint16_t)(i << 12); 						// Write i to GPIOD->ODR register
	if( i == 0xF0 ){
		init_systick(5250000, 1);			// Set systick to 250 ms
		i = 1;
	}
	if ( i == 0x08 ){
		i = 1;		
	} else {
		i = (uint8_t)(i << 1);
	}
	}

	else if(mode == 1){
		GPIOD->ODR ^= (uint16_t)(i << 12);					// Toggle all LEDs on PD12 13, 14, 15
	}

}

void tim2_handler(void)
{
	// Clear pending bit first
	// This is important because of the delay,
	// interrupt handler gets fired off twice!
	// if this is cleared at the end
	TIM2->SR = (uint16_t)(~(1 << 0));

	if(waveform == 0x31 || waveform == 0x32 || waveform == 0x33 || waveform == 0x34){
			if(x>=sample_rate)x=0;
			DAC->DHR12R1 = Signal[x];
			DAC->SWTRIGR |= (1 << 0 ); // trigger ch1
			x++;
		}
		else if(waveform == 0x35){
			DAC->DHR12R1 = amplitude_int*4096/3300*rand();
			DAC->SWTRIGR |= (1 << 0 ); // trigger ch1
		}
}

/*************************************************
* Vector Table
*************************************************/
// get the stack pointer location from linker
typedef void (* const intfunc)(void);
extern unsigned long __stack;

// attribute puts table in beginning of .vectors section
//   which is the beginning of .text section in the linker script
// Add other vectors -in order- here
// Vector table can be found on page 372 in RM0090
__attribute__ ((section(".vectors")))
void (* const vector_table[])(void) = {
	(intfunc)((unsigned long)&__stack), /* 0x000 Stack Pointer */
	Reset_Handler,                      /* 0x004 Reset         */
	Default_Handler,                    /* 0x008 NMI           */
	Default_Handler,                    /* 0x00C HardFault     */
	Default_Handler,                    /* 0x010 MemManage     */
	Default_Handler,                    /* 0x014 BusFault      */
	Default_Handler,                    /* 0x018 UsageFault    */
	0,                                  /* 0x01C Reserved      */
	0,                                  /* 0x020 Reserved      */
	0,                                  /* 0x024 Reserved      */
	0,                                  /* 0x028 Reserved      */
	Default_Handler,                    /* 0x02C SVCall        */
	Default_Handler,                    /* 0x030 Debug Monitor */
	0,                                  /* 0x034 Reserved      */
	Default_Handler,                    /* 0x038 PendSV        */
	Systick_Handler,                     /* 0x03C SysTick       */
	0,                                  /* 0x040 Window WatchDog Interrupt                                         */
	0,                                  /* 0x044 PVD through EXTI Line detection Interrupt                         */
	0,                                  /* 0x048 Tamper and TimeStamp interrupts through the EXTI line             */
	0,                                  /* 0x04C RTC Wakeup interrupt through the EXTI line                        */
	0,                                  /* 0x050 FLASH global Interrupt                                            */
	0,                                  /* 0x054 RCC global Interrupt                                              */
	0,                                  /* 0x058 EXTI Line0 Interrupt                                              */
	0,                                  /* 0x05C EXTI Line1 Interrupt                                              */
	0,                                  /* 0x060 EXTI Line2 Interrupt                                              */
	0,                                  /* 0x064 EXTI Line3 Interrupt                                              */
	0,                                  /* 0x068 EXTI Line4 Interrupt                                              */
	0,                                  /* 0x06C DMA1 Stream 0 global Interrupt                                    */
	0,                                  /* 0x070 DMA1 Stream 1 global Interrupt                                    */
	0,                                  /* 0x074 DMA1 Stream 2 global Interrupt                                    */
	0,                                  /* 0x078 DMA1 Stream 3 global Interrupt                                    */
	0,                                  /* 0x07C DMA1 Stream 4 global Interrupt                                    */
	0,                                  /* 0x080 DMA1 Stream 5 global Interrupt                                    */
	0,                                  /* 0x084 DMA1 Stream 6 global Interrupt                                    */
	0,                                  /* 0x088 ADC1, ADC2 and ADC3 global Interrupts                             */
	0,                                  /* 0x08C CAN1 TX Interrupt                                                 */
	0,                                  /* 0x090 CAN1 RX0 Interrupt                                                */
	0,                                  /* 0x094 CAN1 RX1 Interrupt                                                */
	0,                                  /* 0x098 CAN1 SCE Interrupt                                                */
	0,                                  /* 0x09C External Line[9:5] Interrupts                                     */
	0,                                  /* 0x0A0 TIM1 Break interrupt and TIM9 global interrupt                    */
	0,                                  /* 0x0A4 TIM1 Update Interrupt and TIM10 global interrupt                  */
	0,                                  /* 0x0A8 TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
	0,                                  /* 0x0AC TIM1 Capture Compare Interrupt                                    */
	tim2_handler,                       /* 0x0B0 TIM2 global Interrupt                                             */
	0,                                  /* 0x0B4 TIM3 global Interrupt                                             */
	0,                                  /* 0x0B8 TIM4 global Interrupt                                             */
	0,                                  /* 0x0BC I2C1 Event Interrupt                                              */
	0,                                  /* 0x0C0 I2C1 Error Interrupt                                              */
	0,                                  /* 0x0C4 I2C2 Event Interrupt                                              */
	0,                                  /* 0x0C8 I2C2 Error Interrupt                                              */
	0,                                  /* 0x0CC SPI1 global Interrupt                                             */
	0,                                  /* 0x0D0 SPI2 global Interrupt                                             */
	0,                                  /* 0x0D4 USART1 global Interrupt                                           */
	0,                                  /* 0x0D8 USART2 global Interrupt                                           */
	0,                                  /* 0x0DC USART3 global Interrupt                                           */
	0,                                  /* 0x0E0 External Line[15:10] Interrupts                                   */
	0,                                  /* 0x0E4 RTC Alarm (A and B) through EXTI Line Interrupt                   */
	0,                                  /* 0x0E8 USB OTG FS Wakeup through EXTI line interrupt                     */
	0,                                  /* 0x0EC TIM8 Break Interrupt and TIM12 global interrupt                   */
	0,                                  /* 0x0F0 TIM8 Update Interrupt and TIM13 global interrupt                  */
	0,                                  /* 0x0F4 TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
	0,                                  /* 0x0F8 TIM8 Capture Compare global interrupt                             */
	0,                                  /* 0x0FC DMA1 Stream7 Interrupt                                            */
	0,                                  /* 0x100 FSMC global Interrupt                                             */
	0,                                  /* 0x104 SDIO global Interrupt                                             */
	0,                                  /* 0x108 TIM5 global Interrupt                                             */
	0,                                  /* 0x10C SPI3 global Interrupt                                             */
	0,                                  /* 0x110 UART4 global Interrupt                                            */
	0,                                  /* 0x114 UART5 global Interrupt                                            */
	0,                                  /* 0x118 TIM6 global and DAC1&2 underrun error  interrupts                 */
	0,                                  /* 0x11C TIM7 global interrupt                                             */
	0,                                  /* 0x120 DMA2 Stream 0 global Interrupt                                    */
	0,                                  /* 0x124 DMA2 Stream 1 global Interrupt                                    */
	0,                                  /* 0x128 DMA2 Stream 2 global Interrupt                                    */
	0,                                  /* 0x12C DMA2 Stream 3 global Interrupt                                    */
	0,                                  /* 0x130 DMA2 Stream 4 global Interrupt                                    */
	0,                                  /* 0x134 Ethernet global Interrupt                                         */
	0,                                  /* 0x138 Ethernet Wakeup through EXTI line Interrupt                       */
	0,                                  /* 0x13C CAN2 TX Interrupt                                                 */
	0,                                  /* 0x140 CAN2 RX0 Interrupt                                                */
	0,                                  /* 0x144 CAN2 RX1 Interrupt                                                */
	0,                                  /* 0x148 CAN2 SCE Interrupt                                                */
	0,                                  /* 0x14C USB OTG FS global Interrupt                                       */
	0,                                  /* 0x150 DMA2 Stream 5 global interrupt                                    */
	0,                                  /* 0x154 DMA2 Stream 6 global interrupt                                    */
	0,                                  /* 0x158 DMA2 Stream 7 global interrupt                                    */
	0,                                  /* 0x15C USART6 global interrupt                                           */
	0,                                  /* 0x160 I2C3 event interrupt                                              */
	0,                                  /* 0x164 I2C3 error interrupt                                              */
	0,                                  /* 0x168 USB OTG HS End Point 1 Out global interrupt                       */
	0,                                  /* 0x16C USB OTG HS End Point 1 In global interrupt                        */
	0,                                  /* 0x170 USB OTG HS Wakeup through EXTI interrupt                          */
	0,                                  /* 0x174 USB OTG HS global interrupt                                       */
	0,                                  /* 0x178 DCMI global interrupt                                             */
	0,                                  /* 0x17C RNG global Interrupt                                              */
	0                                   /* 0x180 FPU global interrupt                                              */
};



/*************************************************
* initialize SysTick & TIM2
*************************************************/
void init_systick(uint32_t s, uint8_t cen)
{
	// Clear CTRL register
	SysTick->CTRL = 0x00000;
	// Main clock source is running with HSI by default which is at 8 Mhz.
	// SysTick clock source can be set with CTRL register (Bit 2)
	// 0: Processor clock/8 (AHB/8)
	// 1: Processor clock (AHB)
	SysTick->CTRL |= (0 << 2);
	// Enable callback (bit 1)
	SysTick->CTRL |= ((uint32_t)cen << 1);
	// Load the value
	SysTick->LOAD = s;
	// Set the current value to 0
	SysTick->VAL = 0;
	// Enable SysTick (bit 0)
	SysTick->CTRL |= (1 << 0);
}

void init_timer2(uint32_t psc, uint32_t arr){

	// Enable TIM2 Clock
	RCC->APB1ENR |= (1 << 0);

	// Timer clock runs at ABP1 * 2
	//   since ABP1 is set to /4 of fCLK
	//   thus 168M/4 * 2 = 84Mhz
	//   set prescaler to 83999
	//   it will increment counter every prescalar cycles
	// fCK_PSC / (PSC[15:0] + 1)
	// 84 Mhz / 8399 + 1 = 10 khz timer clock speed
	TIM2->PSC = psc-1;

	// Set the auto-reload value to 10000
	//   which should give 1 second timer interrupts
	TIM2->ARR = arr;

	// Update Interrupt Enable
	TIM2->DIER |= (1 << 0);

	// Enable TIM2 IRQ from NVIC
	NVIC_EnableIRQ(TIM2_IRQn);

	// Enable Timer 2 module (CEN, bit0)
	TIM2->CR1 |= (1 << 0);
}

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		return USBD_REQ_HANDLED;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding)) {
			return USBD_REQ_NOTSUPP;
		}

		return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NOTSUPP;
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	/* setup receive callback */
	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);

	/* setup transmit callback */
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);

	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

/*
 * Receiver callback function
 */
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	 
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 1);										// get 1 byte

	while (usbd_ep_write_packet(usbd_dev, 0x82, buf, 1) == 0);								// echo it

	if(process == 0 ){
		
		if(waveform == 0 && request == 1){
			waveform = buf[0];
			if(waveform <=0x35 && waveform >=0x31){
				printm(usbd_dev, m3, 23);
			}
			else{
				printm(usbd_dev, m4, 31);
				printm(usbd_dev, m5,  8);
				waveform = 0;
			} 
		}

		if(request == 0){
			print(usbd_dev);
			printm(usbd_dev, m5, 8);
			request = 1;
			buf[0]=0;
		}

		if(buf[0] == '\r'){
			process++;
			buf[0]=0;
		}
	}

	if(process == 1){

		if( counter < 6 ){
			if( request_1 == 1 && buf[0] != '\r' ){																							
      			frequency[counter]=buf[0];
	  			counter++;
			}
		}

    	else if( buf[0] != '\r' ){
			//prints(usbd_dev, '\b\b\b\b\b\b\b\b', 8);
			counter = 0;
			printm(usbd_dev, m6, 36);
			printm(usbd_dev, m5, 8);
		}

		if( request_1 == 0 ){
			printm(usbd_dev, m7, 15);
			prints(usbd_dev, "( 1 to ", 7);
			printm(usbd_dev, &max_freq_string, 6);
			prints(usbd_dev, " )", 2);
			printm(usbd_dev, m5, 8);
			request_1 = 1;
			counter = 0;
			buf[0]=0;
		}

		if( buf[0] == '\r' ){
			if(counter > 1){
				frequency_int = 0;
			for(uint8_t i=counter; i>=1; i--){
				frequency_int += pow(10,i-1)*(frequency[counter-i] - 0x30);	
			}
			if( frequency_int <= max_freq ){
				process++;
				counter=0;
				buf[0]=0;
			}
			else{
				printm(usbd_dev, m8, 26);
				printm(usbd_dev, m5, 8);
				frequency_int = 1000;
				counter = 0;
			}
			}
			else{
				process++;
				counter=0;
				buf[0]=0;	
			}
		}

	}
		
	

	if(process == 2){

      	if( counter < 4 ){
				if( request_2 == 1 && buf[0] != '\r' ){																							
      				amplitude[counter]=buf[0];
	  				counter++;
				}
		}

		else if( buf[0] != '\r' ){
				//prints(usbd_dev, '\b\b\b\b\b', 5);
				counter = 0;
				printm(usbd_dev, m6, 36);
				printm(usbd_dev, m5, 8);
		}
    
		if( request_2 == 0 ){
			printm(usbd_dev, m9, 31);
			printm(usbd_dev, m5,  8);
			request_2 = 1;
			counter = 0;
		}

		if( buf[0] == '\r' ){
			if(counter > 1){
				amplitude_int = 0;
			for(uint8_t i=counter; i>=1; i--){
				amplitude_int += pow(10,i-1)*(amplitude[counter-i] - 0x30);	
			}
			if( amplitude_int <= 3000 ){
				GPIOD->ODR ^= (0x0000F000)&(GPIOD->ODR);	
				i = 0xF;					
				mode = 1;
			}
			else{
				printm(usbd_dev, m8, 26);
				printm(usbd_dev, m5, 8);
				amplitude_int = 2000;
				counter = 0;
			}
			}
			else{
				GPIOD->ODR ^= (0x0000F000)&(GPIOD->ODR);	
				i = 0xF;					
				mode = 1;	
			}
		}

	}

}

void print(usbd_device *usbd_dev){
	while (usbd_ep_write_packet(usbd_dev, 0x82, m1, 54 ) == 0);
	while (usbd_ep_write_packet(usbd_dev, 0x82, m2, 35 ) == 0);	
}

void prints(usbd_device *usbd_dev, char message[128], uint8_t n){
	while (usbd_ep_write_packet(usbd_dev, 0x82, message, n ) == 0);	
}

void printm(usbd_device *usbd_dev, char *message, uint8_t n){
	while (usbd_ep_write_packet(usbd_dev, 0x82, message, n ) == 0);	
}


/*************************************************
* main code starts from here
*************************************************/


int main(void)
{
	set_sysclk_to_168();
	init_systick(42000000, 1);

	/* Enable GPIOD clock : bit3 */
	RCC->AHB1ENR |= (1 << 3);

	/* Set up pins 12,13,14,15 as output */
	GPIOD->MODER &= 0x00FFFFFF;
	GPIOD->MODER |= 0x55000000;

	/* Turn on last LED */
	GPIOD->ODR |= 0xF000;

	/* Enable GPIOA clock : bit0 */
	RCC->AHB1ENR |= (1 << 0);

	/* Enable OTG FS clock : bit7 in AHB2 */
	RCC->AHB2ENR |= (1 << 7);

	/* Setup GPIOA pins 9, 11, 12 as AF for USB */
	GPIOA->MODER &= (uint32_t)~(0x03CC0000);
	GPIOA->MODER &= 0xFFFFFFFC;
	GPIOA->MODER |= (0x2 << 18); // pin 9
	GPIOA->MODER |= (0x2 << 22); // pin 11
	GPIOA->MODER |= (0x2 << 24); // pin 12

	// Choose OTG FS as Alternative Function 10 for pins 9, 11, 12
	GPIOA->AFR[1] |= (10 << 4);  // pin 9
	GPIOA->AFR[1] |= (10 << 12);  // pin 11
	GPIOA->AFR[1] |= (10 << 16);  // pin 12

	

	// enable GPIOA clock, bit 0 on AHB1ENR
	RCC->AHB1ENR |= (1 << 0);

	GPIOA->MODER &= 0xFFFFFCFF; // Reset bits 8-9 to clear old values
	GPIOA->MODER |= 0x00000300; // Set pin 4 to analog mode (0b11)

	// enable DAC clock, bit 29 on APB1ENR
	RCC->APB1ENR |= (1 << 29);

	DAC->CR |= (1 << 0); // enable DAC channel 1
	DAC->CR |= (0 << 1); // enable DAC ch1 output buffer
	DAC->CR |= (1 << 2); // enable trigger
	DAC->CR |= (7 << 3); // choose sw trigger as source (0b111)

	// set output to Vref * (dac_value/0xFFF)
	DAC->DHR12R1 = 0xFFF;
	DAC->SWTRIGR |= (1 << 0); // trigger ch1

	usbd_device *usbd_dev;

	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
			usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer)
	);

	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

	uint32_t a = max_freq;
	for(int i=0; i<6; i++){
		uint32_t h = pow(10,5-i);
		max_freq_string[i] = a/h + 0x30;
		a = a % h;
	}

	while(1){
		usbd_poll(usbd_dev);
		if(mode == 1)break;
	}

	if(waveform == 0x31){
		prints(usbd_dev, "\n\rSine Wave\n\r", 13);
		for(int i=0; i<sample_rate; i++){
			Signal[i] = ( amplitude_int*4025/3000.0 )*( sin(2*M_PI*(1.0/sample_rate)*i) + 1 )/2 + 70;
		}
	}
	else if(waveform == 0x32){
		prints(usbd_dev, "\n\rSquare Wave\n\r", 15);
		for(int i=0; i<sample_rate/2; i++){
			Signal[i] = 70;
		}
		for(int i=sample_rate/2; i<sample_rate; i++){
			Signal[i] = amplitude_int*4025/3000.0 + 70;
		}
	}
	else if(waveform == 0x33){
		prints(usbd_dev, "\n\rSawtooth Wave\n\r", 17);
		float m = (amplitude_int*4025/3000.0)/sample_rate;
		for(int i=0; i<sample_rate; i++){
			Signal[i] = m*i + 70;											//(amplitude_int*4096/3300)*(i/sample_rate);
		}
	}
	else if(waveform == 0x34){
		prints(usbd_dev, "\n\rTriangular Wave\n\r", 19);
		float m = 2*(amplitude_int*4025/3000.0)/sample_rate;
		for(int i=0; i<sample_rate/2; i++){
			Signal[i] = m*i + 70;
		}
		for(int i=sample_rate/2; i>=0; i--){
			Signal[sample_rate-i] = m*i + 70; 					//(amplitude_int*4096/3300)*(float)(i/sample_rate);
		}
	}
	else if(waveform == 0x35){
		prints(usbd_dev, "\n\rWhite Noise\n\r", 15);
	}
	
		uint16_t b;
		uint16_t p;
		char digit3;
		char digit2;
		char digit1;
		char digit0;
		char p3;
		char p2;
		char p1;
		char p0;

	for(uint16_t i=0; i<sample_rate; i++){

		b = Signal[i];

		digit3 = b/1000;
		digit3 += 0x30;	
		b = b%1000;

		digit2 = b/100;
		digit2 += 0x30;	
		b = b%100;

		digit1 = b/10;
		digit1 += 0x30;	
		b = b%10;

		digit0 = b + 0x30;

		p = i;

		p3 = p/1000;
		p3 += 0x30;	
		p = p%1000;

		p2 = p/100;
		p2 += 0x30;	
		p = p%100;

		p1 = p/10;
		p1 += 0x30;	
		p = p%10;

		p0 = p + 0x30;

		prints(usbd_dev, "\n\r", 3);
		prints(usbd_dev, "Signal[", 7);
		prints(usbd_dev, &p3, 1);
		prints(usbd_dev, &p2, 1);
		prints(usbd_dev, &p1, 1);
		prints(usbd_dev, &p0, 1);
		prints(usbd_dev, "] = ", 4);
		printm(usbd_dev, &digit3, 1);
		printm(usbd_dev, &digit2, 1);
		printm(usbd_dev, &digit1, 1);
		printm(usbd_dev, &digit0, 1);

	}
	
	printm(usbd_dev, m10, 23);

	float t = 1/(frequency_int*sample_rate);
	uint32_t psc = 42000000/(frequency_int*sample_rate);	//  In some frequency values prescaler cannot be set pricisely,
															//	which causes to bigger frequency values.
	init_timer2(psc,1);										//	Don't forget to set the piorities among timers,  
	/*														//	to avoid latency in signal generation.
	while(1){
		
		if(waveform == 0x31 || waveform == 0x32 || waveform == 0x33 || waveform == 0x34){
			if(x>=1000)x=0;
			DAC->DHR12R1 = Signal[x];
			DAC->SWTRIGR |= (1 << 0 ); // trigger ch1
			x++;
			//for (int i=0; i<1000; i++);
		}
		else if(waveform == 0x35){ 
			DAC->DHR12R1 = amplitude_int*4096/3000.0*(rand()%100)/100;
			DAC->SWTRIGR |= (1 << 0 ); // trigger ch1
		}
		
	}
*/
	return 0;
}


/*
uint32_t tentothe(uint8_t p){
	uint32_t result=1;
	for(uint8_t i=0; i<p; i++){
	result =result*10;
	}
	return result;
}
*/
