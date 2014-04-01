#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_spi.h"
#include "LIS3DSH.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"
#include <string.h>
#include <math.h>
#include "usbd_cdc_vcp.h" // подключаем USB CDC

void delay() {
	volatile  uint32_t i;
	for(i=0;i<=130000;i++)
		;
}

/*
// itoa:  конвертируем n в символы в s
void itoa(int n, char s[])
{
    int i, sign;

    if ((sign = n) < 0)  // записываем знак
        n = -n;          // делаем n положительным числом
    i = 0;
    do {       // генерируем цифры в обратном порядке
        s[i++] = n % 10 + '0';   // берем следующую цифру
    } while ((n /= 10) > 0);     // удаляем
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    reverse(s);
}
*/


//Функция отправляет байт в UART
void send_to_uart(uint8_t data)  {
 while(!(USART2->SR & USART_SR_TC));
 USART2->DR=data;
}

//Функция отправляет строку в UART
void send_str(char * string) {
 uint8_t i=0;
 while(string[i]) {
  send_to_uart(string[i]);
  i++;
 }

}


void pinE0_init(void){

GPIO_InitTypeDef GPIO_InitStructure;

RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

/* Configure PA01 in input pull down mode */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
GPIO_Init(GPIOE, &GPIO_InitStructure);

}



void led_init()
{
	/* GPIOD Periph clock enable */
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}


void enable_int() {

	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	// Enable GPIOA clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	//Enable SYSCFG clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// Configure PE0 pin as input floating
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Connect EXTI Line0 to PE0 pin
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);

	//Configure EXTI Line0
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//Enable and set EXTI Line0 Interrupt to the lowest priority
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


    // Configure PE1 pin as input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // Connect EXTI Line0 to PE1 pin
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);

    // Configure EXTI Line1
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable and set EXTI Line0 Interrupt to the lowest priority
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}




void led_enable(uint16_t number_Pin){

	GPIO_SetBits(GPIOD, number_Pin);
}

void led_disable(uint16_t number_Pin){

	GPIO_ResetBits(GPIOD, number_Pin);
}

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;



int main(void)
{

	//char str[];
	SystemInit();

	// включаем usb
	USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);

	//usart_init(); //Инициализация UART
	led_init();
	enable_int();
	pinE0_init();
	float aX, aY, aZ, old_aZ, change;
	uint16_t old_change;
	int8_t temperature, counter=0;
	char check1, check2, buttonState;
	char ch3[]="receive 1\x0D\x0A";
	char ch4[]="receive 2\x0D\x0A";
	char ch5[]="receive 3\x0D\x0A";
	char ch6[4];
	char ch7[]="\x0D\x0A";

	LIS3DSH_Init();

	//Настройки для пробуждения
	LIS3DSH_Write(0x21, 0x01);
	LIS3DSH_Write(0x23, 0x48);
	LIS3DSH_Write(0x20, 0x67);
	LIS3DSH_Write(0x24, 0x00);
	LIS3DSH_Write(0x51, 0x03);
	LIS3DSH_Write(0x40, 0x03);
	LIS3DSH_Write(0x41, 0x11);

	//LIS3DSH_Write(LIS3DSH_Reg_X_Offset, LIS3DSH_Read(LIS3DSH_Reg_X_Offset));
	//LIS3DSH_Write(LIS3DSH_Reg_Y_Offset, LIS3DSH_Read(LIS3DSH_Reg_Y_Offset));
	//LIS3DSH_Write(LIS3DSH_Reg_Z_Offset, LIS3DSH_Read(LIS3DSH_Reg_Z_Offset));

	old_aZ=1;

	while(1){

	//check1=LIS3DSH_Read(LIS3DSH_Reg_Interrupt_State);
	//check2=LIS3DSH_Read(LIS3DSH_Reg_Interrupt_Manage_Pin_2);


	//temperature=LIS3DSH_Read(LIS3DSH_Reg_Temp_Out)+25;
	//aX = LIS3DSH_Get_X_Out(LIS3DSH_Sense_2g);
	//aY = LIS3DSH_Get_Y_Out(LIS3DSH_Sense_2g);
	aZ = LIS3DSH_Get_Z_Out(LIS3DSH_Sense_2g);

	change=(old_aZ-aZ)/aZ;

	if (change<0) {

		change*=-1;

	}

	if (change>1) {

		led_enable(GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
		old_aZ=aZ;

	} else if (change>0.2) {

		led_enable(GPIO_Pin_13|GPIO_Pin_14);
		old_aZ=aZ;

	} else if (change>0.05) {

		led_enable(GPIO_Pin_13);
		old_aZ=aZ;

	} else {

		led_disable(GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);

	}

	/*
	if(usb_cdc_kbhit()){ // проверка: приняты данные?
			char c;
			c = usb_cdc_getc();  // получение байта с usb
			switch(c){ // разбор принятого байта
				case '1':
					usb_cdc_printf((char *)ch3); // отправка байта на хост
					break;
				case '2':
					usb_cdc_printf((char *)ch4);
					break;
				case '3':
					usb_cdc_printf(ch5);
					break;
				case '4':
					usb_cdc_printf(ch6);
					break;
			}
		}
	*/
	old_change=change*1000;
	sprintf(ch6, "%d", old_change);
	usb_cdc_printf((char *)&ch6);
	usb_cdc_printf((char *)&ch7);
	delay();
	delay();
	delay();
	delay();
	delay();
	delay();
	delay();
	delay();
	delay();
	delay();


	buttonState = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0);

	if (buttonState==1) {

		led_disable(GPIO_Pin_12);
	}

	delay();


	}

}


	void EXTI0_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
		// Этот код исполнится когда акселерометр как следует тряхнут
		// Если нет UARTа то можно просто подмигнуть светодиодом раскомментировав строку ниже
		led_enable(GPIO_Pin_12);
		//send_str("Interrupt occurred\n");
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}



	void EXTI1_IRQHandler(void) {
		if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
		// Этот код исполнится когда акселерометр как следует тряхнут
		// Если нет UARTа то можно просто подмигнуть светодиодом раскомментировав строку ниже
		led_enable(GPIO_Pin_12);
		//send_str("Interrupt occurred\n");
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}


