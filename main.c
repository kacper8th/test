#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
#include "queue.h"
#include "timers.h"

#include "semphr.h"

#include "lcd/lcd.h"
#include "config.h"
#include "i2c/i2c_soft.h"
#include "i2c/ds1307.h"
#include "1Wire/ds18x20.h"
#include "keyb/keyb.h"
#include "uart/uart.h"
#include "adc/adc.h"

/**************************************************************************
 * Deklaracje Funkcji
***************************************************************************/
inline void init(void)
{
	lcd_init();
	LED1_On();
	i2c_init();
	search_sensors();
	KeybInit();
	ADC_Init();
	USART_Init(__UBRR);
}
static void prv10msTimerCallback(TimerHandle_t xTimer);
static void prvRtcTimerCallback(TimerHandle_t xTimer);
static void prvTempTimerCallback(TimerHandle_t xTimer);
void vTaskInfo(void *pvParameters);
void vTask2(void *pvParameters);
void vTask1(void *pvParameters);
void vADCTask(void *pvParameters);


/**************************************************************************
 * Zmienne Globalne
***************************************************************************/
t_DS1307_data DS1307_data;
t_DS18X20_data DS18X20_data;
size_t xFreeHeapSize;

SemaphoreHandle_t xMutexUart;
TaskHandle_t xTask1, xTask2, xADCTask;



/**************************************************************************
 * Funkcja G³ówna
***************************************************************************/
int main( void )
{
	init();

	TimerHandle_t xTimerLCD, xTimerRTC, xTimerTemp;

	xTaskCreate(vTaskInfo, "info", 60, NULL, 0, NULL);
	xTaskCreate(vTask2, "task2", 100, NULL, 1, &xTask2);
	xTaskCreate(vTask1, "task1", 100, NULL, 1, &xTask1);
	xTaskCreate(vADCTask, "adc", 100, NULL, 1, &xADCTask);

	xTimerRTC = xTimerCreate("rtc",1000,pdTRUE,0,prvRtcTimerCallback);
	xTimerLCD = xTimerCreate("lcd",20,pdTRUE,0,prv10msTimerCallback);
	xTimerTemp = xTimerCreate("temp", 1000, pdTRUE, 0, prvTempTimerCallback);
	xTimerStart(xTimerLCD, 0);
	xTimerStart(xTimerRTC, 0);
	xTimerStart(xTimerTemp, 0);

	xMutexUart = xSemaphoreCreateMutex();


	vTaskStartScheduler();

	while(1);
	return 0;
}




/**************************************************************************
 * Menu
***************************************************************************/
void vTask2(void *pvParameters)
{
	for(;;)
	{
		/*
		xSemaphoreTake(xMutexUart, portMAX_DELAY);
		{
			uart_putsln("---------------------------");
		}
		xSemaphoreGive(xMutexUart);
		*/

		ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
			uart_putsln("---------------------------");


		//vTaskDelay(500);
	}
}

void vTask1(void *pvParameters)
{
	for(;;)
	{
		/*
		xSemaphoreTake(xMutexUart, portMAX_DELAY);
		{
			uart_putsln("***************************");
		}
		xSemaphoreGive(xMutexUart);
		*/
		uart_putsln("***************************");

		xTaskNotifyGive(xTask2);
		xTaskNotifyGive(xTask2);
		xTaskNotifyGive(xTask2);

		vTaskDelay(1000);
	}
}



/**************************************************************************
 * Timer 10ms, odœwierzanie ekranu LCD, odczytywanie stanu klawiszy
***************************************************************************/
static void prv10msTimerCallback(TimerHandle_t xTimer)
{
	lcd_display();
	KeybProc();
}




/**************************************************************************
 * Odczyt, wyœwietlenie danych RTC
***************************************************************************/
static void prvRtcTimerCallback(TimerHandle_t xTimer)
{
	DS1307_ReadData(&DS1307_data);

	lcd_write(DS1307_PrintDay(&DS1307_data),0,0);
	lcd_write(DS1307_PrintTime(&DS1307_data),0,4);
	lcd_write(DS1307_PrintDate(&DS1307_data),0,15);
}



/**************************************************************************
 * Odczyt, wyœwietlenie danych temperatury
***************************************************************************/
static void prvTempTimerCallback(TimerHandle_t xTimer)
{
	uint8_t ulExectionCount;

	ulExectionCount = (uint8_t) pvTimerGetTimerID(xTimer);
	ulExectionCount++;
	vTimerSetTimerID(xTimer, (void*) ulExectionCount);

	if(ulExectionCount%2)	DS18X20_start_meas(DS18X20_POWER_EXTERN, NULL);
	else					DS18X20_read_meas(gSensorIDs[0], &DS18X20_data);

	lcd_write("Temp:", 3,10);
	lcd_write(DS18X20_print_temp(&DS18X20_data),3,15);

}



/**************************************************************************
 * Informacje programowe
***************************************************************************/
void vTaskInfo(void *pvParameters)
{
	for(;;)
	{
		xFreeHeapSize = xPortGetFreeHeapSize();
		lcd_write_int(xFreeHeapSize,3,0,0);

		vTaskDelay(1000);
	}
}


void vADCTask(void *pvParameters)
{
	uint32_t usAdc;
	BaseType_t xResult;

	for(;;)
	{
		xResult = xTaskNotifyWait(0,0, &usAdc, portMAX_DELAY);

		//if(xResult == pdPASS)
		//{
			lcd_write_int(usAdc,1,0,0);

			static int x;
			x++;
			lcd_write_int(x,2,0,0);
		//}

	}
}


ISR(ADC_vect)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint16_t usResult;

	usResult = ADCW;

	lcd_write_int(usResult,1,10,0);

	xTaskNotifyFromISR(	xADCTask,
						usResult,
						eSetValueWithOverwrite,
						&xHigherPriorityTaskWoken);
}


/*
void vApplicationIdleHook(void)
{

}
*/









