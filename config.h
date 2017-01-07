/*
 * config.h
 *
 *  Created on: 22 gru 2016
 *      Author: Kacper
 */

#ifndef CONFIG_H_
#define CONFIG_H_


/**************************************************************
 * Makra upraszczaj�ce dost�p do port�w
***************************************************************/
// *** PORT
#define PORT(x) SPORT(x)
#define SPORT(x) (PORT##x)
// *** PIN
#define PIN(x) SPIN(x)
#define SPIN(x) (PIN##x)
// *** DDR
#define DDR(x) SDDR(x)
#define SDDR(x) (DDR##x)


/**************************************************************
 * Wej�cia/wyj�cia
***************************************************************/

#define LED1_Port C
#define LED1_Pin 5

#define LED1_Init() DDR(LED1_Port) |= (1<<LED1_Pin)
#define LED1_On() PORT(LED1_Port) |= (1<<LED1_Pin)
#define LED1_Off() PORT(LED1_Pin) &= ~(1<<LED1_Pin)
#define LED1_Tog() PORT(LED1_Port) ^= (1<<LED1_Pin)


#endif /* CONFIG_H_ */
