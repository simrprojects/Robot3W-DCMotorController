/*
 * L6206.h
 *
 *  Created on: 28.02.2020
 *      Author: Przemek
 */

#ifndef L6206_H_
#define L6206_H_

void L6206_init(void);
void L6206_enable(int ch_A, int ch_B);
void L6206_setDuty(signed int duty_A,signed int duty_B);

#endif /* L6206_H_ */
