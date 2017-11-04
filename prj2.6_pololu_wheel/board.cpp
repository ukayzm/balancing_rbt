#include "Arduino.h"
#include "board.h"

int16_t count_m0, count_m1;
unsigned long total_count_m0, total_count_m1;

void on_intr_m0(void)
{
	if (digitalRead(M0_DIR_PIN))
		count_m0++;
	else
		count_m0--;
	total_count_m0++;
}

void on_intr_m1(void)
{
	if (digitalRead(M1_DIR_PIN))
		count_m1--;
	else
		count_m1++;
	total_count_m1++;
}

void setup_board()
{
	pinMode(M0_INTR_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(M0_INTR_PIN), on_intr_m0, FALLING);
	pinMode(M0_DIR_PIN, INPUT_PULLUP);
	count_m0 = 0;

	pinMode(M1_INTR_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(M1_INTR_PIN), on_intr_m1, FALLING);
	pinMode(M1_DIR_PIN, INPUT_PULLUP);
	count_m1 = 0;
}

