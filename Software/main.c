/*
 * main.c
 *
 *  Created on: 2017 Iul 17 16:20:00
 *  Author: Vlad
 */

#include <DAVE.h>
#include <stdlib.h>
#include <stdio.h>

#include <defines.h>

/**
 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point. It is invoked by the device startup code. It is responsible for
 * invoking the App initialization dispatcher routine - DAVE_Init() and hosting the place-holder for user application
 * code.
 */

uint16_t speed;

int16_t i;
int16_t d;
int16_t last_p;

uint16_t best_kp, best_ki, best_kd;
uint16_t best_kp_in_stage, best_ki_in_stage, best_kd_in_stage;
uint16_t kp, ki, kd;

uint8_t warm_flag, initial_run_flag;
uint8_t lap_number, trial_number;
uint32_t samples_since_out, samples_on_finish;

uint32_t current_time = 0, last_cross_time = 0, last_blink_time = 0, button_press_time = 0;
uint32_t best_time, total_time, best_time_in_stage;

uint32_t sampling_timer, sampling_timer_status;
uint32_t ramp_timer, ramp_timer_status;

void updateMotors(void);
int16_t error(void);
int16_t pid(int16_t p);
void motors(int16_t sum, int16_t v);

void initializeParams(void);
void requestPlacement(void);
void startLineFollow(void);
void speedRamp(void);
void warmedUp(void);
void lapComplete(void);
void alterParams(uint8_t trial_number);
void optimumFound();
void stopLineFollow(void);
void uploadData(void);

void delay(uint32_t delay_time);

int main(void)
{
	DAVE_Init();           /* Initialization of DAVE APPs  */

	motors(0, 0);

	sampling_timer = SYSTIMER_CreateTimer(SAMPLING_TIME, SYSTIMER_MODE_PERIODIC, (void*)updateMotors, NULL);
	ramp_timer = SYSTIMER_CreateTimer(RAMP_TIME, SYSTIMER_MODE_PERIODIC, (void*)speedRamp, NULL);

	initializeParams();

	requestPlacement();

	while(1)
 	{
		current_time = SYSTIMER_GetTime();

		if(samples_since_out > ALLOWED_SAMPLES_OUT)
		{
			stopLineFollow();

			if(initial_run_flag != 1)
			{
				trial_number++;
				alterParams(trial_number);
			}

			requestPlacement();
		}

		if(samples_on_finish > CERTAINLY_FINISH_SAMPLES && (current_time - last_cross_time) > VALID_LAP_TIME)
		{
			samples_on_finish = 0;

			if(warm_flag == 1)
			{
				lapComplete();
			}
			else
			{
				warmedUp();
			}
		}
 	}

	return 0;
}

void updateMotors(void)
{
	motors(pid(error()), (int16_t)speed);
}

int16_t error(void)
{
	uint8_t pin_status_1 = DIGITAL_IO_GetInput(&SENSOR_1);
	uint8_t pin_status_2 = DIGITAL_IO_GetInput(&SENSOR_2);
	uint8_t pin_status_3 = DIGITAL_IO_GetInput(&SENSOR_3);
	uint8_t pin_status_4 = DIGITAL_IO_GetInput(&SENSOR_4);
	uint8_t pin_status_5 = DIGITAL_IO_GetInput(&SENSOR_5);
	uint8_t pin_status_6 = DIGITAL_IO_GetInput(&SENSOR_6);
	uint8_t pin_status_7 = DIGITAL_IO_GetInput(&SENSOR_7);
	uint8_t pin_status_8 = DIGITAL_IO_GetInput(&SENSOR_8);

	uint16_t sensor_counter = pin_status_1 + pin_status_2 + pin_status_3 + pin_status_4 + pin_status_5 + pin_status_6 + pin_status_7 + pin_status_8;

	if(sensor_counter != 0)
	{
		samples_since_out = 0;

		int16_t err = pin_status_1 * i1 + pin_status_2 * i2 + pin_status_3 * i3 + pin_status_4 * i4 + pin_status_5 * i5 + pin_status_6 * i6 + pin_status_7 * i7 + pin_status_8 * i8;

		err = err * 100 / sensor_counter;

		if(sensor_counter == 8)
		{
			samples_on_finish++;

			err = 0;
		}
		else
		{
			samples_on_finish = 0;
		}

		return err;
	}
	else
	{
		samples_since_out++;

		return last_p;
	}
}

int16_t pid(int16_t p)
{
	if(abs(i + p) < MAXI)
	{
		i = i + p;
	}

	d = p - last_p;
	last_p = p;

	int32_t sum = (p * kp + i * ki + d * kd) / 1000;

	return (int16_t)sum;
}

void motors(int16_t sum, int16_t v)
{
	if(sum < 0)
	{
		DIGITAL_IO_SetOutputLow(&DIGITAL_R);
		PWM_SetDutyCycle(&PWM_R, v);

		if(v + sum < 0)
		{
			DIGITAL_IO_SetOutputHigh(&DIGITAL_L);
			PWM_SetDutyCycle(&PWM_L, (10000 + v + sum));
		}
		else
		{
			DIGITAL_IO_SetOutputLow(&DIGITAL_L);
			PWM_SetDutyCycle(&PWM_L, (v + sum));
		}
	}
	else
	{
		DIGITAL_IO_SetOutputLow(&DIGITAL_L);
		PWM_SetDutyCycle(&PWM_L, v);

		if(v - sum < 0)
		{
			DIGITAL_IO_SetOutputHigh(&DIGITAL_R);
			PWM_SetDutyCycle(&PWM_R, (10000 + v - sum));
		}
		else
		{
			DIGITAL_IO_SetOutputLow(&DIGITAL_R);
			PWM_SetDutyCycle(&PWM_R, (v - sum));
		}
	}
}

void initializeParams(void)
{
	kp = START_KP;
	ki = START_KI;
	kd = START_KD;

	best_time = 1000000000;
	best_kp = START_KP;
	best_ki = START_KI;
	best_kd = START_KD;

	initial_run_flag = 1;
}

void requestPlacement(void)
{
	warm_flag = 0;
	lap_number = 0;
	samples_since_out = 0;
	samples_on_finish = 0;
	total_time = 0;

	while(DIGITAL_IO_GetInput(&BUTTON) == 0)
	{
		current_time = SYSTIMER_GetTime();

		if(current_time - last_blink_time > BLINK_TIME)
		{
			last_blink_time = current_time;

			DIGITAL_IO_ToggleOutput(&LED_3);
			DIGITAL_IO_ToggleOutput(&LED_4);
			DIGITAL_IO_ToggleOutput(&LED_5);
		}
	}

	DIGITAL_IO_SetOutputLow(&LED_3);
	DIGITAL_IO_SetOutputLow(&LED_4);
	DIGITAL_IO_SetOutputLow(&LED_5);

	button_press_time = current_time;

	while(DIGITAL_IO_GetInput(&BUTTON) == 1)
	{
		current_time = SYSTIMER_GetTime();
	}

	if(current_time - button_press_time > LONG_PRESS_TIME)
	{
		uploadData();

		requestPlacement();
	}
	else
	{
		delay(START_DELAY);

		startLineFollow();
	}
}

void startLineFollow(void)
{
	speed = V_MIN;

	sampling_timer_status = SYSTIMER_StartTimer(sampling_timer);
	ramp_timer_status = SYSTIMER_StartTimer(ramp_timer);
}

void speedRamp(void)
{
	uint16_t ramped_speed;

	ramped_speed = speed + RAMP_TERM;

	if(ramped_speed < V_REF)
	{
		speed = ramped_speed;
	}
	else
	{
		speed = V_REF;

		ramp_timer_status = SYSTIMER_StopTimer(ramp_timer);
	}
}

void warmedUp(void)
{
	warm_flag = 1;

	last_cross_time = current_time;
}

void lapComplete(void)
{
	uint32_t lap_time, average_time;

	lap_time = current_time - last_cross_time;

	last_cross_time = current_time;

	total_time = total_time + lap_time;

	lap_number++;

	if(lap_number == NUMBER_OF_LAPS)
	{
		average_time = total_time / NUMBER_OF_LAPS;

		total_time = 0;

		lap_number = 0;

		if(initial_run_flag == 1)
		{
			best_time = average_time;

			best_time_in_stage = best_time;
			best_kp_in_stage = START_KP;
			best_ki_in_stage = START_KI;
			best_kd_in_stage = START_KD;

			trial_number = 1;
			alterParams(trial_number);

			stopLineFollow();

			requestPlacement();

			initial_run_flag = 0;
		}
		else
		{
			if(average_time < best_time_in_stage)
			{
				best_time_in_stage = average_time;
				best_kp_in_stage = kp;
				best_ki_in_stage = ki;
				best_kd_in_stage = kd;
			}

			trial_number++;

			if(trial_number < 7)
			{
				alterParams(trial_number);
			}
			else
			{
				stopLineFollow();

				if(best_time_in_stage < best_time)
				{
					best_time = best_time_in_stage;
					best_kp = best_kp_in_stage;
					best_ki = best_ki_in_stage;
					best_kd = best_kd_in_stage;

					trial_number = 1;
					alterParams(trial_number);

					requestPlacement();
				}
				else
				{
					optimumFound();
				}
			}
		}
	}
}

void alterParams(uint8_t trial_number)
{
	kp = best_kp;
	ki = best_ki;
	kd = best_kd;

	switch(trial_number)
	{
		case 1:

			kp = kp + ALTER_TERM;

			break;

		case 2:

			kp = kp - ALTER_TERM;

			break;

		case 3:

			ki = ki + ALTER_TERM;

			break;

		case 4:

			ki = ki - ALTER_TERM;

			break;

		case 5:

			kd = kd + ALTER_TERM;

			break;

		case 6:

			kd = kd - ALTER_TERM;

			break;

		default:

			break;
	}
}

void optimumFound(void)
{
	stopLineFollow();

	DIGITAL_IO_SetOutputHigh(&LED_3);
	DIGITAL_IO_SetOutputHigh(&LED_5);

	while(DIGITAL_IO_GetInput(&BUTTON) == 0)
	{
		current_time = SYSTIMER_GetTime();

		if(current_time - last_blink_time > BLINK_TIME)
		{
			last_blink_time = current_time;

			DIGITAL_IO_ToggleOutput(&LED_3);
			DIGITAL_IO_ToggleOutput(&LED_4);
			DIGITAL_IO_ToggleOutput(&LED_5);
		}
	}

	DIGITAL_IO_SetOutputLow(&LED_3);
	DIGITAL_IO_SetOutputLow(&LED_4);
	DIGITAL_IO_SetOutputLow(&LED_5);

	button_press_time = current_time;

	while(DIGITAL_IO_GetInput(&BUTTON) == 1)
	{
		current_time = SYSTIMER_GetTime();
	}

	uploadData();

	optimumFound();
}

void stopLineFollow(void)
{
	sampling_timer_status = SYSTIMER_StopTimer(sampling_timer);

	motors(0, 0);
}

void uploadData(void)
{
	char best_time_string[10], best_kp_string[10], best_ki_string[10], best_kd_string[10];
	char data[100];
	uint16_t leng;

	sprintf(best_time_string, "%d", (int)best_time);
	sprintf(best_kp_string, "%d", (int)best_kp);
	sprintf(best_ki_string, "%d", (int)best_ki);
	sprintf(best_kd_string, "%d", (int)best_kd);

	strcpy(data, "best time = ");
	strcat(data, best_time_string);
	strcat(data, "\r\n");
	strcat(data, "kp = ");
	strcat(data, best_kp_string);
	strcat(data, "\r\n");
	strcat(data, "ki = ");
	strcat(data, best_ki_string);
	strcat(data, "\r\n");
	strcat(data, "kd = ");
	strcat(data, best_kd_string);

	leng = strlen(data);

	UART_Transmit(&UART_0, (uint8_t *)data, leng * sizeof(uint8_t));
}

void delay(uint32_t delay_time)
{
	uint32_t enter_time = current_time;

	while((current_time - enter_time) < delay_time)
	{
		current_time = SYSTIMER_GetTime();
	}
}
