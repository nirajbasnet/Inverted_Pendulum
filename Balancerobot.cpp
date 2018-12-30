/*
 * Balancerobot.cpp
 *
 * Created: 2/12/2015 3:42:14 AM
 *  Author: Niraj
 */ 

#define  atMega32
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "header.h"
#include "lcd.h"
#include "I2Cmaster.h"
#include "IMU_MPU6050.h"
#include "PIDgeneral.h"
#include "math.h"
#include "Complementary_filter.h"
#include "Kalman_filter.h"

# define USART_BAUDRATE 38400
# define BAUD_PRESCALE ((( F_CPU / ( USART_BAUDRATE * 16UL))) - 1)
#define pwm_topvalue 2000
#define stop_midvalue 1000
#define kp_increment 1
#define kd_increment 1
#define ki_increment 1
#define pwmtime_increment 1
#define pwmmargin_increment 50
#define offset_increment 10

PID anglePID;
MPU6050 accelgyro;
Complementary com_pitch;
Kalman kal_pitch;

volatile unsigned char pidflag=0;
volatile unsigned char pidcounter=0;
volatile unsigned char displaycounter=0;
volatile int inter_ocrA=pwm_topvalue/2;
volatile int inter_ocrB=pwm_topvalue/2;
volatile float filter_pitch=0;
volatile float pitch_angle=0;
volatile float pitch_rate=0;
volatile float temperature=0;
volatile float motor1_velocity=0;
volatile float motor2_velocity=0;
unsigned int kpvalue=10;
unsigned int kdvalue=0;
unsigned int kivalue=0;
unsigned int pwm_time=7;
unsigned int pwm_margin=100;



void PIDsetup();
void MPU6050setup();
void PWMmotorsetup();
void timer0setup();
void timer2setup();
void initialize_uart();
void map_joystick_input(unsigned char data);
void complementary_filtersetup();
void kalman_setup();

int main(void)
{
	PWMmotorsetup();
	timer0setup();
	timer2setup();
	PIDsetup();
	lcd_init();
	initialize_uart();
	sei();
	MPU6050setup();
	while(1)
	{
		accelgyro.Read_MPU6050();
		accelgyro.Read_raw_accel_angle();
		temperature=accelgyro.readTemperature();
		
		pitch_angle=accelgyro.get_pitchangle();
		pitch_rate=accelgyro.get_pitchrate();
		
		
		if( !( ( UCSRA & (1<<RXC) )==0) )
		{
			map_joystick_input(UDR);
		}
		
		if(pidcounter>=pwm_time)
		{
			pidcounter=0;
			anglePID.SetTargetPoint(-1,FALSE);
			anglePID.SetInput(filter_pitch);
			anglePID.CalculatePID();
			
			inter_ocrA=stop_midvalue+(int)anglePID.getPIDoutput()+(int)motor1_velocity;//+accelgyro.get_pitchrate()*5;
			//inter_ocrA=OCR1A+(int)anglePID.getPIDoutput()+(int)motor1_velocity;
			if(inter_ocrA>(stop_midvalue+pwm_margin))
			inter_ocrA=stop_midvalue+pwm_margin;
			else if(inter_ocrA<(stop_midvalue-pwm_margin))
			inter_ocrA=stop_midvalue-pwm_margin;
			
			inter_ocrB=stop_midvalue+(int)anglePID.getPIDoutput()+(int)motor2_velocity;//+accelgyro.get_pitchrate()*5;
			//inter_ocrB=OCR1B+(int)anglePID.getPIDoutput()+(int)motor2_velocity;
			if(inter_ocrB>(stop_midvalue+pwm_margin))
			inter_ocrB=stop_midvalue+pwm_margin;
			else if(inter_ocrB<(stop_midvalue-pwm_margin))
			inter_ocrB=stop_midvalue-pwm_margin;
			
			OCR1A=inter_ocrA;
			OCR1B=inter_ocrB;
			
		}
		if(displaycounter>=16)
		{
			displaycounter=0;
			lcd_clear();
			Printf("OCA=%d kp=%d",OCR1A,kpvalue);
			Printf("\nFP=%f kd=%d",filter_pitch,kdvalue);
		}
		
	}
}
ISR(TIMER0_COMP_vect)  //10ms loop for angle filter 
{
	filter_pitch=com_pitch.getangle(pitch_angle,pitch_rate,0.01);
	//filter_pitch=kal_pitch.getKalmanAngle(pitch_angle,pitch_rate,0.01);
	
	displaycounter++;
}
ISR(TIMER2_COMP_vect)
{
	pidcounter++;
}

void map_joystick_input(unsigned char data)
{
	switch(data)
	{
		case 'L':kpvalue=kpvalue+kp_increment;
		         break;
		case 'R':kpvalue=kpvalue-kp_increment;
		         break;
	    case 'P':kdvalue=kdvalue+kd_increment;
		         break;
		case 'O':kdvalue=kdvalue-kd_increment;
		         break;
		case 'F':kivalue=kivalue+ki_increment;
		         break;
		case 'B':kivalue=kivalue-ki_increment;
		         break;
	    case 'M':pwm_time=pwm_time+pwmtime_increment;
		         break;
		case 'N':pwm_time=pwm_time-pwmtime_increment;
		         break;
		case 'U':pwm_margin=pwm_margin+pwmmargin_increment;
		         break;
		case 'V':pwm_margin=pwm_margin-pwmmargin_increment;
		         break;
		case 'W':anglePID.SetOffset(150+offset_increment);
		         break;
		case 'X':anglePID.SetOffset(150-offset_increment);
		         break;		 
	}
	anglePID.SetTuningConstants(kpvalue,kivalue,kdvalue);
	/*int x_velocity=0, y_velocity=0;
	float kx=1;
	float ky=1;

	if(data<100)
	{
		x_velocity = data-50;
	}
	else if (data>100 && data<200)
	{
		y_velocity = 150-data;
	}
	motor1_velocity=y_velocity*ky+x_velocity*kx;
	motor2_velocity=y_velocity*ky+x_velocity*kx;
	*/
}
void PIDsetup()
{
	anglePID.PIDinitialize();
	anglePID.SetSamplefrequency(1);
	anglePID.SetOutputLimits(-800,800);
	anglePID.SetIntegralLimits(TRUE,-50,50);
	anglePID.SetTuningConstants(5,0,0); 
}
void MPU6050setup()
{
	accelgyro.initialize();
	accelgyro.SetAccelRange(MPU6050_ACCEL_FS_4);
	accelgyro.SetGyroRange(MPU6050_GYRO_FS_250);
}
void complementary_filtersetup()
{
	com_pitch.setAngle(0);
	com_pitch.set_firstorder_tune(0.97);
	
}
void kalman_setup()
{
	kal_pitch.setAngle(0);
	kal_pitch.setQangle(0.001);
	kal_pitch.setQbias(0.003);
	kal_pitch.setRmeasure(0.03);
	
}
void PWMmotorsetup()
{
	DDRD|=(1<<PIND4)|(1<<PIND5);
	DDRD|=(1<<PIND2)|(1<<PIND3)|(1<<PIND7)|(1<<PIND6);
	PORTD|=(1<<PIND2)|(1<<PIND7);
	PORTD&=~((1<<PIND3)|(1<<PIND6));
	
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
	TCCR1B|=(1<<WGM12)|(1<<WGM13)|(1<<CS10);     //1 PRESCALER
	ICR1=pwm_topvalue;
	OCR1A=stop_midvalue;
	OCR1B=stop_midvalue;
	
	
}

void timer0setup()   //10 ms timer1
{
	TCCR0|=(1<<WGM01)|(1<<CS02)|(1<<CS00);   //1024 prescaler
	TIMSK|=(1<<OCIE0);
	//TIFR|=(1<<OCF0);
	OCR0=154;
}
void timer2setup()  //7 ms timer 2
{
	TCCR2|=(1<<WGM21)|(1<<CS22)|(1<<CS20);   //1024 prescaler
	TIMSK|=(1<<OCIE2);
	//TIFR|=(1<<OCF);
	OCR2=108;
}
void initialize_uart()
{
		INPUT(RX_PIN);
		OUTPUT(TX_PIN);
		REGISTER_SET2( UCSRB, RXEN, TXEN );
		REGISTER_SET3( UCSRC, UCSZ1, UCSZ0, URSEL );
		UBRRH=(BAUD_PRESCALE>>8);
		UBRRL=BAUD_PRESCALE;
}
