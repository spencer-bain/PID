//Written by Spencer Bain
/**************************************************************************************
INTRODUCTION
I am brain storming some idea's for code to control a system that consistes of two motors
that will have propelers on them to give the system thrust. It will essentially be a 
teeter totter(see-saw) that has a propeller on either end. I would like this code to
quite literaly balance the system.

HARDWARE:
2 ESC's (blheli 20 Amps little bee's)
1 Microcontroller (Atmega128)
2 Brushless DC moters
2 3D printed props
1 stick/beam

*************************************************************************************/



/**************************************************************************************
PID CODE
PID is proportional, Intergrate, and Divivative controls.
The idea is that you have system that you want to control a certain variable. For
example an position of a motor output shaft, a temperature of a house, the moister
level of soil in a garden, and etc. The input also known as Setpoint, is the "value"
that the system should converge on if the system is setup correctly. The systems inputs
should corrisond to the desiered output. That is if the set point of a system that is
controlling the angle of a motors output shaft is at pi(radians) or 180(degrees), the
output that is the motor shaft should begin moving to pi(radians), 180(degrees) in 
the refrence frame. Input is X, output Y should converge onto X.

Since this is being coded mostly sequentially coding the PID control flow can be 
explained sequentially.
            
                          _______________________
                         |                       |
                     /-->|P      K_p*e(t)        |_______
                     |   |_______________________|       \   control
         ___   error |    _______________________       _V_+ signal  _______  Output
Input  +/   \  e(t)  |   |                       |   + /   \  u(t)  |       |  y(t)     
x(t)-->| sum |---------->|I  K_i*intergal(e(t))  |--->| sum |------>| plant |------->
        \___/        |   |_______________________|     \___/        |_______|   |
          ^ -        |    _______________________        ^+                     |
          |          |   |                       |       |                      |
          |          \-->|D  K_d*derivative(e(t))|_______/                      |
          |              |_______________________|                              |
          \_____________________________________________________________________/

*************************************************************************************/

/*
PIN24/ADC1/PC1 	signal/meassurment/poteniometer
PIN25/ADC2/PC2 	signal/setpoint/poteniometer

PIN11/OC0B/PD5 	A1
PIN12/OC0A/PD6	A2
PIN13/PD7		EN

PIN16/PB2		led
*/

#include"pid.c"
#include <avr/io.h>
#include <avr/interrupt.h>

void timer0_init(void){
	TCCR0A |= (1<<COM0A1);//In Fast PWM, clear OCRA on match, set OCRA at BOTTOM
	TCCR0A |= (1<<COM0B1);//In Fast PWM, clear OCRB on match, set OCRB at BOTTOM
	TCCR0A |= (1<<WGM01)|(1<<WGM00);//Fast PWM

	TCCR0B |= (1<<CS00);//no prescaler

	OCR0A = 0x00;
	OCR0B = 0x00;
}

void timer2_init(void){
	TCCR2B |= (1<<CS20)|(1<<CS21)|(1<<CS22);//1024 prescaler
	OCR2A = 0x08;	//OCR2A*precaler how many clock cycles till
					//the OC2A flag is set
	TIMSK2 |= (1<<OCIE2A);
	TIFR2  |= (1<<OCF2A);
}

void updating_timer0(uint8_t comA, uint8_t comB){
	OCR0A = comA;
	OCR0B = comB;
}

void send_control_signal(int16_t control_signal){
	uint8_t pwm_signal = (control_signal>>2);
	
	if(control_signal < 0){
		PORTD |= (1<<PD7)|(1<<PD5);
		PORTD &= ~(1<<PD6);
		updating_timer0(0x00,pwm_signal);
	}else if(control_signal > 0){
		PORTD |= (1<<PD7)|(1<<PD6);
		PORTD &= ~(1<<PD5);
		updating_timer0(pwm_signal,0x00);
	}else{
		PORTD &= ~(1<<PD7);//turn off motor
	}
}

volatile uint16_t adc1_data;
volatile uint16_t adc2_data;
volatile struct Pid *pid;

ISR(TIMER2_COMPA_vect){
	int16_t control_signal = update_control_signal(pid,adc2_data);
	send_control_signal(control_signal);
	PORTB |= (1<<PB2);
}

int main(){
	uint8_t count = 0;
	DDRD |= (1<<PD5)|(1<<PD6)|(1<<PD7);
	
	DDRC  &= ~((1<<PC2)|(1<<PC1)); //make port C bit 1 the ADC input  
	PORTC &= ~((1<<PC2)|(1<<PC1));  //port C bit 1 pullups must be off

	DDRB |= (1<<PB2);
	timer0_init();
	timer2_init();
	sei();

	uint16_t kd = 0;
	uint16_t kp = 16;
	uint16_t ki = 0;

	int16_t setpoint = (1<<9);//2^10 for the ADC, setpoint is at half 

	set_gains(pid,kd,kp,ki);
	set_setpoint(pid,setpoint);
	set_time_step(pid,OCR2A*1024);//1024 is the prescaler for timer2


	ADMUX = (0<<ADLAR)|(0<<REFS1)|(1<<REFS0)|(1<<MUX0)|(0<<MUX1)|(0<<MUX2);//single-ended input, PORTF bit 7, right adjusted, 10 bits
	//reference is AVCC

	ADCSRA =  (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);//ADC enabled, don't start yet, single shot mode 
	//division factor is 128 (125khz)
	while(1){
		set_setpoint(pid,adc1_data);
		
		ADCSRA |=  (1<<ADSC);//(1<<ADFR);//poke the ADSC bit and start conversion

		while(bit_is_clear(ADCSRA,ADIF)){}       //spin while interrupt flag not set

		ADCSRA |= (1<<ADIF);                   //its done, clear flag by writing a one 
		
		count++;
		if(count%2==0){
			adc1_data = ADC;	//read the ADC2 output as 16 bits
			
			//switching ADC's
			ADMUX |= (1<<MUX0);
			ADMUX &= ~(1<<MUX1);
		}
		else{
			adc2_data = ADC;	//read the ADC1 output as 16 bits
			ADMUX |= (1<<MUX1);
			ADMUX &= ~(1<<MUX0);
		}
		//if(adc1_data > 512){PORTB |= (1<<PB2);}
		//else{PORTB &= ~(1<<PB2);}
	}//while
	return 0;
}//main

