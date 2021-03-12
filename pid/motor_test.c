// Written by: Spencer Bain
// 22-02-21 EU
/*
PIN24/ADC1 	signal/meassurment/poteniometer
PIN25/ADC2 	signal/setpoint/poteniometer
PIN11/OC0B/PD5 	A1
PIN12/OC0A/PD6	A2
PIN13/PD7		EN
*/

#include <avr/io.h>
#include <util/delay.h>

int main(){
	uint16_t adc_result;
	DDRB |= (1<<PB2);
	PORTB &= ~(1<<PB2);
	DDRD |= (1<<PD5)|(1<<PD6)|(1<<PD7);
	PORTD &= ~((1<<PD5)|(1<<PD6)|(1<<PD7));

	//Initalize ADC and its ports
	DDRC  &= ~((1<<PC2)|(1<<PC1)); //make port C bit 1 the ADC input  
	PORTC &= ~((1<<PC2)|(1<<PC1));  //port C bit 1 pullups must be off

	ADMUX = (0<<ADLAR)|(0<<REFS1)|(1<<REFS0)|(0<<MUX0)|(1<<MUX1)|(0<<MUX2);//single-ended input, PORTF bit 7, right adjusted, 10 bits
	//reference is AVCC

	ADCSRA =  (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);//ADC enabled, don't start yet, single shot mode 
	//division factor is 128 (125khz)
	while(1){
		ADCSRA |=  (1<<ADSC);//(1<<ADFR);//poke the ADSC bit and start conversion

		while(bit_is_clear(ADCSRA,ADIF)){}       //spin while interrupt flag not set

		ADCSRA |= (1<<ADIF);                   //its done, clear flag by writing a one 

		adc_result = ADC;	//read the ADC2 output as 16 bits
			
		
		if(adc_result > 950){
			PORTD |= (1<<PD5)|(1<<PD7);
			PORTD &= ~(1<<PD6);
			PORTB |= (1<<PB2);
		}
		else if(adc_result < 75){
			PORTD |= (1<<PD6)|(1<<PD7);
			PORTD &= ~(1<<PD5);
			PORTB &= ~(1<<PB2);
		}
		else{
			PORTD &= ~((1<<PD5)|(1<<PD6)|(1<<PD7));
			PORTB ^= (1<<PB2);
			_delay_ms(200);
		}
	}//while
}//main
