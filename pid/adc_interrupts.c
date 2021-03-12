// Written by: Spencer Bain
// 22-02-21 EU
/*
PIN24/ADC1 	signal/meassurment/poteniometer
PIN11/OC0B 	A1
PIN12/OC0A	A2
PIN13		EN
*/

#include <avr/io.h>

int main(){
	uint8_t count = 0;
	uint16_t adc_result;
	DDRB |= (1<<PB2);
	PORTB &= ~(1<<PB2);
	//Initalize ADC and its ports
	DDRC  &= ~((1<<PC2)|(1<<PC1)); //make port C bit 1 the ADC input  
	PORTC &= ~((1<<PC2)|(1<<PC1));  //port C bit 1 pullups must be off

	ADMUX = (0<<ADLAR)|(0<<REFS1)|(1<<REFS0)|(1<<MUX0)|(0<<MUX1)|(0<<MUX2);//single-ended input, PORTF bit 7, right adjusted, 10 bits
	//reference is AVCC

	ADCSRA =  (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);//ADC enabled, don't start yet, single shot mode 
	//division factor is 128 (125khz)
	while(1){
		ADCSRA |=  (1<<ADSC);//(1<<ADFR);//poke the ADSC bit and start conversion

		while(bit_is_clear(ADCSRA,ADIF)){}       //spin while interrupt flag not set

		ADCSRA |= (1<<ADIF);                   //its done, clear flag by writing a one 


		count++;
		if(count%2==0){
			adc_result = ADC;	//read the ADC2 output as 16 bits
			
			//switching ADC's
			ADMUX |= (1<<MUX0);
			ADMUX &= ~(1<<MUX1);
		}
		else{
			//adc_result = ADC;	//read the ADC1 output as 16 bits
			ADMUX |= (1<<MUX1);
			ADMUX &= ~(1<<MUX0);
		}
		if(adc_result > 512){PORTB |= (1<<PB2);}
		else{PORTB &= ~(1<<PB2);}
	}//while
}//main
