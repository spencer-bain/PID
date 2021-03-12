//Written by Spencer Bain

#include"pid.c"
#include <avr/io.h>
#include <avr/interrupt.h>

void timer2_init(void){
//	TCCR2A = 0x00;
	TCCR2B |= (1<<CS20)|(1<<CS21)|(1<<CS22);//1024 prescaler
	OCR2A = 0x08;	//OCR2A*precaler how many clock cycles till
					//the OC2A flag is set
	//TIMSK2 |= (1<<OCIE2A)|(1<<TOIE2);
	//TIFR2  |= (1<<OCF2A)|(1<<TOV2);
	TIMSK2 |= (1<<OCIE2A);
	TIFR2  |= (1<<OCF2A);
	//TCNT2 	= 0x00; //don't do this, this stops the timmer
}

//The Not GNU website says this timmer vector isn't supported
//try running this and see that it does work!
ISR(TIMER2_COMPA_vect){
	PORTB |= (1<<PB2);
}

int main(){
	DDRB |= (1<<PB2);
	PORTB &= ~(1<<PB2);
	timer2_init();
	sei();
	while(1){
		//PORTB |= (1<<PB2);
	}
	return 0;
}//main

