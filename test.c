/*
 * test.c
 *
 * Created: 10.08.2017
 * Author : igr
 */ 

#include <avr/io.h>

#define  F_CPU 8000000
#include <util/delay.h>
//---------------------------------------------------------
#define FAULT (PINB & (1<<0))
#define DIR_R (PORTB |= (1<<1))
#define DIR_L (PORTB &= ~(1<<1))
#define LEFT	  0
#define RIGHT	  1
#define SPEED	100			// 0 - 255  (8-bit T/C)

#define analog_max	153		// Vref = 1V (10bit ADC - 1024 // 1 = 0.00097V) 
							//0.5V = 512 / 30% = 153
							//1V = 1024 / 30% = 307 
//---------------------------------------------------------
void init(void);
unsigned char turn(unsigned char direction, unsigned char speed);
unsigned int get_average(void);
//---------------------------------------------------------
int main(void)
{
    init();
	
    while (1) 
    {
		//---example of control signal
		while (!FAULT)
		{
			if (get_average() > analog_max)
			{
				turn(LEFT, SPEED);
			}//if
		}//while
		
    }//while(1)
}//main
//---------------------------------------------------------
void init(void)
{
	//---ports
	DDRB |= ( (1<<1) | (1<<3) );  //--DIR, PWM - out //MAX14870
	
	DDRB &= ~(1<<0); //--FAULT in //MAX14870
	PORTB |= (1<<0); //---pull_up
	
	DDRC &= (1<<0);	//ADC0 - analog signal in
	
	//---ADC
	ADCSRA = 0xC6;
	
	//---T/C2
	ASSR=0x00;
	
	//fast PWM
	//Fpwm(max) = 50kHz for MAX14870 --> prescaler = 256, Fpwm = 31250Hz
	//non-inverting PWM mode 
	TCCR2=(1<<WGM20)|(1<<WGM21)|(1<<COM21)|(0<<COM20)|(1<<CS22)|(1<<CS21)|(0<<CS20);	
	
	TCNT2=0x00;
	OCR2=0x00;	//out = 0V
	TIMSK=0x00;	//overflow interrupt disable
	
	return;
}//init()
//---------------------------------------------------------
unsigned char turn(unsigned char direction, unsigned char speed)
{
	if (direction) DIR_R;
	else DIR_L;
	
	//cli();
	OCR2 = speed;
	//sei();
	
	return FAULT;
}//turn()
//--------------------------------------------------------
unsigned int get_average(void)
{
	static unsigned int avrg_summ;
	
	static unsigned int adc_res[64];
	static unsigned char cntr;
	
	//average calculation
	
	for (unsigned char i=0; i<64; i++)
	{
		avrg_summ += adc_res[i];
	}//for
	
	avrg_summ -= adc_res[cntr];
	
	//---read new value from ADC
	ADMUX = ( 0x00 | (ADMUX & 0xF0) ); // REFS1:REFS0	0:0	/AREF
	// ADLAR(bit 5) 0	/ADCH <- 2 high bit
	_delay_us(15);
	ADCSRA |= 0x40;							// ADSC = 1
	while ( 0 == (ADCSRA & 0x10) );			// while (ADIF == 0)
	adc_res[cntr] = ( (unsigned int) (ADCL + (ADCH<<8)) );
	avrg_summ += adc_res[cntr];
	
	if (cntr < 62) cntr++;
	else cntr = 0;
	
	return avrg_summ/64;
}//get_average()
//-------------------------------------------------------
