/*
 * MotorControl.c
 *
 * Created: 19-04-2012 3:17:27 PM
 *  Author: Arijit Dasgupta
 */ 

//Takes characters from serial port, I to initialize and de-initialize the motor and also to cool off the motor... R, L tells the system to move
//left and right and as a confirmation sends a C after rotation.

#define F_CPU 8000000L
#define BAUD 9600 //Baud rate
#define MYUBRR F_CPU/16/BAUD-1 //UBRR

#define MAXROT 200 //Max step in each direction

#define ledon PORTD |= (1<<7)
#define ledoff PORTD &= ~(1<<7)
#define ledtoggle PORTD ^= (1<<7)
#define cooloff PORTC = 0b0;

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

void init();
void rotate(int); //Rotation function, will input arg 0 = 3 for rotation
void init_uart();
void UART_write(char x);
uint8_t UART_read();

int counter = MAXROT/2, rot = 0;//Initializing the rotation parameters
uint8_t init_flag = 0; //System initialization flag

int main(void)
{
    init();
	sei();
	sleep_enable();
	while(1){
		sleep_mode(); //Setting the microcontroller to sleep
    }
}

void init(){
	init_uart();
	DDRD = (1<<7); //Configuring the LED port as output
	DDRC = (0b1111); //Configuring the motor control ports as output
}

void init_uart(){
	UBRRH = (MYUBRR) >> 8;
    UBRRL = MYUBRR; //Set the Baud rate
    UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE); //Rx Tx enabled in UART and RX complete interrupts enable
    UCSRC = (1<<URSEL)|(3<<UCSZ0); //Setting 9600-N-1 with URSEL for setting bits
}

void UART_write(char x){
	if(x == '\n')
		x = '\r';
	while(!(UCSRA & (1<<UDRE)));
	UDR = x;
}

uint8_t UART_read(){
	uint8_t x;
	while(!(UCSRA & (1<<RXC)));
	x = UDR;
	return x;
}

void rotate_right(){
	if(counter > 0){
		counter--;
		rot = counter % 4;
		PORTC = (0b1111) & (1<<rot);
	}
}

void rotate_left(){
	if(counter < MAXROT){
		counter++;
		rot = counter % 4;
		PORTC = (0b1111) & (1<<rot);
	}
}

ISR(USART_RXC_vect){ //Interrupt routine for rotation and flag setting
	cli();
	char x;
	x = UDR;
	if(x == 'i' || x == 'I'){
		init_flag ^= (1<<0);
		if(init_flag == 0){
			UART_write('i');
			cooloff;
		}
		else{
			UART_write('I');
			cooloff;
		}
	}
	else if((x == 'r' || x == 'R') && init_flag != 0){
		rotate_right();
		if(counter == 0){
			UART_write('F');
		}
		else{
			UART_write('C');	
		}
	}
	else if((x == 'l' || x == 'L') && init_flag != 0){
		rotate_left();
		if(counter == MAXROT){
			UART_write('F');
		}
		else{
			UART_write('C');	
		}
	}
	ledtoggle;
	sei();
}