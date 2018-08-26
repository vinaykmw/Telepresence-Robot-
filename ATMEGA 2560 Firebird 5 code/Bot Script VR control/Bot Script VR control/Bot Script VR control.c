/*
  * Team Id: 1228
  * Author List: Srishtee Jain, Vinay Khushwaha, Vikash Kumar, Varun kumar Singh
  * Filename: CODE_5
  * Theme: Harvester Bot
  * Functions: void motion_pin_config(), void buzzer_pin_config(), void lcd_pin_config(),void servo1_pin_config (),void servo2_pin_config (),
               void servo3_pin_config (), void left_encoder_pin_config(), void right_encoder_pin_config(), void adc_pin_config(),
               void port_init(), void left_position_encoder_interrupt_init(), void right_position_encoder_interrupt_init(), 
               void motion_set(unsigned char), void forward(), void left_(), void right_(), void stop(), void left_degree(unsigned int), void right_degree(unsigned int), 
               void angle_rotate(unsigned int), void check_l(), void setangle(unsigned int), void adc_init(), char ADC_Conversion(unsigned char), void buzzer_on(), 
               void buzzer_off(), void timer5_init(), void timer1_init(), void USART_Init(), void USART_Transmit(unsigned char), char USART_Receive(),
               void velocity(unsigned char, unsigned char), void linear_distance_mm(unsigned int), void forward_mm(unsigned int), void init_devices(),
               void pluck(), void deposit(), void print_sensor(char, char, unsigned char), void move(), void moveEast(), void moveWest(),
               void moveNorth(), void moveSouth(), void matrixInitialise(int, int), int inrange(int, int), struct node isValid_and_smallg(),
               void find_array_path(int, int), Deposition(int), void TreePosition(int), void PointsLocator(int, int), void findDirection(int, int),
               void servo_1(unsigned char), void servo_2(unsigned char ), void servo_3(unsigned char ),void checkPhase1(), void checkPhase2(),
               void move1(),void Initialize_white_sensors(),void soft_left(), void soft_right(), void soft_left(), void soft_right(), void soft_left_degree(unsigned int),
               void soft_right_degree(unsigned int), main()
  * Global Variables: ShaftCountLeft, ShaftCountRight, Degrees, ADC_Value, flag, Left_white_line, Center_white_line, Right_white_line, 
                      visited[7][7], cx, cy, ca=0, fx, fy, Tx[3], Ty[3], Dx[3], Dy[3],check, struct node, nodeCount=0, struct node matrix[7][7],plck
  */


#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.h"
#include <stdlib.h>

/*******************************************GLOBAL VARIABLES************************************************/

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning

unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line  = 0;
unsigned char Right_white_line = 0;


int plck;
int visited[7][7];
int cx=0,cy=0;    //current coordinates(x,y) of bot
signed ca=0;    //initial current angle=0
int fx,fy;        // final coordinates of bot
int Tx[3];        //tree x-coordinates array
int Ty[3];        //tree y-coordinates array
int Dx[3];        //Deposition zone x-coordinate array
int Dy[3];        //Deposition zone y-coordinate array
int check;      //variable to check the error in rotation
//initializes an array which contains the tree positions as 0 
 struct node{
     int x,y,nodeNo;        //    int parentX,parentY;
     int parentNo;            //    node parent;
     int g;                    //variable to solve the grid via shortest path
     int visited;            // 1 is visited
     int obstacle;            // 1 is obstacle
     int path; 
     float displacement;
 };
 int nodeCount=0; //initial node number of bot
 struct node matrix[7][7]; //matrix of type node
 
/**************************************************************************************************************/
/*************************************FUNCTIONs DECLARATION*****************************************************/
void motion_pin_config();
void buzzer_pin_config();
void lcd_pin_config();
void servo1_pin_config ();
void servo2_pin_config ();
void servo3_pin_config ();
void left_encoder_pin_config();
void right_encoder_pin_config();
void adc_pin_config();
void port_init();
void left_position_encoder_interrupt_init();
void right_position_encoder_interrupt_init();
void motion_set(unsigned char);
void forward(); 
void left_();
void right_();
void soft_left();
void soft_right();
void stop();
void left_degrees(unsigned int);
void right_degrees(unsigned int);
void soft_left_degree(unsigned int);
void soft_right_degree(unsigned int);
void angle_rotate(unsigned int);
void check_l();
void setangle(int);
void adc_init();
unsigned char ADC_Conversion(unsigned char);
void buzzer_on();
void buzzer_off();
void timer5_init();
void timer1_init();
void Initialize_white_sensors();
void USART_Init();
void USART_Transmit(unsigned char);
unsigned char USART_Receive();
void velocity(unsigned char, unsigned char);
void linear_distance_mm(unsigned int);
void forward_mm(unsigned int);
void init_devices();
void pluck();
void deposit();
void print_sensor(char, char, unsigned char);
void move();
void move1();
void moveEast();
void moveWest();
void moveNorth();
void moveSouth();
void matrixInitialise(int, int);
int inrange(int, int);
struct node isValid_and_smallg();
void find_array_path(int, int);
void Deposition(int);
void TreePosition(int);
void PointsLocator(int, int);
void findDirection(int, int);
void servo_1(unsigned char);
void servo_2(unsigned char);
void servo_3(unsigned char);
void checkPhase1();
void checkPhase2();
/*****************************************************************************************************************/
/*******************************CONFIGURE ALL THE PINS THAT WILL BE REQUIRED TO IMPLEMENT THE THEME*************************************/    
/*
 * Function Name: motion_pin_configure
 * Description: Enables the pin controlling motors
 * Example Call: motion_pin_config()
 */
void motion_pin_config (void)
{
    DDRA = DDRA | 0x0F;
    PORTA = PORTA & 0xF0;
    DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
    PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*
 * Function Name: buzzer_pin_config
 * Description: Enable buzzer pin
 * Example Call: buzzer_pin_config()
 */
void buzzer_pin_config (void)
{
    DDRC = DDRC | 0x08;        //Setting PORTC 3 as output
    PORTC = PORTC & 0xF7;        //Setting PORTC 3 logic low to turnoff buzzer
}

/*
 * Function Name: lcd_port_config
 * Description: Enables lcd pins
 * Example Call: lcd_port_config()
 */
void lcd_port_config (void)
{
    DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
    PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

/*
 * Function Name: servo1_pin_config
 * Description: Configure PORTB 5 pin for servo motor 1 operation
 * Example Call: servo1_pin_config()
 */
void servo1_pin_config (void)
{
    DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
    PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

/*
 * Function Name: servo2_pin_config
 * Description: Configure PORTB 6 pin for servo motor 2 operation
 * Example Call: servo2_pin_config()
 */
void servo2_pin_config (void)
{
    DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
    PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}
/*
 * Function Name: servo3_pin_config
 * Description: Configure PORTB 7 pin for servo motor 3 operation
 * Example Call: servo3_pin_config()
 */
void servo3_pin_config (void)
{
    DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
    PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}
/*
 * Function Name: left_encoder_pin_config
 * Description: Enables left motor position encoder pins
 * Example Call: left_encoder_pin_config ()
 */
void left_encoder_pin_config (void)
{
    DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input //0b11101111
    PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin//0b00010000
}
/*
 * Function Name: right_encoder_pin_config
 * Description: Enables right motor position encoder pins
 * Example Call: right_encoder_pin_config ()
 */
void right_encoder_pin_config (void)
{
    DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input //0b10111111
    PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}
/*
 * Function Name: adc_pin_config
 * Description: Enables adc pins of atmega2560
 * Example Call: adc_pin_config()
 */
void adc_pin_config (void)
{
    DDRF = 0x00;
    PORTF = 0x00;
    DDRK = 0x00;
    PORTK = 0x00;
}

void port_init()
{
    motion_pin_config(); //robot motion pins config
    adc_pin_config(); //adc pins config
    left_encoder_pin_config(); //left encoder pin config
    right_encoder_pin_config(); //right encoder pin config
    lcd_port_config (); //lcd pins config
    servo3_pin_config();   //servo 3 pins config
    servo2_pin_config();   //servo 2 pins config
    servo1_pin_config();   //servo 1 pins config
    buzzer_pin_config(); //buzzer pins config
}

/***************************************END OF THE CONFIGURATIONS OF THE PINS**************************************************/
/***************************************FUNCTIONS TO INITIALIZES THE POSITION ENCODER INTERRUPTS*******************************/
/*
 * Function Name: left_position_encoder_interrupt_init
 * Description: Initializes left motor position encoders
 * Example Call: left_position_encoder_interrupt_init()
 */
void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
    cli(); //Clears the global interrupt
    EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
    EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
    sei();   // Enables the global interrupt
}
/*
 * Function Name: right_position_encoder_interrupt_init
 * Description: Initializes right motor position encoders
 * Example Call: right_position_encoder_interrupt_init()
 */
void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
    cli(); //Clears the global interrupt
    EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
    EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
    sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
    ShaftCountRight++;  //increment right shaft position count
}
//ISR for left position encoder
ISR(INT4_vect)
{
    ShaftCountLeft++;  //increment left shaft position count
}

/*****************************************END OF INITIAZING OF INTERRUPTS****************************************************/
/*
* Function Name: motion_set
* Input: character which specifies the direction
* Output: make the change in the bot as per the direction
* Example Call: motion_set(0x00)
*/
void motion_set (unsigned char Direction)
{
    unsigned char PortARestore = 0;

    Direction &= 0x0F;         // removing upper nibbel for the protection
    PortARestore = PORTA;         // reading the PORTA original status
    PortARestore &= 0xF0;         // making lower direction nibbel to 0
    PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
    PORTA = PortARestore;         // executing the command
}
/*
 * Function Name: forward
 * Output: both wheels move forward with maximum speed
 * Logic: make left and right motor red wire pins high (0110) 
 * Example Call: forward()
 */
void forward (void) 
{
    motion_set(0x06);
} 
/*
 * Function Name: left_
 * Output: Left wheel backward, Right wheel forward
 * Logic: make left motor red wire pin 0 and right motor red wire pin 1 (0101) 
 * Example Call: left_()
 */
void left_ (void)
{
    motion_set(0x05);
}
/*
 * Function Name: right_
 * Output: Right wheel backward, Left wheel forward
 * Logic: make right motor red wire pin 0 and left motor red wire pin 1 (1010) 
 * Example Call: right_()
 */
void right_ (void) 
{
    motion_set(0x0A);
}
/*
 * Function Name: soft_left
 * Output: Left wheel stationary, Right wheel forward
 * Logic: make right motor red wire pin 1 and left motor red wire pin 0 
 * Example Call: soft_left()
 */
void soft_left (void) 
{
    motion_set(0x04);
}
/*
 * Function Name: soft_right
 * Output: Left wheel forward, Right wheel is stationary
 * Logic: make right motor red wire pin 0 and left motor red wire pin 1 (1010) 
 * Example Call: soft_right()
 */
void soft_right (void) 
{
    motion_set(0x02);
}
/*
 * Function Name: stop
 * Output: both wheels stop
 * Logic: 0000 
 * Example Call: stop()
 */
void stop (void)
{
    motion_set(0x00);
}
void back (void) //both wheels backward
{
	motion_set(0x09);
}
//////////////////////////////////////////////////

/////////////////
void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

/**************************************FUNCTIONS TO ROTATE THE ROBOTE**************************************/

/*
 * Function Name: left_degrees
 * Input: degree
 * Output: rotate the by specific angle bot in left direction
 * Logic: first turn left and then move the required degree
 * Example Call: left_degrees(70)
 */
void left_degrees(unsigned int Degrees)
{
    // 88 pulses for 360 degrees rotation 4.090 degrees per count
    left_(); //Turn left
    angle_rotate(Degrees);
}

/*
 * Function Name: right_degrees
 * Input: degree
 * Output: rotate the bot by specific angle in right direction
 * Logic: first turn right and then move the required degree
 * Example Call: right_degrees(70)
 */
void right_degrees(unsigned int Degrees)
{
    // 88 pulses for 360 degrees rotation 4.090 degrees per count
    right_(); //Turn right
    angle_rotate(Degrees);
}
/*
 * Function Name: soft_left_degree
 * Input: degree
 * Output: rotate the bot by specific angle in soft_left direction
 * Logic: first turn soft_left and then move the required degree
 * Example Call: soft_left_degree(70)
 */
void soft_left_degree(unsigned int Degrees)
{
    //176 pulses for 360 degree rotation 2.045 per count
    soft_left();
    Degrees=Degrees*2;
    angle_rotate(Degrees);
}
/*
 * Function Name: soft_right_degree
 * Input: degree
 * Output: rotate the bot by specific angle in soft_right direction
 * Logic: first turn soft_right and then move the required degree
 * Example Call: soft_right_degree(70)
 */
void soft_right_degree(unsigned int Degrees)
{
    //176 pulses for 360 degree rotation 2.045 per count
    soft_right();
    Degrees=Degrees*2;
    angle_rotate(Degrees);
}

 /*
 * Function Name:angle_rotate
 * Input: degree
 * Output: rotate the bot by specific angle
 * Example Call: angle_rotate(70)
 */

void angle_rotate(unsigned int Degrees)
{
    float ReqdShaftCount = 0;
    unsigned long int ReqdShaftCountInt = 0;

    ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
    ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
    ShaftCountRight = 0;
    ShaftCountLeft = 0;

    while (1)
    {
        if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
        break;
    }
    stop(); //Stop robot
}

/*
 * Function Name: check_l
 * Output: correct the error in angle rotation
 * Logic: if Centre_white_line sensor is on white and Left_white_line sensor is on black then it will correct the angle by rotating 5 degrees 
          in left and move 5-5 degree until Centre_white_line is on black till 30 degree and if not found black in between that then it start 
          rotating right 5-5 degree till 60 degree
 * Example Call: check_l()
 */
void check_l()
{
    int l=5;                                    //left_degree error variable
    int r=5;                                    //right_degree error variable
    unsigned int a=5;                    //fix value of angle which will be used to remove the error
    Initialize_white_sensors();
    while(!(Center_white_line>0x10))
    {
        
        
        Initialize_white_sensors();
        
        if(Left_white_line<0x10&&Center_white_line<0x10&&Right_white_line<0x10&&check==1)
        {
            
            
            if(l<90)
            {
                left_degrees(a);
                l=l+5;
                _delay_ms(100);
            }
            else if(r<=100&&l>=90)
            {
                r=r+5;
                right_degrees(a);
                _delay_ms(100);
            }
            
        }
        else if(Left_white_line<0x10&&Center_white_line<0x10&&Right_white_line<0x10&&check==2)
        {
            if(r<90)
            {
                right_degrees(a);
                r=r+5;
                _delay_ms(100);
            }
            else if(l<100&&r>=90)
            {
                l=l+5;
                left_degrees(a);
                _delay_ms(100);
            }
            
        }
        else
        break;
    }
}

/*
 * Function Name: setangle
 * Input: angle in degree
 * Output: set an angle at specified angle from current angle
 * Logic: let current angle is 20 and we want to rotate to 90 assuming north to be 0 degree. Then rotationAngle will be = 90-20=50 degree.
          if rotationAngle is +ve then it will rotate by left_degree else using right_degree
 * Example Call: setangle(70)
 */
void setangle(signed int angle)
{
    signed int rotateAngle=angle-ca;
    ca=ca+rotateAngle;
    lcd_print(1,1,ca,3);

    if (rotateAngle>0)
    {
        if(rotateAngle==270)
        {
            right_degrees(90);
            check=2;
            stop();
            check_l();
        }
        else
        {
            left_degrees(rotateAngle); //Rotate left by 90 degrees
            check=1;
            stop();
            check_l();
        }
        
    }

    else
    {
        if((abs(rotateAngle))==270)
        {
            left_degrees(90);
            check=1;
            stop();
            check_l();
        }
        else
        {
            right_degrees(abs(rotateAngle)); //Rotate right by 90 degrees
            check=2;
            stop();
            check_l();
        }
        
        
    }
    stop();
    _delay_ms(400);
    
    
}
/*******************************************END OF FUNCTIONS FOR ROTATING THE ROBOT********************************************/
/*******************************************FUNCTIONS TO INITIALIZE ADC********************************************************/

void adc_init()  //Function for initiazing adc
{
    ADCSRA = 0x00;
    ADCSRB = 0x00;        //MUX5 = 0
    ADMUX = 0x20;        //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
    ACSR = 0x80;
    ADCSRA = 0x86;        //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
    unsigned char a;            //Variable to store the ADC value
    if(Ch>7)
    {
        ADCSRB = 0x08;
    }
    Ch = Ch & 0x07;
    ADMUX= 0x20| Ch;
    ADCSRA = ADCSRA | 0x40;        //Set start conversion bit
    while((ADCSRA&0x10)==0);    //Wait for conversion to complete
    a=ADCH;
    ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
    ADCSRB = 0x00;
    return a;
}
/***************************************************END OF ADC*********************************************************/

void buzzer_on (void)
{
    unsigned char port_restore = 0;
    port_restore = PINC;
    port_restore = port_restore | 0x08;
    PORTC = port_restore;
}

void buzzer_off (void)
{
    unsigned char port_restore = 0;
    port_restore = PINC;
    port_restore = port_restore & 0xF7;
    PORTC = port_restore;
}
// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz

void timer5_init()
{
    TCCR5B = 0x00;    //Stop
    TCNT5H = 0xFF;    //Counter higher 8-bit value to which OCR5xH value is compared with
    TCNT5L = 0x01;    //Counter lower 8-bit value to which OCR5xH value is compared with
    OCR5AH = 0x00;    //Output compare register high value for Left Motor
    OCR5AL = 0xFF;    //Output compare register low value for Left Motor
    OCR5BH = 0x00;    //Output compare register high value for Right Motor
    OCR5BL = 0xFF;    //Output compare register low value for Right Motor
    OCR5CH = 0x00;    //Output compare register high value for Motor C1
    OCR5CL = 0xFF;    //Output compare register low value for Motor C1
    TCCR5A = 0xA9;    /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
                       For Overriding normal port functionality to OCRnA outputs.
                        {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
    
    TCCR5B = 0x0B;    //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;    //Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;    //Output compare Register high value for servo 1
 OCR1AL = 0xFF;    //Output Compare Register low Value For servo 1
 OCR1BH = 0x03;    //Output compare Register high value for servo 2
 OCR1BL = 0xFF;    //Output Compare Register low Value For servo 2
 OCR1CH = 0x03;    //Output compare Register high value for servo 3
 OCR1CL = 0xFF;    //Output Compare Register low Value For servo 3
 ICR1H  = 0x03;    
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
                     For Overriding normal port functionality to OCRnA outputs.
                  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

/*******************************************FUNCTIONS OF USART FOR COMMUNICATION WITH RASPBERRY PI*************************************/

void USART_Init () {
    UCSR2A=0x00;
    UCSR2B=0x00;
    //Setting baud rate to 9600
    UBRR2L=0x5F;
    UBRR2H=0x00;
    //Enable the receiver and transmitter
    UCSR2B = (1 << RXEN2) | (1 << TXEN2);
    //Set 1 stop bits and data bit length is 8-bit
    UCSR2C = (3 << UCSZ20);
    UCSR2C &= ~(1 << USBS2);
}

void USART_Transmit (unsigned char data) {
    //Wait until the Transmitter is ready
    while (! (UCSR2A & (1 << UDRE2)) );
    //Make the 9th bit 0 for the moment
    UCSR2B &=~(1 << TXB82);
    //If the 9th bit of the data is a 1
    if (data & 0x0100)
    //Set the TXB8 bit to 1
    UCSR2B |= (1 << TXB82);
    //Get that data out here!
    UDR2 = data;
}

unsigned char USART_Receive() {
    while ( !(UCSR2A & (1 << RXC2)) ); //Wait for the RXC to not have 0
    return UDR2; //Get that data out there and back to the main program!
}
/*******************************************END OF USART*******************************************************************************/

//function to set the velocity of left and right motor
void velocity (unsigned char left_motor, unsigned char right_motor)
{
    OCR5AL = (unsigned char)left_motor;
    OCR5BL = (unsigned char)right_motor;
}

//Function used for moving robot forward by specified distance
void linear_distance_mm(unsigned int DistanceInMM)
{
    float ReqdShaftCount = 0;
    unsigned long int ReqdShaftCountInt = 0;

    ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
    ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
    
    ShaftCountRight = 0;
    while(1)
    {
        if(ShaftCountRight > ReqdShaftCountInt)
        {
            break;
        }
    }
    stop(); //Stop robot
}

//function to move linear distance forward in mm
void forward_mm(unsigned int DistanceInMM)
{
    forward();
    linear_distance_mm(DistanceInMM);

}

void init_devices()
{
    cli(); //Clears the global interrupt
    port_init();  //Initializes all the ports
    adc_init();  //Initializes ADC
    timer5_init();  //Initializes timer for motor
    timer1_init(); //Initializes timer for servo
    left_position_encoder_interrupt_init(); //initializes left encoder interrupt
    right_position_encoder_interrupt_init(); //initializes right encoder interrupt
    USART_Init(); //Initialize UART2 for serial communication
    sei();   // Enables the global interrupt
}
 /*
  * Function Name: pluck
  * Output: pluck the fruit 
  * Logic: close the claws of the arm by rotating servo3 with specific angle and sends D to rpi to know the deposition zone of plucked fruit 
  * Example Call: pluck()
  */
void pluck()
{
    char dep=0x36;
    _delay_ms(100);
    servo_3(100);   //to close the claws
    _delay_ms(400);
    if(plck==1)     //to pluck the fruit in checkphase1 position
    {
        servo_1(84);  
        _delay_ms(100);
        servo_2(60);
        _delay_ms(100);
        servo_1(86);
        _delay_ms(100);
        servo_2(67);
        _delay_ms(200);
    }
    else if(plck==2)     //to pluck the fruit in checkphase2 position
    {
        servo_1(100);
        _delay_ms(100);
        servo_2(62);
        _delay_ms(200);
    }
    servo_1(39);     //make the arm to its original position
    _delay_ms(100);
    servo_2(125);
    _delay_ms(100);
    while(!(dep==0x31||dep==0x32||dep==0x33))
    {
    USART_Transmit(68);
    dep=USART_Receive();
    }
    if(dep==0x31)    //for blueberry
    Deposition(0);   
    if(dep==0x32)    //for orange
    Deposition(1);
    if(dep==0x33)    //for apple
    Deposition(2);
}
 /*
  * Function Name:deposit
  * Output: drop the fruits at specific angle in the deposition box
  * Example Call: deposit()
  */
void deposit()
{
    servo_1(74);  //to bent the arm
    _delay_ms(100);
    servo_2(117);
    _delay_ms(100);
    servo_1(91);
    _delay_ms(100);
    servo_2(113);
    _delay_ms(400);
    servo_3(0); //to open the claws
    _delay_ms(300);
    servo_1(74);
    _delay_ms(100);
    servo_2(117);
    _delay_ms(100);
    servo_2(125);
    _delay_ms(200);
    servo_1(39);
    _delay_ms(200);
}
//function to print sensors values on lcd
void print_sensor(char row, char coloumn,unsigned char channel)
{
    ADC_Value = ADC_Conversion(channel);
    lcd_print(row, coloumn, ADC_Value, 3);
}

/******************************************************Functions to move the bot in specific direction******************************************/
 void Initialize_white_sensors()
 {
     Left_white_line = ADC_Conversion(3);    //Getting data of Left WL Sensor
     Center_white_line = ADC_Conversion(2);    //Getting data of Center WL Sensor
     Right_white_line = ADC_Conversion(1);    //Getting data of Right WL Sensor
     
     print_sensor(1,6,3);    //Prints value of White Line Sensor1
     print_sensor(1,10,2);    //Prints Value of White Line Sensor2
     print_sensor(1,14,1);    //Prints Value of White Line Sensor3d
 }
 
 /*
  * Function Name: move
  * Output:  moves robot on black line until a node is detected, robot stops when a node is detected and move 40 mm forward
  * Logic:     if center sensor detect black line robot moves forward, if left sensor is on black robot turn left ,if right sensor is reading black 
             line then turn right.
 
  * Example Call: move();
  */
void move(void)
{
    while(1) 
    {
        Initialize_white_sensors();

        flag=0;

        

        if((Center_white_line>0x016 && Left_white_line>0x014)||(Center_white_line>0x016&&Right_white_line>0x014))
        {
            buzzer_on();
            _delay_ms(100);
            buzzer_off();
            forward_mm(40);
            stop();
            break;
            
        }
        else if (Center_white_line<=0x010 && Right_white_line<=0x010 && Left_white_line<=0x010)
        {
            if (Center_white_line>Left_white_line && Center_white_line>Right_white_line)
            {
                forward();
                velocity(150,150);
                
        
            }
            
            else if (Right_white_line > Center_white_line)
            {
                forward();
                velocity(160,130);
                check=2;
            }
            else if (Left_white_line > Center_white_line)
            {
                forward();
                velocity(130,160);
                check=1;
            }
            else
            {    
            check_l();
            }            
            
            
            
        }
        
        
        else if(Center_white_line<=0x010 && Right_white_line>0x010 && Left_white_line>0x010)
        {
            if (Right_white_line>Left_white_line)
            {
                
                forward();
                velocity(200,170);
            }
            else
            {
                forward();
                velocity(170,200);
            }
        }

        else if(Center_white_line>0x10 && Right_white_line<=0x010 && Left_white_line<=0x010)
        {
            flag=1;
            forward();
            velocity(200,200);
        }

        else if((Center_white_line>0x010 && Right_white_line<=0x10 && Left_white_line>0x010) )
        {
            flag=1;
            forward();
            velocity(170,200);
            check==1;
        }
        else if((Center_white_line<=0x010 && Right_white_line<=0x010 && Left_white_line>0x10))
        {
            flag=1;
            forward();
            velocity(170,200);
            check=1;
        }

        else if((Center_white_line>0x010 && Right_white_line>0x010 && Left_white_line<=0x010) )
        {
            flag=1;
            forward();
            velocity(200,170);
            check=2;
        }
        else if((Center_white_line<=0x010 && Right_white_line>0x010 && Left_white_line<=0x010)  )
        {
            flag=1;
            forward();
            velocity(200,170);
            check=2;
        }

        
    }
}

/*
  * Function Name: move1
  * Output:  (this is for depositing the fruit)moves robot on black line until a node is detected, robot stops when a node is detected  
  * Logic:     if center sensor detect black line robot moves forward, if left sensor is on black robot turn left ,if right sensor is reading black 
             line then turn right.
 
  * Example Call: move1();
  */
void move1(void)
{
    while(1) 
    {
        Initialize_white_sensors();

        flag=0;



        if((Center_white_line>0x016 && Left_white_line>0x014)||(Center_white_line>0x016&&Right_white_line>0x014))
        {
            stop();
            _delay_ms(100);
            break;
            
        }
        if (Center_white_line<=0x010 && Right_white_line<=0x010 && Left_white_line<=0x010)
        {
            if (Center_white_line>Left_white_line && Center_white_line>Right_white_line)
            {
                forward();
                velocity(150,150);
        
            }
            
            if (Right_white_line > Center_white_line)
            {
                forward();
                velocity(150,100);
            }
            if (Left_white_line > Center_white_line)
            {
                forward();
                velocity(100,150);
            }
            else
            check_l();
            
            
        }
        
        
        if(Center_white_line<=0x010 && Right_white_line>0x010 && Left_white_line>0x010)
        {
            if (Right_white_line>Left_white_line)
            {
                
                forward();
                velocity(200,150);
            }
            else
            {
                forward();
                velocity(150,200);
            }
        }

        if(Center_white_line>0x10 && Right_white_line<=0x010 && Left_white_line<=0x010)
        {
            flag=1;
            forward();
            velocity(200,200);
        }

        if((Center_white_line>0x010 && Right_white_line<=0x10 && Left_white_line>0x010) )
        {
            flag=1;
            forward();
            velocity(150,200);
            check=1;
        }
        if((Center_white_line<=0x010 && Right_white_line<=0x010 && Left_white_line>0x10))
        {
            flag=1;
            forward();
            velocity(150,200);
            check=1;
        }

        if((Center_white_line>0x010 && Right_white_line>0x010 && Left_white_line<=0x010) )
        {
            flag=1;
            forward();
            velocity(200,150);
            check=2;
        }
        if((Center_white_line<=0x010 && Right_white_line>0x010 && Left_white_line<=0x010)  )
        {
            flag=1;
            forward();
            velocity(200,150);
            check=2;
        }

        
    }
}
 /*
  * Function Name: moveEast
  * Output:  set direction of bot towards east and call move function to follow line along east until next node detected
  * Example Call: moveEast()
  */
void moveEast()
{
    setangle(270); //Rotate robot towards east i.e. towards East direction
    _delay_ms(500);
    move(); //move the bot till next node is found
    cx++;    //increment of x-coordinate in east direction
    lcd_print(2,1,cx,1);    
    lcd_print(2,3,cy,1);
    _delay_ms(1);
    visited[cx][cy]=1;    //make the traversed node one
}
 /*
   * Function Name: moveWest
   * Output:  set direction of bot towards east and call move function to follow line along east until next node detected
   * Example Call: moveWest()
   */
 void moveWest()
{
    setangle(90); //Rotate robot left by 90 degree i.e. towards West direction
    _delay_ms(500);
    move (); //move the bot till next node is found
    cx--;    //decrement in x-coordinate in West direction
    lcd_print(2,1,cx,1);    
    lcd_print(2,3,cy,1);
    _delay_ms(1);
    visited[cx][cy]=1;  //make the traversed node one
}
 /*
   * Function Name: moveNorth
   * Output:  set direction of bot towards east and call move function to follow line along east until next node detected
   * Example Call: moveNorth()
   */
void moveNorth()
{
    setangle(0); //Rotate robot left by 0 degrees i.e toward north
    _delay_ms(500);
    move (); //move the bot till next node is found
    cy++;  //increment in y-coordinate in North direction
    lcd_print(2,1,cx,1);    
    lcd_print(2,3,cy,1);
    _delay_ms(1);
    visited[cx][cy]=1;  //make the traversed node one

}
 /*
   * Function Name: moveSouth
   * Output:  set direction of bot towards east and call move function to follow line along east until next node detected
   * Example Call: moveSouth()
   */
 void moveSouth()
{
    setangle(180); //Rotate robot left by 180 degrees i.e towards south
    _delay_ms(500);
    move ();  //calling move function to move till next node is not found
    cy--;     //decrement in y-coordinate in south direction
    lcd_print(2,1,cx,1);    
    lcd_print(2,3,cy,1);
    _delay_ms(1);
    visited[cx][cy]=1; //make the traversed node one
}

/***************************************************End of functions that use to move the bot in specific direction*****************************/
/***************************************************Functions to traverse the grid*****************************************************************/
 /*
  * Function Name: matrixInitialise
  * Input:     finalPointX ,finalPointY these are final points where bot has to reach
  * Output:  This function initialize the nodes and calculate G value and displacement of the points, set point as obstacle or validity of point 
  * Logic:     Two for loops are used to access all nodes of matrix and each point is a object of node this matrix keep data of points and help in 
             finding points which are accesible.
  * Example Call: matrixInitialise(5,2);
  */
 void matrixInitialise(int finalPointX,int finalPointY)
 {
     for(int j=0;j<7;j++)
     {
         for(int i=0;i<7;i++)
         {
             matrix[i][j].x=i;
             matrix[i][j].y=j;
             matrix[i][j].nodeNo=nodeCount;
             matrix[i][j].g=abs(finalPointX-i)+abs(finalPointY-j);
             matrix[i][j].obstacle=0;
             matrix[i][j].visited=0;
             matrix[i][j].displacement= (finalPointX-i)*(finalPointX-i)+(finalPointY-j)*(finalPointY-j);
             nodeCount++;
         }
     }
     matrix[Tx[0]][Ty[0]].obstacle=2;
     matrix[Tx[1]][Ty[1]].obstacle=2;
     matrix[Tx[2]][Ty[2]].obstacle=2;
 }
  /*
   * Function Name:    inrange
   * Input:            x,y  these variable are coordinate of point  
   * Output:        output is 1 if obstacle is not present and point is inside matrix and not visited too 
   * Logic:            if value of x and y is between 0 and 7 and obstacle value is 0 and point is not visited then this value is true 
   * Example Call:  inrange(2,5)
   */
 int inrange(int x,int y)
 {
     
     if((x<7&&x>=0)&&(y<7&&y>=0)&&(matrix[x][y].obstacle==0)&&(matrix[x][y].visited==0))
     return 1;//true
     else return 0;
     
 }
  /*
   * Function Name:        isValid_and_smallg
   * Output:            this function return adjacent node which is valid and has smallest g value and displacement value from the destination point 
   * Logic:                takes a point and check points to its east west north south , find valid points from these and return point point with least
                        G value and displacement. points which are out of range are given g value of 200 
   * Example Call:      isValid_and_smallg();
   */
 struct node isValid_and_smallg()
 {    struct node objw,obje,objn,objs;
     struct node ar[4];
     
     if(inrange(cx-1,cy)==1)
     {
         //    cout<<"valid ";
         objw=matrix[cx-1][cy];
         ar[0]=objw;
     }
     if(inrange(cx-1,cy)==0)
     {
         //    cout<<"invalid ";
         objw.g=200;
         objw.displacement=50000;
         ar[0]=objw;
     }
     
     if(inrange(cx+1,cy)==1)
     {
         //    cout<<"valid ";
         obje=matrix[cx+1][cy];
         ar[1]=obje;
     }
     if(inrange(cx+1,cy)==0)
     {
         //    cout<<"invalid ";
         obje.g=200;
         obje.displacement=50000;
         ar[1]=obje;
     }
     
     if(inrange(cx,cy+1)==1)
     {
         //    cout<<"valid ";
         objn=matrix[cx][cy+1];
         ar[2]=objn;
     }
     
     if(inrange(cx,cy+1)==0)
     {
         //    cout<<"invalid ";
         objn.g=200;
         objn.displacement=50000;
         ar[2]=objn;
     }
     
     if(inrange(cx,cy-1)==1)
     {
         //    cout<<"valid ";
         objs=matrix[cx][cy-1];
         ar[3]=objs;
     }
     if(inrange(cx,cy-1)==0)
     {
         //    cout<<"invalid ";
         objs.g=200;
         objs.displacement=50000;
         ar[3]=objs;
     }
     
     
     struct node min=ar[0];

     for(int i=0;i<4;i++)
     {
         if(min.g>=ar[i].g&&min.displacement>=ar[i].displacement)
         {
             
            min=ar[i];
            }
     }
     return min ;
 }
  /*
   * Function Name:        find_array_path
   * Input:                x_,y_ coordinate of final point where bot has to reach
   * Output:            moves bot step by step towards the final position
   * Logic:                this function call isValid_and_samllg() and moves to the point returned by this and find direction to this point by calling
                        findDirection();, after this update matrix point to visited() 
   * Example Call:        find_array_path(2,5);
   */
 void find_array_path(int x_,int y_)
{    
    //cout<<"\n\n *********************************\n\n";
    //int i=0;
    matrixInitialise(x_,y_);
    while(1)
    {    struct node temp;
        temp=isValid_and_smallg();
        findDirection(temp.x,temp.y);
        matrix[temp.x][temp.y].visited=1;
        cx=temp.x,cy=temp.y;
        if(cx==x_&&cy==y_)
        {
            break;
        }
    }
    
}
 /*
 * Function Name: Deposition
 * Input: no of deposition zone (i.e. either 1 for blueberry, 2 for orange, 3 for apple)
 * Output: traverse the bot to the final position and deposit the fruit
 * Logic: it calls find_array_path to find the shortest path to the deposition node and calls move1 function to move ahead and deposit the fruit
           from its right hand side
 * Example Call: Deposition(2)
 */
void Deposition (int i)
{
    Dx[0]=0;Dy[0]=5; //blueberry deposition
    Dx[1]=5; Dy[1]=5; //orange deposition box
    Dx[2]=2;Dy[2]=5; //apple deposition box
    //matrixInitialise(D[i],D[i]);
    find_array_path( Dx[i],Dy[i]);
    setangle(0);
    _delay_ms(500);
    move1();
    deposit();
    setangle (180);
    _delay_ms(100);
    check_l();
    _delay_ms(200);
    move();
    _delay_ms(100);


}
 /*
  * Function Name: TreePosition
  * Input: Tree-number        
  * Output: Define the nodes on which trees are being placed and call point_locator to locate points around tree one by one
  * Example Call: TreePosition(3)
  */ 
 void TreePosition(int i)
 {
     Tx [0] = 2;Ty[0]=1; //apple fruit tree position
     Tx [1] = 1;Ty[1]=4; //blueberry fruit tree position
     Tx [2] = 4;Ty[2]=2; //orange fruit tree position
     PointsLocator(Tx[i],Ty[i]);// calling points locator for tree;
 }
 /*
  * Function Name: PointsLocator
  * Input: Tree position
  * Output: Locate the points around the tree from where the fruits to be plucked and call find_array_path to go to that node if the node 
            is valid i.e node is not outside the grid or any obstacle is not present on that and pluck fruit if it is present in required fruit table. 
  * Logic: It send R everytime it reaches a respective node around tree to rasp pi to start image processing and receives 9 if the fruit is to
           plucked. 
  * Example Call: PointsLocator(2,3)
  */
 void PointsLocator(int Fx,int Fy)
 {
     int X[4];                //x-coordinates around tree from where fruits to be plucked
     int Y[4];                //y-coordinates around tree from where fruits to be plucked
     char data1,data2,data3;
     X [0] = Fx - 1; Y [0] = Fy ;
     X [1] = Fx;     Y [1] = Fy + 1;
     X [2] = Fx + 1; Y [2] = Fy;
     X [3] = Fx;     Y [3] = Fy - 1;
     for (int i = 0; i <= 3; i++)
     {   servo_1(39); //make the arm straight and open
         servo_2(125);
         servo_3(0);  //claw is open
         
         if(matrix[X[i]][Y[i]].obstacle==2||X[i]<0||X[i]>6||Y[i]<0||Y[i]>6)
         stop();
         else
         {
             //in starting make every point traversed by bot 0 to reach the new node around tree
            // matrixInitialise(X[i],Y[i]);
             find_array_path( X[i],Y[i]);
             data1=0x35;         //reset values of data1, data2, data3 to receive new value
             data2=0x35;
             data3=0x35;
             if(i==0)            //as plucking mechanism is toward right therefore set an angle with respect to each node
             setangle(0);
             if(i==1)
             setangle(270);
             if(i==2)
             setangle(180);
             if(i==3)
             setangle(90);
             _delay_ms(100);
             check_l();
             buzzer_on();          //on buzzer indicating it reaches a node around tree
             _delay_ms(100);
             buzzer_off();
             checkPhase1();       //first phase of robotic arm to check the fruit
             while(!(data1==0x39||data1==0x32||data1==0x37))
             {
                USART_Transmit(82); //send R to rasp pi to tell it that reaches the node from where the fruit has to plucked
                data1= USART_Receive();
             }             
             if(data1==0x39)      //if fruit is found in first phase and has to be plucked then pluck function called
             {   servo_2(60);
                 _delay_ms(200);
                 servo_1(96);
                 _delay_ms(200);
                 plck=1;
                 pluck();
                 
            }    
            else if(data1==0x37)               //else it checks at second and third phase respectively
            { 
                checkPhase2();
                while(!(data2==0x39||data2==0x32||data2==0x37))
                {
                    USART_Transmit(82); 
                    data2= USART_Receive();
                }
                if(data2==0x39)
                {
                    
                    servo_2(63);
                    _delay_ms(200);
                    servo_1(95);
                    _delay_ms(200);
                    plck=2;
                    pluck();    
                }
    
            }
        }
    }         
 }
  /*
   * Function Name: findDirection
   * Input: node coordinates
   * Output: move the bot to the final position 
   * Logic: checks the difference between the current position and final position it has to reach. We have considered north to be +ve y-direction, 
           south be -ve y-direction, east be +ve x-direction, and west be -ve x-direction
   * Example Call: findDirection(2,3)
   */
 void findDirection(int x_pickup,int y_pickup)
 {
     //finding the difference in x and y direction from current position to final position
     int x=x_pickup-cx;
     int y=y_pickup-cy;
     
     
     if(y>0) //If difference in y direction is positive move north
     {
         while(cy!=y_pickup)
         {
             moveNorth();
         }
         
     }
     else if(y<0) //if difference in y direction is negative move south
     {
         while(cy!=y_pickup)
         {
             moveSouth();
         }
     }
     if(x>0) //if difference in x direction is positive move East
     {
         while(cx!=x_pickup)
         {
             moveEast();
         }
         
     }
     else if(x < 0)  //if difference in x direction is negative move west
     {
         while(cx!=x_pickup)
         {
             moveWest();
         }
         
     }
 }
 /*************************************************************************************************************************************************/
/**************************************************SET SERVOS ANGLE********************************************************************************/


//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
    float PositionPanServo = 0;
    PositionPanServo = ((float)degrees / 1.86) + 35.0;
    OCR1AH = 0x00;
    OCR1AL = (unsigned char) PositionPanServo;
}

//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
    float PositionTiltServo = 0;
    PositionTiltServo = ((float)degrees / 1.86) + 35.0;
    OCR1BH = 0x00;
    OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
    float PositionServo = 0;
    PositionServo = ((float)degrees / 1.86) + 35.0;
    OCR1CH = 0x00;
    OCR1CL = (unsigned char) PositionServo;
}

/*
  * Function Name: checkPhase1
  * Output: set the robotic arm to phase 1 for detecting fruit
  * Example Call: checkPhase1()
  */
void checkPhase1()
{
    servo_2(83);
    _delay_ms(300);
    servo_1(82);
    _delay_ms(300);
    
}

/*
  * Function Name: checkPhase2
  * Output: set the robotic arm to phase 2 for detecting fruit
  * Example Call: checkPhase2()
  */
void checkPhase2()
{
    servo_2(90);
    _delay_ms(300);
    servo_1(87);
    _delay_ms(300);
    
}
/*
* Function Name: main
* Output: sends C to confirm the communication with rpi and traverse through the path around the tree and deposit the fruit in its respective 
          deposition zone and sends E to rpi indicating work task has been complete
*/

int main(void)
{    float sVal,sVaR;
    init_devices();
    lcd_set_4bit();
    lcd_init();
    unsigned char check[3];
    unsigned char speedL;
    unsigned char speedR;
    unsigned char speed;
	unsigned char servo_X;
	unsigned char servo_Y;
    speedL = 0;
    speedR=0;
	
    
    while(1)
    {
        while(1)
        { unsigned char tempV;
            int i=0;
            tempV = USART_Receive();
            if(tempV=='l')
            {    //instantAngle = atoi(check);
                speedL = speed;
                speed=0;
                break;
            }
            else if(tempV=='r')
            {    //instantAngle = atoi(check);
                speedR = speed;
                speed=0;
                break;
            }
			else if(tempV=='x')
			{    //instantAngle = atoi(check);
				servo_X = speed;
				speed=0;
				break;
			}
			else if(tempV=='y')
			{    //instantAngle = atoi(check);
				servo_Y = speed;
				speed=0;
				break;
			}
            else
            {    speed = speed*10+ tempV;
                check[i]= tempV;
                i++;
            }
        }
        
        //unsigned char tempV;
        //tempV = USART_Receive();
        forward();
        velocity(speedL,speedR);
		servo_1(servo_X);
		servo_2(servo_Y);
      }   
    }
	

	
	
