/*
 * Water level sensor, button presser & motor controller for mister.
 * 
 * Copyright (C) 2021 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */







#ifdef _18F2450

// PIC18F2450 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2 // System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 21        // Brown-out Reset Voltage bits (2.1V)
#pragma config VREGEN = ON     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = BB1K     // Boot Block Size Select bit (1KW Boot block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) or (001000-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) or (000000-000FFFh) is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) or (001000-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) or (000000-000FFFh) is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) or (001000-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) or (000000-000FFFh) is not protected from table reads executed in other blocks)
#endif // _18F2450



#ifdef _18F14K50

// CONFIG1L
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection bits (No CPU System Clock divide)
#pragma config USBDIV = OFF     // USB Clock Selection bit (USB clock comes directly from the OSC1/OSC2 oscillator block; no divide)

// CONFIG1H
#pragma config FOSC = XT        // Oscillator Selection bits 
#pragma config PLLEN = ON       // 4 X PLL Enable bit (Oscillator multiplied by 4)
#pragma config PCLKEN = ON      // Primary Clock Enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 19        // Brown-out Reset Voltage bits (VBOR set to 3.0 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bit (WDT is always enabled. SWDTEN bit has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up bit (HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RA3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = OFF      // Boot Block Size Select bit (1kW boot block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protection bit (Block 0 not write-protected)
#pragma config WRT1 = OFF       // Table Write Protection bit (Block 1 not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block not protected from table reads executed in other blocks)
#endif // _18F14K50


#ifdef _18F1320

#endif // _18F1320





// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef _18F2450
#include <pic18f2450.h>
#define CLOCKSPEED 5000000
#define PWM_TRIS TRISCbits.TRISC1
#define PWM_LAT LATCbits.LATC1


// mist LEDs
#define LED_TRIS0 TRISAbits.TRISA1
#define LED_PORT0 PORTAbits.RA1
#define LED_TRIS1 TRISAbits.TRISA3
#define LED_PORT1 PORTAbits.RA3
#define LED_TRIS2 TRISBbits.TRISB3
#define LED_PORT2 PORTBbits.RB3

// mist buttons
#define MIST_TRIS0 TRISAbits.TRISA2
#define MIST_LAT0 LATAbits.LATA2
#define MIST_TRIS1 TRISAbits.TRISA4
#define MIST_LAT1 LATAbits.LATA4
#define MIST_TRIS2 TRISBbits.TRISB4
#define MIST_LAT2 LATBbits.LATB4

#endif // _18F2450

#ifdef _18F14K50
#include <pic18f14k50.h>
#endif

#ifdef _18F1320
// CONFIG1H
#pragma config OSC = INTIO2     // Oscillator Selection bits (Internal RC oscillator, port function on RA6 and port function on RA7)
#pragma config FSCM = ON        // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode enabled)

// CONFIG2L
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOR = ON         // Brown-out Reset Enable bit (Brown-out Reset enabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled, RA5 input pin disabled)

// CONFIG4L
#pragma config STVR = ON        // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Low-Voltage ICSP Enable bit (Low-Voltage ICSP disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (00200-000FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (001000-001FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (00200-000FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (001000-001FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (00200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (001000-001FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>


#include <pic18f1320.h>
#define CLOCKSPEED 8000000

// A0/AN0 - water level sensor input
// A1 - mist button
// A2 - mist button
// A4 - mist button
// B0/AN4 - mist LED
// B1/TX - debug
// A3/AN3 - mist LED
// B3 - motor output
// B4/AN6 - mist LED


#define PWM_TRIS TRISBbits.TRISB3
#define PWM_LAT LATBbits.LATB3

// mist LEDs.  Must be analog
#define LED_TRIS0 TRISBbits.TRISB0
#define LED_PORT0 PORTBbits.RB0
#define LED_TRIS1 TRISAbits.TRISA3
#define LED_PORT1 PORTAbits.RA3
#define LED_TRIS2 TRISBbits.TRISB4
#define LED_PORT2 PORTBbits.RB4

// mist buttons
#define MIST_TRIS0 TRISAbits.TRISA1
#define MIST_LAT0 LATAbits.LATA1
#define MIST_TRIS1 TRISAbits.TRISA2
#define MIST_LAT1 LATAbits.LATA2
#define MIST_TRIS2 TRISAbits.TRISA4
#define MIST_LAT2 LATAbits.LATA4
#endif // _18F1320






typedef union 
{
	struct
	{
		unsigned interrupt_complete : 1;
// water pump
        unsigned pwm_on : 1;
	};
	
	unsigned char value;
} flags_t;


#define BUTTON_TIME (HZ / 2)
#define LED_DEBOUNCE (HZ * 3)
#define ANALOG_PERIOD (HZ / 10)
typedef struct
{
// mist status from LED
    unsigned on : 1;
// mist status is unknown if this is 0
    unsigned valid : 1;
// button press timer
    uint16_t button_timer;
// debounced LED
    uint16_t led_on_timer;
    uint16_t led_off_timer;
} mist_t;

#define HZ 1000
#define TIMER0_PERIOD (CLOCKSPEED / 4 / HZ - 1)
#define MOTOR_PERIOD (CLOCKSPEED / 4 / 50)
#define MOTOR_OFF (CLOCKSPEED / 4 / 1000)
// duty cycle for pumping
#define MOTOR_ON  (CLOCKSPEED / 4 / 770)
//#define MOTOR_ON  (CLOCKSPEED / 4 / 800)
// water sensor values
#define LOW_THRESHOLD 32
#define HIGH_THRESHOLD 64
// LED is off if voltage is above this
#define LED_OFF 220
#define TOTAL_MISTS 3




volatile flags_t flags;
volatile mist_t mist[TOTAL_MISTS];
volatile uint16_t tick;
volatile uint16_t init_delay = HZ * 5;
volatile uint16_t analog_accum = 0;
volatile uint16_t analog_counter = 0;
volatile uint16_t analog_timer = 0;
// analog LED voltages
volatile uint16_t led0 = 0;
volatile uint16_t led1 = 0;
volatile uint16_t led2 = 0;
volatile uint16_t led0_counter = 0;
volatile uint16_t led1_counter = 0;
volatile uint16_t led2_counter = 0;

#ifdef _18F2450
volatile uint16_t motor_duty = 0;
#endif

// debounce the sensor
volatile uint16_t water_low_counter = 0;
volatile uint16_t water_high_counter = 0;
#define HIGH_DEBOUNCE 0
#define LOW_DEBOUNCE 10



// UART ------------------------------------------------------------------------

#ifdef _18F2450
#define UART_OUT_SIZE 256
#endif // _18F2450
#ifdef _18F1320
#define UART_OUT_SIZE 64
#endif // _18F1320

#define UART_IN_SIZE 8
static uint16_t serial_in_count = 0;
static uint16_t serial_in_ptr = 0;
static uint16_t serial_in_ptr2 = 0;
static uint8_t serial_in_buffer[UART_IN_SIZE];
static uint16_t serial_out_count = 0;
static uint16_t serial_out_ptr = 0;
static uint16_t serial_out_ptr2 = 0;
static uint8_t serial_out_buffer[UART_OUT_SIZE];
void init_uart()
{
    RCSTA = 0b10010000;
    TXSTA = 0b00100100;
    BAUDCTL = 0b00001000;
    SPBRG = CLOCKSPEED / 4 / 115200;
    PIR1bits.RCIF = 0;
    PIE1bits.RCIE = 1;
}

void handle_uart_rx()
{
    flags.interrupt_complete = 0;
// clear interrupt
    uint8_t c = RCREG;
    if(serial_in_count < UART_IN_SIZE)
    {
        serial_in_buffer[serial_in_ptr++] = c;
        serial_in_count++;
        if(serial_in_ptr >= UART_IN_SIZE)
        {
            serial_in_ptr = 0;
        }
    }
}

void handle_uart()
{
// clear the overflow bit
    if(RCSTAbits.OERR)
    {
        RCSTAbits.OERR = 0;
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
    }

    if(PIR1bits.TXIF)
    {
        if(serial_out_count > 0)
        {
            TXREG = serial_out_buffer[serial_out_ptr2++];
            if(serial_out_ptr2 >= UART_OUT_SIZE)
            {
                serial_out_ptr2 = 0;
            }
            serial_out_count--;
        }
    }
}

void flush_uart()
{
    while(serial_out_count)
    {
        handle_uart();
    }
}


void print_byte(uint8_t c)
{
	if(serial_out_count < UART_OUT_SIZE)
	{
		serial_out_buffer[serial_out_ptr++] = c;
		serial_out_count++;
		if(serial_out_ptr >= UART_OUT_SIZE)
		{
			serial_out_ptr = 0;
		}
	}
}

void print_text(const uint8_t *s)
{
	while(*s != 0)
	{
		print_byte(*s);
		s++;
	}
}

void print_number_nospace(uint16_t number)
{
	if(number >= 10000) print_byte('0' + (number / 10000));
	if(number >= 1000) print_byte('0' + ((number / 1000) % 10));
	if(number >= 100) print_byte('0' + ((number / 100) % 10));
	if(number >= 10) print_byte('0' + ((number / 10) % 10));
	print_byte('0' + (number % 10));
}

void print_number(uint16_t number)
{
    print_number_nospace(number);
   	print_byte(' ');
}

const uint8_t hex_table[] = { 
    '0', '1', '2', '3', '4', '5', '6', '7', 
    '8', '9', 'a', 'b', 'c', 'd', 'e','f'
};

void print_hex2(uint8_t number)
{
    print_byte(hex_table[number >> 4]);
    print_byte(hex_table[number & 0xf]);
   	print_byte(' ');
}

void print_bin(uint8_t number)
{
	print_byte((number & 0x80) ? '1' : '0');
	print_byte((number & 0x40) ? '1' : '0');
	print_byte((number & 0x20) ? '1' : '0');
	print_byte((number & 0x10) ? '1' : '0');
	print_byte((number & 0x8) ? '1' : '0');
	print_byte((number & 0x4) ? '1' : '0');
	print_byte((number & 0x2) ? '1' : '0');
	print_byte((number & 0x1) ? '1' : '0');
}




// MANE ------------------------------------------------------------------------

int main(int argc, char** argv)
{
#ifdef _18F1320
    OSCCON = 0b01110011;
#endif

    flags.value = 0;
    init_uart();

    print_text("\n\n\n\nWelcome to mist\n");
    flush_uart();



// mane timer
// 1:32 prescaler for 48Mhz clock
    T0CON = 0b10001000;
    TMR0 = -TIMER0_PERIOD;
    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;

    analog_accum = 0;
    analog_counter = 0;

#ifdef _18F2450
// motor PWM
    PWM_LAT = 1;
    PWM_TRIS = 0;
// DEBUG
//    PWM_TRIS = 1;
    T1CON = 0b10000001;
    motor_duty = MOTOR_OFF;
    TMR1 = -motor_duty;
    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;
    flags.pwm_on = 1;

// ADC
    TRISBbits.TRISB1 = 1;
    ADCON0 = 0b00000001;
// analog A0 only
    ADCON1 = 0b00001110;
    ADCON2 = 0b00111110;
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    ADCON0bits.GO = 1;
#endif // _18F2450

#ifdef _18F1320
// motor MOSFET
    PWM_LAT = 0;
    PWM_TRIS = 0;
// DEBUG: disable pump
//    PWM_TRIS = 1;

// ADC
    TRISAbits.TRISA0 = 1;
    ADCON0 = 0b00000001;
// analog pins
    ADCON1 = 0b10100110;
    ADCON2 = 0b00111110;
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    ADCON0bits.GO = 1;
#endif // _18F1320




// status LED input
    LED_TRIS0 = 1;
    LED_TRIS1 = 1;
    LED_TRIS2 = 1;
// status button input mode
    MIST_TRIS0 = 1;
    MIST_TRIS1 = 1;
    MIST_TRIS2 = 1;

// status button voltage
// set to 1 to disable misting
    MIST_LAT0 = 0;
    MIST_LAT1 = 0;
    MIST_LAT2 = 0;

    INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;




    while(1)
    {
        ClrWdt();
        handle_uart();
    }
    
    return (EXIT_SUCCESS);
}

void __interrupt(low_priority) isr1()
{
}

void __interrupt(high_priority) isr()
{
    flags.interrupt_complete = 0;
	while(!flags.interrupt_complete)
	{
		flags.interrupt_complete = 1;

// mane timer
        if(INTCONbits.TMR0IF)
        {
            flags.interrupt_complete = 0;
            INTCONbits.TMR0IF = 0;
            TMR0 = -TIMER0_PERIOD;
            tick++;
            analog_timer++;
            if(init_delay > 0)
            {
                init_delay--;
            }

//             if(!(tick % 100))
//             {
//                 print_number(LED_PORT);
//                 print_text("\n");
//             }






// manage water level
            if(analog_timer > ANALOG_PERIOD)
            {
                analog_accum /= analog_counter;
                led0 /= led0_counter;
                led1 /= led1_counter;
                led2 /= led2_counter;

// print causes PWM to fail on slow micros
                print_text("ANALOG: ");
                print_number(analog_counter);
                print_number(analog_accum);
                print_number(led0);
                print_number(led1);
                print_number(led2);
                print_text("\n");

// delay until the ESC initializes
// only pump if the switch is on
                if(init_delay == 0)
                {
                    if(analog_accum > HIGH_THRESHOLD)
                    {
                        water_low_counter = 0;
// overflows if it debounces the full side
                       if(water_high_counter < HIGH_DEBOUNCE)
                       {
                           water_high_counter++;
                       }
                       else
                       if(water_high_counter == HIGH_DEBOUNCE)
                       {
                            water_high_counter++;
                            print_text("water high\n");
#ifdef _18F2450
                            motor_duty = MOTOR_OFF;
#endif
#ifdef _18F1320
                            PWM_LAT = 0;
#endif
                       }
                    }
                    else
                    if(analog_accum < LOW_THRESHOLD)
                    {
                        water_high_counter = 0;
                        if(water_low_counter < LOW_DEBOUNCE)
                        {
                            water_low_counter++;
                        }
                        else
                        if(water_low_counter == LOW_DEBOUNCE)
                        {
                            water_low_counter++;
                            print_text("water low\n");

#ifdef _18F2450
                            motor_duty = MOTOR_ON;
#endif
#ifdef _18F1320
                            PWM_LAT = 1;
#endif
                        }
                    }
                }


// button is down
#define MIST_BUTTON(tris, led, ptr, text) \
{ \
    if(!tris) \
    { \
        ptr.button_timer += ANALOG_PERIOD; \
        if(ptr.button_timer >= BUTTON_TIME) \
        { \
            print_text(text " pressed\n"); \
            tris = 1; \
        } \
    } \
 \
 \
/* LED is off */ \
    if(led) \
    { \
        ptr.led_on_timer = 0; \
        if(ptr.led_off_timer < LED_DEBOUNCE) \
        { \
            ptr.led_off_timer += ANALOG_PERIOD; \
        } \
        else \
        if(ptr.on || !ptr.valid) \
        { \
            ptr.on = 0; \
            ptr.valid = 1; \
            print_text(text " off\n"); \
        } \
    } \
    else \
/* LED is on */ \
    { \
        ptr.led_off_timer = 0; \
        if(ptr.led_on_timer < LED_DEBOUNCE) \
        { \
            ptr.led_on_timer += ANALOG_PERIOD; \
        } \
        else \
        if(!ptr.on || !ptr.valid) \
        { \
            ptr.on = 1; \
            ptr.valid = 1; \
            print_text(text " on\n"); \
        } \
    } \
 \
 \
 \
    if(ptr.valid && !ptr.on) \
    { \
/* press button to start mist */ \
        tris = 0; \
        ptr.button_timer = 0; \
/* assume it's on & retest when the button releases */ \
/* can't know if it's really on until LED_DEBOUNCE */ \
        ptr.on = 1; \
/* delay the mist test until after the button is released */ \
        ptr.led_off_timer = 0; \
        ptr.led_on_timer = 0; \
        print_text(text " starting\n"); \
    } \
}

                MIST_BUTTON(MIST_TRIS0, (led0 > LED_OFF), mist[0], "mist0")
                MIST_BUTTON(MIST_TRIS1, (led1 > LED_OFF), mist[1], "mist1")
                MIST_BUTTON(MIST_TRIS2, (led2 > LED_OFF), mist[2], "mist2")



                analog_timer = 0;
                analog_counter = 0;
                analog_accum = 0;
                led0 = 0;
                led1 = 0;
                led2 = 0;
                led0_counter = 0;
                led1_counter = 0;
                led2_counter = 0;
            }
        }

        if(PIR1bits.ADIF)
        {
            flags.interrupt_complete = 0;
            PIR1bits.ADIF = 0;

            uint8_t code = ADCON0;
            switch(code)
            {
// water level
                case 0b00000001:
                    ADCON0 = 0b00010001;
                    if(analog_counter < 255)
                    {
                        analog_accum += ADRESH;
                        analog_counter++;
                    }
                    break;
// led0
                case 0b00010001:
                    ADCON0 = 0b00001101;
                    if(led0_counter < 255)
                    {
                        led0 += ADRESH;
                        led0_counter++;
                    }
                    break;
// led1
                case 0b00001101:
                    ADCON0 = 0b00011001;
                    if(led1_counter < 255)
                    {
                        led1 += ADRESH;
                        led1_counter++;
                    }
                    break;
// led2
                case 0b00011001:
                    ADCON0 = 0b00000001;
                    if(led2_counter < 255)
                    {
                        led2 += ADRESH;
                        led2_counter++;
                    }
                    break;
            }
            ADCON0bits.GO = 1;
        }


#ifdef _18F2450
        if(PIR1bits.TMR1IF)
        {
            PIR1bits.TMR1IF = 0;
            if(flags.pwm_on)
            {
                flags.pwm_on = 0;
                PWM_LAT = 0;
                TMR1 = -motor_duty;
            }
            else
            {
                flags.pwm_on = 1;
                PWM_LAT = 1;
                TMR1 = -(MOTOR_PERIOD - motor_duty);
            }
        }
#endif // _18F2450

        if(PIR1bits.RCIF)
        {
            flags.interrupt_complete = 0;
            handle_uart_rx();
        }
    }
}



