/*
 
Copyright (c) 2012, Cameron Harper
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met: 

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#define F_CPU       16000000UL

#include <stdint.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/io.h>
#include <avr/wdt.h>

#include <util/delay_basic.h>

#define UART_TXPIN      0x02
#define UART_TXDDR      DDRD

#define UART_MAXLEN     8           
#define BAUD            19200UL    

struct uart_driver {

    #define UART_IDLE       0
    #define UART_RX         1
    #define UART_READY      2
    #define UART_TX         4
    #define UART_TX_FINISH  5
    uint8_t state;
    
    uint8_t *data;
    uint16_t len;
    
    uint8_t buffer[UART_MAXLEN];
};

volatile struct uart_driver uart;

#define FRAMECHAR   0x7F
#define STUFFCHAR   0x7D
#define XORCHAR     0x40

/* uart rx interupt with framing */
ISR(USART_RX_vect)
{
    uint8_t c = UDR0;

    switch(uart.state & 0x7f){
    case UART_IDLE:

        if(c == FRAMECHAR){
            uart.state = UART_RX;
            uart.len = 0;
            uart.data = (uint8_t *)uart.buffer;
        }
        break;
        
    case UART_RX:
        
        switch(c){
        case STUFFCHAR:
            uart.state |= 0x80;
            break;
            
        case FRAMECHAR:

            if(uart.len){
                uart.state = UART_READY;
                break;
            }

            uart.len = 0;
            uart.data = (uint8_t *)uart.buffer;
            break;
            
        default:
            
            if(uart.state & 0x80){
                uart.state &= ~(0x80);
                c ^= XORCHAR;        
            }
            
            if(uart.len == UART_MAXLEN){
                uart.state = UART_IDLE;
                break;
            }
            
            uart.data[uart.len++] = c;
        }
        
        break;
    }
}

/* uart tx interupt with framing */
ISR(USART_TX_vect)
{
    switch(uart.state & 0x7f){
    case UART_TX:
        
        if(uart.len){
            
            if(uart.state & 0x80){
                uart.state &= ~0x80;
                UDR0 = *uart.data ^ XORCHAR;
            }
            else if((*uart.data == FRAMECHAR)||(*uart.data == STUFFCHAR)){
                uart.state |= 0x80;
                UDR0 = FRAMECHAR;
                return;
            }
            else
                UDR0 = *uart.data;
            
            uart.data++;
            uart.len--;
            
        }
        else{
            UDR0 = FRAMECHAR;
            uart.state = UART_TX_FINISH;
        }
        break;
    
    case UART_TX_FINISH:
        uart.state = UART_IDLE;
    }    
}

/* send a buffer over UART with framing */
static void uart_send(uint8_t *data, uint16_t len)
{
    ATOMIC_BLOCK(ATOMIC_FORCEON){

        if(uart.state == UART_READY){
            
            uart.data = data;
            uart.len = len;
            
            uart.state = UART_TX;
            
            /*send the first character to get things started...*/
            UDR0 = FRAMECHAR;
        }
    }
}

/* soft TWI context */
typedef struct {
    
    uint16_t half;  /* ticks in a half cycle */
    uint16_t retry;  /* number of half cycles to stretch SCL */

    /* +2 PORT register
     * +1 DDR register
     * +0 PIN register */
    volatile uint8_t *pinreg;

    /* masks */
    uint8_t sda_out;    
    uint8_t scl_out;
    uint8_t sda_in;    
    uint8_t scl_in;

} twi_t;

#define PR01_VERSION            0
#define PR01_DEFAULT_RETRY      1000
#define PR01_DEFAULT_RATE       ((F_CPU / 10000) >> 1)

/* PR01 Service Primitive Tags */
typedef enum {
    PR01_VERSION_REQ = 0x0,
    PR01_VERSION_CNF,
    PR01_READ_REQ,
    PR01_READ_CNF,
    PR01_WRITE_REQ,
    PR01_WRITE_CNF,
    PR01_SETATTR_REQ,
    PR01_SETATTR_CNF,
} pr01_primitive_t;

/* PR01 Response Codes */
typedef enum {
    PR01_RESP_SUCCESS = 0x0,
    PR01_RESP_TEMPFAIL,
    PR01_RESP_BUSY,
    PR01_RESP_COLLISION,
    PR01_RESP_TIMEOUT,
    PR01_RESP_NACK,
    PR01_RESP_NOBUS    
} pr01_resp_t;

static inline void twi_delay(twi_t *t)
{
    _delay_loop_2(t->half);
}

#define TWI_DELAY(T) twi_delay(T)

/* set state of scl */
static inline void scl_hi(twi_t *t){*(t->pinreg+1) &= ~(t->scl_out);}
static inline void scl_low(twi_t *t){*(t->pinreg+1) |= (t->scl_out);}

/* check the programmed and actual state of scl */
static inline unsigned scl_out_state(twi_t *t){return (*(t->pinreg+1) & t->scl_out)?1:0;}
static inline unsigned scl_in_state(twi_t *t){return (*t->pinreg & t->scl_in)?1:0;}

/* set state of sda */
static inline void sda_hi(twi_t *t){*(t->pinreg+1) &= ~(t->sda_out);}
static inline void sda_low(twi_t *t){*(t->pinreg+1) |= (t->sda_out);}

/* check the programmed and actual state of sda */
static inline unsigned sda_out_state(twi_t *t){return (*(t->pinreg+1) & t->sda_out)?1:0;}
static inline unsigned sda_in_state(twi_t *t){return (*t->pinreg & t->sda_in)?1:0;}

static int twi_stretch_scl(twi_t *t)
{
    uint16_t i = t->retry;

    scl_hi(t);
    
    while(!scl_in_state(t)){

        if(i)i--;
        else return PR01_RESP_TIMEOUT;

        TWI_DELAY(t);
    }

    return PR01_RESP_SUCCESS;
}

static int twi_start(twi_t *t)
{
    int i;

    /* already started (restart) */
    if(scl_out_state(t)){

        /* release SDA if we are holding it */
        if(sda_out_state(t)){

            sda_hi(t);
            TWI_DELAY(t);    
        }

        /* release SCL */
        if(twi_stretch_scl(t))
            return PR01_RESP_TIMEOUT;

        TWI_DELAY(t);
    }
    /* probably idle...but check */
    else if(!scl_in_state(t)){

        return PR01_RESP_BUSY;
    }
     
    /* SDA is being held down by another station...clear it */
    if(!sda_in_state(t)){
    
        /* SDA should come high in the next 9 clock cycles */
        for(i=0; i<9; i++){

            scl_low(t);

            TWI_DELAY(t);

            if(twi_stretch_scl(t))
                return PR01_RESP_TIMEOUT;

            TWI_DELAY(t);

            if(sda_in_state(t))
                break;
        }

        /* SDA seems permanently low */
        if(i == 9)
            return PR01_RESP_NOBUS;
    }        

    /* now start condition */
    sda_low(t);
    
    TWI_DELAY(t);

    scl_low(t);

    return PR01_RESP_SUCCESS;
}

static int twi_getbyte(twi_t *t, unsigned option, uint8_t *b)
{
    int ret;
    uint8_t i, c;

    /* start */
    if(option & 0x1){

        if((ret = twi_start(t)))
            return ret;
    }
    else{
    
        /* make sure this master holds the bus */
        if(!scl_out_state(t))
            return PR01_RESP_NOBUS;
    }
    
    /* release SDA */
    sda_hi(t);

    for(i=8; i; i--, c<<=1){

        TWI_DELAY(t);

        if(twi_stretch_scl(t))
            return PR01_RESP_TIMEOUT;

        if(sda_in_state(t))
            c |= 0x1;
        else
            c &= ~(0x1);
        
        TWI_DELAY(t);
        
        scl_low(t);
    }

    sda_low(t);

    TWI_DELAY(t);

    if(twi_stretch_scl(t))
        return PR01_RESP_TIMEOUT;

    *b = c;

    TWI_DELAY(t);
    
    /* finish with stop or start */
    if(option & 0x6){

        sda_hi(t);

        TWI_DELAY(t);

        /* stop gets out here */
        if(option & 0x2){

            if(!sda_in_state(t))
                return PR01_RESP_COLLISION;
            
            return PR01_RESP_SUCCESS;
        }

        sda_low(t);

        TWI_DELAY(t);
    }

    scl_low(t);

    return ret;
}

static int twi_putbyte(twi_t *t, unsigned option, uint8_t b)
{
    int ret;
    uint8_t i = 0x0;

    /* start */
    if(option & 0x1){

        if((ret = twi_start(t)))
            return ret;
    }
    else{
    
        /* can't send without arbitration */
        if(!scl_out_state(t))
            return PR01_RESP_NOBUS;
    }

    /* data */
    for(i=8; i; i--, b<<=1){

        (b & 0x80)?(sda_hi(t)):(sda_low(t));

        TWI_DELAY(t);

        if(twi_stretch_scl(t))
            return PR01_RESP_TIMEOUT;

        /* collision - abort */
        if((b & 0x80) && !(sda_in_state(t))){

            sda_hi(t);
            return PR01_RESP_COLLISION;
        }
         
        TWI_DELAY(t);

        scl_low(t);
    }

    sda_hi(t);

    TWI_DELAY(t);

    if(twi_stretch_scl(t))
        return PR01_RESP_TIMEOUT;

    b = (sda_in_state(t))?1:0;
        
    TWI_DELAY(t);

    scl_low(t);

    /* end with stop */
    if(option & 0x2){

        TWI_DELAY(t);

        /* collision - abort */
        if(!sda_in_state(t)){
            scl_hi(t);
            return PR01_RESP_COLLISION;
        }

        sda_low(t);

        TWI_DELAY(t);

        if(twi_stretch_scl(t)){

            sda_hi(t);
            
            return PR01_RESP_TIMEOUT;
        }

        TWI_DELAY(t);

        sda_hi(t);

        TWI_DELAY(t);

         return PR01_RESP_SUCCESS;       
    }
    /* end with start */
    else  if(option & 0x4){
        if((ret = twi_start(t)))
            return ret;
    }
        
    if(!b)
        return PR01_RESP_SUCCESS;
    else
        return PR01_RESP_NACK;
}

static int twi_init(twi_t *t, volatile uint8_t *pinreg, uint8_t sda_in, uint8_t sda_out, uint8_t scl_in, uint8_t scl_out)
{
    t->pinreg = pinreg;
    t->sda_in = sda_in;
    t->sda_out = sda_out;
    t->scl_in = scl_in;
    t->scl_out = scl_out;
    t->retry = PR01_DEFAULT_RETRY;
    t->half = PR01_DEFAULT_RATE;

    *(pinreg+1) &= ~(scl_out|sda_out);  
    *(pinreg+2) &= ~(scl_out|sda_out);  

    return PR01_RESP_SUCCESS;
}

static int pr01_process(twi_t *t, uint8_t *data, unsigned len)
{
    uint8_t service;
    int ret;
    
    if(!len)
        return -1;
    
    service = data[0];
    data[0] = data[0]+1;
    
    switch(service){
    case PR01_VERSION_REQ:
        
        if(len != 1)
            return -1;
        
        data[1] = PR01_VERSION;
        len = 2;
        break;

    case PR01_READ_REQ:
        
        if(len != 2)
            return -1;
        
        if((ret = twi_getbyte(t, data[1], data+2))){
            data[1] = 1;
            data[2] = ret;
        }
        else{
            data[1] = 0;
        }        
        len = 3;
        break;
        
    case PR01_WRITE_REQ:
    
        if(len != 3)
            return -1;
        
        data[1] = twi_putbyte(t, data[1], data[2]);
        len = 2;
        break;

    default:
        return -1;
    }

    uart_send(data, len);    
    
    return 0;
}

void main(void) __attribute__((noreturn));

void main(void)
{
    twi_t t;

    /* uart */
    UBRR0L = (uint8_t)(F_CPU/(BAUD*16L)-1);
    UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0) | (1<<TXCIE0);
    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
    PORTD |= (1<<PIND1);
    DDRD |= (1<<PIND1);
    PORTD |= (1<<PIND0);

    twi_init(&t, &(PINC), (1<<PINC4), (1<<PINC4), (1<<PINC5), (1<<PINC5));

    uart.state = UART_IDLE;

    sei();

    while(1){

        if(uart.state == UART_READY){
            if(pr01_process(&t, uart.data, uart.len))
                uart.state = UART_IDLE;
        }

    }
}
