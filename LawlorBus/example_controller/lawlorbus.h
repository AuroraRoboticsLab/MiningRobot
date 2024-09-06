/**
Arduino header file implementation of LawlorBus, 
an I2C protocol variant that aims for better noise immunity
via direct drive of the output lines.

Relies on software "bit banging".


TODO:
  - Add length field, probably needs checksum, possibly:
    <~len 4 bits> <len 4 bits>
  - Shrink command field to 4 bits?
  - Replace digitalRead/Write with I/O port manipulation, for speed

Uno 328P / Nano 328P / Mega2560: pin change is on PORTC
    LD: A4 (same as SDA)
    LC: A5 (same as SCL)
Attiny85: pin change is on PORTB
    LD: 0/PB0 
    LC: 2/PB2

*/
#ifndef __LAWLORBUS_H
#define __LAWLORBUS_H 1

// Decide if we're a controller, or a peripheral
#if LAWLORBUS_CONTROLLER
#  define LAWLORBUS_PERIPHERAL 0
#elif LAWLORBUS_PERIPHERAL
#  define LAWLORBUS_CONTROLLER 0
#else
#  error "You need to #define either LAWLORBUS_CONTROLLER 1 or LAWLORBUS_PERIPHERAL 1"
#endif

#ifndef PIN_LD
// Arduino pin numbers for LD/LC pins:
#define PIN_LD A4
#define PIN_LC A5
#endif

// Maximum amount of data to send either way, in bytes.
//   EVERY DEVICE NEEDS TO USE THE SAME VALUE (for now)
#ifndef LAWLORBUS_MAX 
#  define LAWLORBUS_MAX 4
#endif

struct lawlorbus_max_command {
    unsigned char addr;
    unsigned char data[LAWLORBUS_MAX];
};

struct lawlorbus_max_report {
    unsigned char data[LAWLORBUS_MAX];
};

// Microsecond delay per phase of the clock while sending bits:
//   10 -> byte error rate 1.2% over jumpers
//   15 -> byte error rate 0.00% over jumpers
#define LAWLORBUS_DELAYUS_CLOCKBIT 15

// Microsecond delay during attention phase before controller send
#define LAWLORBUS_DELAYUS_CONTROLLER_SEND 50
// Microsecond delay before peripheral send
#define LAWLORBUS_DELAYUS_PERIPHERAL_SEND 20


// Return code: timeout
#define LAWLORBUS_RET_TIMEOUT -1
// Return code: successful data exchange
#define LAWLORBUS_RET_SUCCESS 10 
// Return code: idle bus
#define LAWLORBUS_RET_IDLE 0


// Return the current state of the lawlorbus clock line
#define LAWLORBUS_CLOCK digitalRead(PIN_LC)
// Return the current state of the lawlorbus data line
#define LAWLORBUS_DATA digitalRead(PIN_LD)


// Internal functions (i), don't call these directly

unsigned char lawlorbusI_timeout = 0; // for reporting timeouts

// Set the bus to input mode 
void lawlorbusI_mode_input(int mode)
{
    pinMode(PIN_LD,INPUT);
    pinMode(PIN_LC,mode);
}

// Return the next byte read off the bus:
//   a series of data bits, LSB first, clocked in on the rising edge.
unsigned char lawlorbusI_recv_byte()
{
    int leash=300; // total timeout = this * a few microseconds per read

    unsigned char data=0;
    
    int bit=1; // start at LSB
    do {
        // wait for low clock
        while (LAWLORBUS_CLOCK==1) { 
            if (--leash<0) goto timeout;
        } 
        // wait for rising edge
        while (LAWLORBUS_CLOCK==0) { 
            if (--leash<0) goto timeout;
        } 
        
        // read one data bit
        if (LAWLORBUS_DATA) data |= bit;
        
        bit = bit<<1; // up to the next bit
    } while (bit<0x100);
    
    return data;

timeout:
    ++lawlorbusI_timeout;
    return 0;
}

// Set the bus to output mode, with this clock
void lawlorbusI_mode_output(int dataline, int clockline)
{
    pinMode(PIN_LD,OUTPUT); digitalWrite(PIN_LD,dataline);
    pinMode(PIN_LC,OUTPUT); digitalWrite(PIN_LC,clockline);
    
}

// Send this byte on the (prepared) bus.  Leaves clock high.
void lawlorbusI_send_byte(unsigned char data=0) {
    int bit=1; // start at LSB
    do {
        // set data line
        digitalWrite(PIN_LD,data & bit);
    
        digitalWrite(PIN_LC,0); // clock low
        delayMicroseconds(LAWLORBUS_DELAYUS_CLOCKBIT);
        digitalWrite(PIN_LC,1); // clock high
        // - this is the transition where other side reads -
        delayMicroseconds(LAWLORBUS_DELAYUS_CLOCKBIT);
        
        bit = bit<<1; // up to the next bit
    } while (bit<0x100);
}


#if LAWLORBUS_CONTROLLER
// Controller: bus idles in output mode

void lawlorbus_begin()
{
    lawlorbusI_mode_output(0,1);
}

int lawlorbus_send(int peripheral_addr,
    const void *command, int command_bytes, 
    void *report, int report_bytes)
{ 
    // Begin send by bringing the clock low (attention)
    lawlorbusI_mode_output(0,0);
    
    lawlorbus_max_command mc = {0};
    lawlorbus_max_report mr={0};
    mc.addr = peripheral_addr;
    memcpy(mc.data,command,command_bytes);
    delayMicroseconds(LAWLORBUS_DELAYUS_CONTROLLER_SEND);
    
    // Send address and command
    lawlorbusI_send_byte(mc.addr);
    for (int c=0;c<LAWLORBUS_MAX;c++)
        lawlorbusI_send_byte(mc.data[c]);
    


    // Switch to weak pull-up mode for report
    lawlorbusI_mode_input(INPUT_PULLUP);
    // Wait to see the peripheral's 0 clock  
    int leash = 300; 

    while (LAWLORBUS_CLOCK==1) { 
        if (--leash<0) goto timeout;
    } 

    // We see the peripheral's 0 clock!  Switch to full input mode
    lawlorbusI_mode_input(INPUT);

    for (int r=0;r<LAWLORBUS_MAX && 0==lawlorbusI_timeout;r++)
        mr.data[r] = lawlorbusI_recv_byte();
    if (lawlorbusI_timeout>0) goto timeout;

    memcpy(report,mr.data,report_bytes);

    // back to bus idle
    lawlorbusI_mode_output(0,1);

    return LAWLORBUS_RET_SUCCESS;

timeout:
    // back to bus idle anyway
    lawlorbusI_mode_output(0,1);

    return LAWLORBUS_RET_TIMEOUT;
}

#endif // controller section


#if LAWLORBUS_PERIPHERAL
// Peripheral: bus idles in input mode

/// Set up the lawlorbus pins
void lawlorbus_begin()
{
    lawlorbusI_mode_input(INPUT);
}

/// Listen for a lawlorbus command.
///   If one arrives for our address, send the report.
int lawlorbus_listen(int my_addr,
    void *command, int command_bytes, 
    const void *report, int report_bytes)
{
    if (LAWLORBUS_CLOCK!=0) return 0; // clock high, nothing happening.
    // else clock is low: we're in attention phase
    lawlorbusI_timeout=0;
    int ret=LAWLORBUS_RET_IDLE;
    
    lawlorbus_max_command mc;
    mc.addr = lawlorbusI_recv_byte();
    
    for (int c=0;c<LAWLORBUS_MAX && 0==lawlorbusI_timeout;c++)
        mc.data[c] = lawlorbusI_recv_byte();
    
    lawlorbus_max_report mr={0};
    if (mc.addr==my_addr && 0==lawlorbusI_timeout)
    { // that's us!
        lawlorbusI_mode_output(1,0); // ack the command immediately
        
        memcpy(command,mc.data,command_bytes); // copy out command
        memcpy(mr.data,report,report_bytes); // copy in report
        
        delayMicroseconds(LAWLORBUS_DELAYUS_PERIPHERAL_SEND);
        for (int r=0;r<LAWLORBUS_MAX;r++)
            lawlorbusI_send_byte(mr.data[r]);
        
        lawlorbusI_mode_input(INPUT);
        ret = LAWLORBUS_RET_SUCCESS; // got command and sent out report
    }
    else {
        // wait through another address's report
        for (int r=0;r<LAWLORBUS_MAX && 0==lawlorbusI_timeout;r++)
            mr.data[r] = lawlorbusI_recv_byte();
        // ret = 1;  //<- not useful to report other bus traffic
    }
    
    if (lawlorbusI_timeout>0) return LAWLORBUS_RET_TIMEOUT;
    return ret;
}



#endif // peripheral section




#endif




