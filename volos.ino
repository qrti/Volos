// volos.ino voltage tracker for oscilloscope V0.1 qrt160315 qrt@qland.de
//
// V0.10 initial

#include <inttypes.h>
#include <ctype.h>
#include <EEPROM.h>
#include "TimerOne.h"

#define MAXINT   0x7fff
#define MAXUINT  0xffff
#define MAXLONG  0x7fffffff
#define MAXULONG 0xffffffff

// LED cathode to Ground
#define LED_ON  HIGH
#define LED_OFF LOW

// 100 us T1 cycle * 1200 = 120 ms
#define T1CYC   100
#define DISPW  1200

// Vcc 5 V
#define VCC     5

const int Pin_VIN = A0;                 // test voltage IN
const int Pin_RED_LED13 = 13;           // standard LED
                   
//const int Pin_KEY_1       = A8;       // AVR PK0
//const int Pin_KEY_2       = A9;       // AVR PK1
//const int Pin_KEY_3       = A10;      // AVR PK2
//const int Pin_KEY_4       = A11;      // AVR PK3
//const int Pin_KEY_MINUS   = A12;      // AVR PK4
//const int Pin_KEY_PLUS    = A13;      // AVR PK5
//const int Pin_KEY_FUNC    = A14;      // AVR PK6
//const int Pin_KEY_POINT   = A15;      // AVR PK7

//const int Pin_TRIGGER     = 30;       // AVR PC7

#define KEY_MINUS       (1<<4)
#define KEY_PLUS        (1<<5)
#define KEY_SET         (1<<6)
#define KEY_START       (1<<7)
#define KEY_LONG        0x8000

#define TRIGGER         0x80

uint8_t data[DISPW];
uint16_t dai;
 
void timer1()
{    
    if(dai >= DISPW){
        dai = 0;
        PORTC = 0b10000000;
        PORTC = 0b00000000;
    }
    
    PORTA = data[dai++];
}

void setup()
{
    DDRA  = 0b11111111;                 // arduino digital pin 29..22
    PORTA = 0b00000000;
    
    DDRC  = 0b11000000;                 // arduino digital pin 30..37
    PORTC = 0b00000000;
    
    DDRK  = 0b00000000;                 // arduino analog pin A15.A8
    PORTK = 0b11111111;
    
    pinMode(Pin_VIN, INPUT);
    pinMode(Pin_RED_LED13, OUTPUT);
    
    digitalWrite(Pin_RED_LED13, LED_OFF);    
    
    Timer1.initialize(T1CYC);           // attach timer 1
    Timer1.attachInterrupt(timer1);        
    
    eeReadSet();                        // read settings from EEPROM
}

#define SET_TIME    0
#define SET_LEVEL   1
#define SET_ROLL    2
#define SET_YSC     3
#define SET_NUD     4
#define MEAS_RUN    5
#define MEAS_STOP   6

uint8_t mode;

void loop()
{
    switch(mode){
        case MEAS_RUN:
            measRun();
            break;
        
        case MEAS_STOP:
            measStop();
            break;
        
        case SET_TIME:
            setTime();
            break;

        case SET_LEVEL:
            setLevel();
            break;

        case SET_ROLL:
            setRoll();
            break;
            
        case SET_YSC:
            setYsc();
            break;     

        case SET_NUD:
            setNud();
            break;            
    }        
}

//------------------------------------------------------------------------------

uint32_t mint, sint;                // measure + store interval
uint32_t mintc, sintc;              //                          counter
uint32_t lintc;                     // LED interval counter

uint16_t mcnt, sti;                 // measure counter, store index
uint32_t msum;                      //         sum

// time per DIV
#define NUMTID          7
#define NUMTID_DEF      0
// screen time           12m 24m  1h   2h   4h    6h    12h         (@ 12 DIV screen)
// time per DIV          1m  2m   5m   10m  20m   30m   1h
uint16_t tdiv[NUMTID] = {60, 120, 300, 600, 1200, 1800, 3600};   // (in seconds)
int8_t tid;             // array index 

// level    0 1 2 3 4
//          0 1 2 3 4 V 
#define NUMLEV          5 
#define NUMLEV_DEF      0
int8_t lev;                         
                
// roll       0  1 
//          off on                      
#define NUMROL          2 
#define NUMROL_DEF      0 
int8_t rol;                        
                 
// y scale  0 ADmax / 1, osci y 1/4 -> lev .. lev + 1.25 V -> lev max 4 V
//          1       / 2         1/2 -> lev .. lev + 2.50 V            3 V
//          2       / 3         3/4 -> lev .. lev + 3.75 V            2 V
//          3       / 4         1/1 ->   0 .. 5.0 V                   0 V
#define NUMYSC          4 
#define NUMYSC_DEF      0 
int8_t ysc;                         

// number of divs    0  1  2  3  4  5 6                  
//                  10 11 12 13 14 15 16                       
#define NUMNUD          7 
#define NUMNUD_DEF      2      
#define BASENUD        10     
int8_t nud;

bool refresh = true;
bool eesave = false;

void prepMeas(bool full)
{
    if(full){
        for(uint16_t i=0; i<DISPW; i++)
            data[i] = 0;
            
        sint = (uint32_t)tdiv[tid] * (BASENUD + nud) * 1000 / DISPW;        // <- TDIV = STOIV * DATAS / NUMDIV
        mint = sint / (tid + 4);    
        sti = 0;
    }    
    
    if(eesave){                     // write settings to EEPROM
        eeWriteSet();
        eesave = false;
    }
    
    sintc = millis() + sint;
    mintc = millis() + mint;
    mcnt = 0;
    msum = 0;
    lintc = millis();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void measRun()
{   
    if(rol==0 && sti>=DISPW){
        digitalWrite(Pin_RED_LED13, LED_ON);
        mode = MEAS_STOP;
        return;
    }       
    
    keyMeasRun();
    
    if(millis() >= mintc){
        msum += analogRead(Pin_VIN);
        mcnt++;
        mintc += mint;
    }
    
    if(millis() >= sintc){
        if(sti >= DISPW){
            for(int i=1; i<DISPW; i++)
                data[i-1] = data[i];
            
            sti = DISPW - 1;
        }     
        
        int16_t r = ((int16_t)(msum / mcnt) - (1023 * lev / VCC)) / (ysc + 1);     
        if(r > 255) r = 255;
        if(r < 0) r = 0;
        data[sti++] = r;

        msum = 0;
        mcnt = 0;
        sintc += sint;        
    }     
    
    if(millis() >= lintc){
        digitalWrite(Pin_RED_LED13, !digitalRead(Pin_RED_LED13)); 
        lintc += 500;               // 500 ms toggle
    }
}

void keyMeasRun()
{
    uint16_t key = checkKey();
    
    switch(key){
        case KEY_START:
            digitalWrite(Pin_RED_LED13, LED_ON);
            mode = MEAS_STOP;
            break;
    }
}

//------------------------------------------------------------------------------

void measStop()
{
    keyMeasStop();
}

void keyMeasStop()
{
    uint16_t key = checkKey();
    
    switch(key){
        case KEY_START:
            prepMeas(false);
            mode = MEAS_RUN;
            break;
        
        case KEY_SET:
            digitalWrite(Pin_RED_LED13, LED_OFF);
            mode = SET_TIME;
            refresh = true;
            break;
            
        case KEY_SET | KEY_LONG:
            prepMeas(true);
            break;            
    }
}

//------------------------------------------------------------------------------

uint8_t chr_t[] = { 0, 2, 1, 1, 
                    1, 0, 1, 4, 
                    2, 2, 1, 1 };

uint8_t chr_l[] = { 0, 1, 1, 3, 
                    1, 0, 1, 1, 
                    2, 1, 1, 1 };                     
                     
uint8_t chr_y[] = { 0, 2, 1, 2, 
                    1, 0, 1, 2, 
                    2, 2, 1, 2 };             

uint8_t chr_r[] = { 0, 0, 1, 3, 
                    1, 3, 1, 1, 
                    2, 2, 1, 1 };              
   
uint8_t chr_n[] = { 0, 0, 1, 4, 
                    1, 3, 1, 1, 
                    2, 0, 1, 3 };    

#define CHRWIDTH    3
#define VALMAX      8                          
#define VALDY      (256 /  VALMAX)              
#define CHRDY       VALDY * 3 / 2

void setTime()
{
    set(&tid, NUMTID, SET_LEVEL, chr_t, 1);
}

void setLevel()
{
    set(&lev, NUMLEV, SET_ROLL, chr_l, 0);
}

void setRoll()
{
    set(&rol, NUMROL, SET_YSC, chr_r, 0);
}

void setYsc()
{
    set(&ysc, NUMYSC, SET_NUD, chr_y, 0);
}

void setNud()
{
    set(&nud, NUMNUD, SET_TIME, chr_n, 0);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void set(int8_t* v, int8_t num, uint8_t next, uint8_t* chr, int8_t first)
{
    uint16_t key = checkKey();
    
    if(refresh){
        uint16_t chrdx = DISPW / (BASENUD + nud) / 2;
        drawChar(chr, 0);
        drawValue(*v+first, (CHRWIDTH+1)*chrdx);
        refresh = false;
    }
    
    switch(key){
        case KEY_MINUS:
            if(--*v < 0) { *v = 0; } else { refresh = eesave = true; }
            break;
            
        case KEY_PLUS:
            if(++*v >= num) { *v = num-1; } else { refresh = eesave = true; }
            break;
            
        case KEY_START:
            prepMeas(true);
            mode = MEAS_RUN;
            break;      
            
        case KEY_SET:
            mode = next;
            refresh = true;
            break;         
    }
}

//------------------------------------------------------------------------------

uint16_t checkKey()
{
    static uint32_t t;
    static uint16_t k;
    uint16_t key = PINK ^ 0xff;
    
    if(key != 0){
        if(t == 0){
            t = millis();
            k = key;
        }
        
        if(!(k&KEY_LONG) && millis()-t>2000){  
            k |= KEY_LONG;
            key = k;
        }
        else{
            key = 0;
        }
    }
    else{
        if(k!=0 && !(k&KEY_LONG) && millis()-t>50)
            key = k;
            
        t = 0;
        k = 0;
    }
    
    return key;
}
                     
void drawChar(uint8_t *c, uint16_t xb)
{
    uint16_t chrdx = DISPW / (BASENUD + nud) / 2;
    
    for(int i=0; i<3*4; i+=4){
        uint16_t x0 = c[i] * chrdx;
        uint16_t y0 = (uint16_t)c[i+1] * CHRDY;
        uint16_t x1 = x0 + c[i+2] * chrdx;
        uint16_t y1 = y0 + c[i+3] * CHRDY;
        
        for(uint16_t x=x0; x<x1; x++)
            data[xb + x] = x % 2 ? y1 : y0;
    }
    
    for(uint16_t x=0; x<chrdx; x++)         // delete gap between chr and data
        data[xb + 3*chrdx + x] = 0;
    
}                                                                 
                          
void drawValue(uint8_t v, uint16_t xb)
{
    uint16_t y0 = VALDY;
    uint16_t valdx = DISPW / (BASENUD + nud);

    for(uint16_t i=0; i<v; i++){
        for(uint16_t x=0; x<valdx; x++)
            data[xb + x] = x % 2 ? y0 : 0;

        xb += valdx;
        y0 += VALDY;
    }
    
    for(uint16_t x=xb; x<DISPW; x++)        // delete to end of data
        data[x] = 0;
}                             

//------------------------------------------------------------------------------

void eeReadSet()
{
    tid = EEPROM.read(SET_TIME); 
    if(tid<0 || tid>=NUMTID) tid = NUMTID_DEF;
    
    lev = EEPROM.read(SET_LEVEL);
    if(lev<0 || lev>=NUMLEV) lev = NUMLEV_DEF;

    rol = EEPROM.read(SET_ROLL); 
    if(rol<0 || rol>=NUMROL) rol = NUMROL_DEF;

    ysc = EEPROM.read(SET_YSC);  
    if(ysc<0 || ysc>=NUMYSC) ysc = NUMYSC_DEF;

    nud = EEPROM.read(SET_NUD);  
    if(nud<0 || nud>=NUMNUD) nud = NUMNUD_DEF;    
}

void eeWriteSet()
{
    EEPROM.write(SET_TIME, tid); 
    EEPROM.write(SET_LEVEL, lev);
    EEPROM.write(SET_ROLL, rol); 
    EEPROM.write(SET_YSC, ysc);  
    EEPROM.write(SET_NUD, nud);  
}                             