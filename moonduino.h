#ifndef ROTATOR_H
#define ROTATOR_H 1

#include <avr/eeprom.h>
#include <avr/interrupt.h>

//#define DEBUG

// Active inline function
//#define OPTIMIZE_SPEED

// Force the F_CPU define to 16Mhz
#ifndef F_CPU
    #define F_CPU (16000000UL)
#endif

#if defined(OPTIMIZE_SPEED)
    #define FORCE_INLINE    __attribute__((always_inline))
    #define INLINE          inline
#else
    #define FORCE_INLINE    /*__attribute__ ((noinline))*/
    #define INLINE          /*static*/
#endif

/*******************************************************************************
*
* Typedef/enum section
* 
*******************************************************************************/
typedef uint32_t    pos_t;
typedef uint32_t    time_t;
typedef uint8_t *   string_t;

enum Direction      { FOCUS_OUTWARD = 0, FOCUS_INWARD = 1, NOT_MOVE = 2 };
enum Digits         { TWO_DIGITS = 2, FOUR_DIGITS = 4, FIVE_DIGITS = 5, MAX_DIGIT = 5};
enum Version        { MAJOR = 8, MINOR = 10, PREVIOUS_VERSION_EE = 0x88, CURRENT_VERSION_EE = 0x89 };
enum Bool           { FALSE = 0, INACTIVE = 0, TRUE = 1, ACTIVE = 1};
enum MotorStep      { HALF_STEP = 0, FULL_STEP_2_COILS = 1, FULL_STEP_1_COIL = 2};
enum SpecialChars   { SZERO = '\0', SPACE = ' ', HASH = '#', DDOT = ':', MINUS = '-', PLUS = '+'};
enum Whells         { MIN = 20, MAX = 62};

/*******************************************************************************
*
* U A R T  S E C T I O N
* 
*******************************************************************************/
#define CMDLEN      (2)       // Command len
#define PARAMLEN    (8)       // Param len (max) (power of 2)

#define BAUD_RATE   (19200UL) // UART speed
#define UBRR_VALUE	(F_CPU / (16UL * BAUD_RATE)) - 1UL

// Serial Data Available
#define SDA()		(UCSR0A & _BV(RXC0))

// Serial Buffer Available
#define SBA()		(UCSR0A & _BV(UDRE0))

// Serial status
#define STA()		(UCSR0A & (_BV(FE0)|_BV(DOR0)|_BV(UPE0))) 

unsigned char cmd[ CMDLEN ];              // Command received
unsigned char param[ PARAMLEN + CMDLEN ]; // Param received

#define PARAM_NOTEMPTY()    (param[0] != SpecialChars::SZERO)
#define PARAM_EMPTY()       (param[0] == SpecialChars::SZERO)

/*******************************************************************************
*
* B I T  M A N I P U L A T I O N
* 
*******************************************************************************/
#define _BR(bit, reg)		((reg) &= ~_BV(bit))		     // Bit Reset
#define IS_BIT_SET(v, b)    (((v) >> (b)) & Bool::ACTIVE) // Is Bit Set : testing bit b into v

#define lowByte(v) 			((uint8_t) ((v) & 0xff))    // Get low of word
#define highByte(v) 		((uint8_t) ((v) >> 8))      // get high of word
#define CAST16_T(v)         ((uint16_t) ((v) & 0xffff)) // cast value to 16 bits
#define BOOL(v)             ((bool) (lowByte(v) & Bool::ACTIVE))
#define NOT(v)              ((~(v)) & Bool::ACTIVE)

/*******************************************************************************
*
* U S E R F U L  D E F I N E
* 
*******************************************************************************/
#define DECODE(c1,c2)       ((c2 << 8) | c1)
#define ENCODE(a)			(a[0] | (a[1] << 8))

#define _toLower(c)			((c) | 0x20)
#define _toUpper(c)			((c) & 0xdf)    // ==> (c) & ~0x20

#define _isHexChar(c)       (_toLower(c) >= 'a' && _toLower(c) <= 'f')
#define _isDigit(c)			((c) >= '0' && (c) <= '9')
//#define _isDigit(c)			((c ^ '0') < 10)
#define toHexChar(i)        (i>9) ? ('A' + (i-10)) : ('0' + i)

#define max(x, y)           (x) > (y) ? (x) : (y)
#define min(x,y)            (x) < (y) ? (x) : (y)
#define isOdd(x)            ((x) & 1) 

#define isPowerOf2(m)       (m) && !((m) & ((m) - 1))
#define quickMod(n,m)       (n) & ((m) - 1)
#define quickMod2(n,m)      isPowerOf2(m) ? quickMod(n,m) : n % m

// Perform int absolue value
#define quickAbs(v)         (((v) - ((v) + (v))) & ((v) >> 31))

#define _C2V(c)				(((c) & 7) + 9)	// ascii char to hex value. Check before that c is a char
#define _V2V(c)				((c) & 0xf)		// ascii digit to value. Ckeck before that c is a number

#define ENABLE_TIMER0()		TIMSK0|=_BV(OCIE0A);TCCR0B|=_BV(CS02)
#define DISABLE_TIMER0()    _BR(OCIE0A, TIMSK0); _BR(CS02, TCCR0B)

#define ENABLE_TIMER1()		TIMSK1|=_BV(OCIE1A);TCCR1B|=_BV(CS10)
#define DISABLE_TIMER1()    _BR(OCIE1A, TIMSK1); _BR(CS10, TCCR1B)

#define ENABLE_TIMER2()		TIMSK2|=_BV(OCIE2A);TCCR2B|=_BV(CS22)
#define DISABLE_TIMER2()	_BR(OCIE2A, TIMSK2); _BR(CS22, TCCR2B)

#define DEBUGMESSAGE(s, v, l) \
    SerialTransmitString((string_t)s); \
    sendString(v,l); \
    SerialTransmitString((string_t)"\n");
/*******************************************************************************
*
* F O C U S E R  L E D
* 
*******************************************************************************/
#define PIN_LED_MOTOR_RUN	(0x10)
#define BUILTINLED			_BV(PB5) 	// Builtin led on board (0x20)

#define FOCUSERLEDBLINK()   PORTD ^= PIN_LED_MOTOR_RUN // Blink focuser LED
#define FOCUSERLEDOFF()     PORTD &= ~PIN_LED_MOTOR_RUN// Turn off focuser LED
#define FOCUSERLEDON()      PORTD |= PIN_LED_MOTOR_RUN // Turn on focuser LED

#define FOCUSERLEDBIT           1       // b00000010
#define FOCUSERLEDBLINKBIT      0       // b00000001
#define FOCUSERLEDFREQBIT       0xFC    // b11111100
#define DELAY_PULSE_FOCUSER_LED 512

//volatile time_t timeSinceLastBlink;
uint16_t focuserLedFreqBlk = 0; // Frequence blink
volatile time_t timer1_counter;

/*******************************************************************************
*
* Application Version
* 
*******************************************************************************/
// define 'ascii' version (not numeric)
//#define VERSION_FULL    (('0' + Version::MAJOR) << 8) | ('0' + Version::MINOR)

/*******************************************************************************
*
* E E P R O M  S E C T I O N
* 
*******************************************************************************/
/*
 * +----------------+-----+-----------------------------------------------------+
 * | Key            | idx | Description                                         |
 * +----------------+-----+-----------------------------------------------------+
 * | E_MAGIC        |  0  | EEPROM Version (uint8_t) : 1 byte                   |
 * | E_POWER        |  1  | Motor powored (uint8_t) : 1 byte                    |
 * | E_CURRENTPOS   |  2  | Current focuser position (uint32_t) : 4 bytes       |
 * | E_SHIFT        |  6  | anti tilt feature (uint8_t) : 1 byte                |
 * | E_SHIFTVALUE   |  7  | tilt compensate value (uint16_t) : 2 bytes          |
 * | E_SHIFTDIR     |  9  | tilt direction (uint8_t) : 1 byte                   |
 * | E_LED          | 10  | state of led (uint8_t) : 1 byte                     |
 * | E_LEDBLINK     | 11  | Led blinking state (uint8_t) : 1 byte               |
 * | E_SHIFTINIT    | 12  | Enable/disable shift init (uint8_t) : 1 byte        |
 * | E_BACKLASH     | 13  | Enable/disable backlash (uint8_t) : 1 byte          |
 * | E_BACKLASHIN   | 14  | Backlash value (uint16_t) : 2 bytes                 |
 * | E_LEDFREQ      | 16  | Led blinking frequence (uint16_t) : 2 bytes         |
 * | E_MOTORSTEP    | 18  | Motor step (uint8_t) : 1 byte                       |
 * | E_BACKLASHOUT  | 19  | Backlash value (uint16_t) : 2 bytes                 |
 * | E_INVERSE      | 21  | Inverse direction                                   |
 * +----------------+-----+-----------------------------------------------------+
 */
enum EepromIndex { 
    E_MAGIC          = 0,
    E_POWER          = 1,
    E_CURRENTPOS     = 2,
    E_SHIFT          = 6,
    E_SHIFTVALUE     = 7,
    E_SHIFTDIR       = 9,
    E_LED            = 10,
    E_LEDBLINK       = 11,
    E_SHIFTINIT      = 12,
    E_BACKLASH       = 13,
    E_BACKLASHIN     = 14,
    E_LEDFREQ        = 16,
    E_MOTORSTEP      = 18,
    E_BACKLASHOUT    = 19,
    E_INVERSE        = 21
};

/*******************************************************************************
*
* O P T I O N  S E C T I O N
* 
*******************************************************************************/

/*******************************************************************************
* +-----------+---+-----------------------+------------------------------------+
* | Key		  |	  | Description			  | Comments                           +
* +-----------+---+-----------------------+------------------------------------+
* | LED		  | LD| Led state 			  | 0 led off, 1 led on                |
* | LEDBLINK  | LB| Led blink state       | 0 blink inactive, 1 blink active   |
* | RUNNING	  | RU| Motor state			  | 0 motor sleep, 1 motor run         |
* | POWER	  | PW| Motot powered state	  | 0 motor unpowered, 1 powered       |
* | FOCUSDIR  | FI| Motor direction		  | 0 OUTWARD, 1 INWARD                |
* | INVERSE	  | IV| Inverse direction     | 0 no, 1 yes                        |
* | -----	  | --|                       |                                    |
* | DATA      | FD| Use during main loop  | 0 no new datas, 1 new datas        |
* | SHIFT	  | SH| shift state			  | 0 shift inactive, 1 shift active   |
* | SHIFTREQ  | SR| shift done state	  | 0 shift inactive, 1 shift done     |
* | SHIFTDIR  | SD| shift direction		  | 0 shift OUTWARD, 1 shift INWARD    |
* | SHIFTINIT | SI| Move mirror at init   | 0 init inactive, 1 init active     |
* | BACKLASH  | BL| Backlash compensation | 0 backlash inactive, 1 active      |
* | FULLSTEP  | FS| Motor full step       | 0 : halfstep, 1 : fullstep         |
* | POSREACH  | PR| Position reach        | 0 no, 1 yes                        |
* +-----------+---+-----------------------+------------------------------------+
*
*   15   14   13   12   11   10    9    8    7    6    5    4    3    2    1    0
* +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
* | -- | PR | FS | BL | SI | SD | SR | SH | FD | -- | IV | FI | PW | RU | LB | LD |
* +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+

**********************************************************************************/
enum Option { 
    LED         = 0,
    LEDBLINK    = 1,
    RUNNING     = 2,
    POWER       = 3,
    FOCUSDIR    = 4,
    INVERSE     = 5,
    //ISDDOT      = 5,
    //DATA        = 6,
    DATA        = 7,
    SHIFT       = 8,
    SHIFTREQ    = 9,
    SHIFTDIR    = 10,
    SHIFTINIT   = 11,
    BACKLASH    = 12,
    FULLSTEP    = 13,
    POSREACH    = 14

};

volatile uint16_t options;

#define RESET_OPTIONS()     options = 0
#define SET_OPTION(o)       options |=_BV(o)
#define UNSET_OPTION(o)     options &= ~_BV(o)

#define SET_OPTION2(o, v)   if((v) & Bool::ACTIVE){SET_OPTION(o);}else{UNSET_OPTION(o);}

// Test if option is set
#define GET_OPTION(o)       (options >> (o))
#define ISSET_OPTION(o)     (GET_OPTION(o) & Bool::ACTIVE)

#define ISNOTSET_OPTION(o)  (ISSET_OPTION(o) ^ Bool::ACTIVE)

#define GET_OPTION_DIR(o)   (Direction)((Direction::FOCUS_OUTWARD) + ISSET_OPTION(o))
#define GET_CURRENT_DIR()   GET_OPTION_DIR(Option::FOCUSDIR)

/*******************************************************************************
*
* M O T O R  S E C T I O N
* 
*******************************************************************************/
volatile pos_t currentPos;
volatile pos_t targetPos;
uint8_t motorStep;              // 0, half step, 1 full step (2 coils), 2 full step(1 coil)
int8_t stepIncrement;           // Must be 1 or -1

#define STEPPER_DISABLEDELAY    (10)         // Delay (in ms) before shutdown the motor
#define MAX_STEP                (0x3CAFE)    // Max step value
#define DEFAULT_FOCUSER_POS     (0x1CAFE)    // Default focuser position

#define ALLMOTORPIN_UP		_BV(PB0)|_BV(PB1)|_BV(PB2)|_BV(PB3)
#define ALLMOTORPIN_DOWN	~_BV(PB0)&~_BV(PB1)&~_BV(PB2)&~_BV(PB3)

#define MOTORPOWERON()		PORTB |= ALLMOTORPIN_UP
#define MOTORPOWEROFF()	    PORTB &= ALLMOTORPIN_DOWN

#define SETMOTORSTATE()     if (ISSET_OPTION(Option::POWER)){MOTORPOWERON();}else{MOTORPOWEROFF();}

struct MotorParams {
    int8_t seqSize;
    int8_t sequence[ 8 ];
} paramsMotor[3] = {
    {7, {0x8, 0xC, 0x4, 0x6, 0x2, 0x3, 0x1, 0x9}},// halfstep 4076 step
    {3, {0xC, 0x6, 0x3, 0x9}},                    // fullstep 2038 step - 2 coils
    {3, {0x8, 0x4, 0x2, 0x1}}                     // Fullstep 2038 step - 1 coils
};

/*******************************************************************************
*
* B A C K L A S H
* 
*******************************************************************************/
uint16_t backlashValueInward;
uint16_t backlashValueOutward;

/*******************************************************************************
*
* 
* 
*******************************************************************************/
// Shifting value
uint16_t shiftValue; // Not need to be volatile because she doesn't change

#define HAS_SHIFTING()      ISSET_OPTION(Option::SHIFT) && shiftValue > 0
#define SHIFTING_REQUEST()  ((HAS_SHIFTING()) && ISSET_OPTION(Option::SHIFTREQ) )

/*******************************************************************************
 *
 * W A TC H D O G  F U N C T I O N
 *
 *******************************************************************************/
#define watchdog_reset()    MCUCR |= _BV(WDRF)

/*******************************************************************************
*
* Functions declaration
* 
*******************************************************************************/
int main(int argc, char *argv[]);

void initSerial();
void initTimers();
void initHardware();
void readEEPROM(const bool forceInit);
void initEEPROM();
void readBasicEEPROM();
void initShifting();
void doShifting();

// Interrupt Service Routine
//ISR(TIMER0_COMPA_vect);
ISR(TIMER1_COMPA_vect);
ISR(TIMER2_COMPA_vect);
ISR(USART_RX_vect);

// specials function inline
INLINE void valueToHexaString(uint8_t * response, const pos_t v, const Digits d) FORCE_INLINE;

INLINE void SerialTransmitString(string_t s) 	        FORCE_INLINE;
INLINE void sendString(const pos_t v, const Digits d)   FORCE_INLINE;
INLINE void setTargetPosition(const pos_t target)       FORCE_INLINE;
INLINE pos_t HexToInt()                                 FORCE_INLINE;
INLINE pos_t qabs(int32_t a)                            FORCE_INLINE;
INLINE Direction computeDirection(const pos_t current, const pos_t target) FORCE_INLINE;
#endif
