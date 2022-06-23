/*
 ███╗   ███╗ ██████╗  ██████╗ ███╗   ██╗██████╗ ██╗   ██╗██╗███╗   ██╗ ██████╗           
████╗ ████║██╔═══██╗██╔═══██╗████╗  ██║██╔══██╗██║   ██║██║████╗  ██║██╔═══██╗          
██╔████╔██║██║   ██║██║   ██║██╔██╗ ██║██║  ██║██║   ██║██║██╔██╗ ██║██║   ██║          
██║╚██╔╝██║██║   ██║██║   ██║██║╚██╗██║██║  ██║██║   ██║██║██║╚██╗██║██║   ██║          
██║ ╚═╝ ██║╚██████╔╝╚██████╔╝██║ ╚████║██████╔╝╚██████╔╝██║██║ ╚████║╚██████╔╝          
╚═╝     ╚═╝ ╚═════╝  ╚═════╝ ╚═╝  ╚═══╝╚═════╝  ╚═════╝ ╚═╝╚═╝  ╚═══╝ ╚═════╝           
                                                                                        
██████╗ ██╗   ██╗     █████╗ ██╗     ██████╗ ██╗  ██╗ █████╗ ███╗   ███╗ █████╗ ██╗  ██╗
██╔══██╗╚██╗ ██╔╝    ██╔══██╗██║     ██╔══██╗██║  ██║██╔══██╗████╗ ████║██╔══██╗╚██╗██╔╝
██████╔╝ ╚████╔╝     ███████║██║     ██████╔╝███████║███████║██╔████╔██║███████║ ╚███╔╝ 
██╔══██╗  ╚██╔╝      ██╔══██║██║     ██╔═══╝ ██╔══██║██╔══██║██║╚██╔╝██║██╔══██║ ██╔██╗ 
██████╔╝   ██║       ██║  ██║███████╗██║     ██║  ██║██║  ██║██║ ╚═╝ ██║██║  ██║██╔╝ ██╗
╚═════╝    ╚═╝       ╚═╝  ╚═╝╚══════╝╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝
 
 * 30/04/2020 V6.8	 Full C version
 * 04/07/2020 V6.9   Bluetooth communication module available ==> ABORDED
 * 09/11/2020 V7.1   Use 32 bits for position
 * 17/01/2021 V8.0	 use vector interrupt for uart and motor
 * 					 remove speed control (no need)
 * 					 compute ratio between small and large wheel
 * 20/04/2021 V8.2   remove modulo computing to motor run
 * 					 add cli() & cei() in uart initialization
 * 					 add sl command for led control
 * 					 minor change in hextoint method
 * 					 disable interrupt OCIE2A after motor down
 * 					 change MOD4WIRE type from pos_t to uint8_t
 * 					 minor bug in _isAlpha() method
 *					 change freq of timer 0
 *					 add getCurrentPos() for read value in blocking section
 * 04/07/2021 V8.3	 Use avr/eeprom rather than EEProm lib
 * 27/07/2021 V8.4	 Add some features for mirror shifting
 * 					 Changes options var to 16 bits
 * 					 add setCurrentPos() for write value in blocking section
 * 12/09/2021 V8.5	 replace all sendingX() by sendString() 
 * 					 capability to remove shifting / led code
 * 16/10/2021 V8.6   Add backlash compensate	
 * 03/01/2022 V8.7	 Add motor speed capability		
 * 					 Add led frequency independante modification	
 * 01/02/2022 V8.8	 change hexToInt(), add doShifting() procedure, 
 * 					 change DECODE function	 
 * 01/03/2022 V8.9	 define timer 0 for led frequency. split backlash into
 * 					 2 values (inward & outward). Change SM command to SI
 * 					 and GM to GI
 * 					 check if focuser move in setTargetPosition()
 * 01/06/2022 V8.10	 some improvement
 */

/*
	 Command					Return				Comment
	 Char #						Value
1 2 3 4 5 6 7 8 9 A B C D E F
: F G #							N/A			Go to the new position as set by the ":SNYYYYY#" command.
: F Q #							N/A			Immediately stop any focus motor movement.
: G D #							XX#			Return the current direction
: G I #							00# or 01#	Return "00#" if the focus motor is not moving,
											otherwise return "01#"
: G N #							YYYYY#		Return the new position previously set by a
											":SNYYYYY" command where YYYYY is a five-digit
											unsigned hex number.
: G P #							YYYYY#		Return the current position where YYYYY is a
											five-digit unsigned hex number.
: G V #							DD#			Get the version of the firmware as a two-digit
											decimal number where the first digit is the major
											version number, and the second digit is the minor
											version number.
###: S D Y #						N/A			Set the new direction. 0 outward, 1 inward
: S N Y Y Y Y Y #				N/A			Set the new position where YYYYY is a five-digit
											unsigned hex number.
: S P Y Y Y Y Y #				N/A			Set the current position where YYYYY is a five-digit
: S H Y Y #						N/A			Set motor steping. 0 for half step, 1 for full step
: G H #							YY#			Return 1 for half step, 0 for full step

#### S P E C I A L S  F E A T U R E S ####
: R A #							N/A			reset eeprom value to default value
: S L Y #						N/A			Set Led. 0 : off, 1 : on, 3 : blinking
: G L #							YY#			return focuser led status. O0# off, 01# on, 03# on & blink
: S T Y #						N/A			Set to 1 to power motor on, 0 for off
: G T #							YY#			Return motor powered status. 00# power off, 01# power on
###: S R Y Y Y #					N/A			Set number of round from current position on power of 2
###: G R # 						YYYY#		Return number of step by round
: G W #							YYYY#		Return the focuser status (option variable)
: S F Y Y Y Y #					N/A			Set led frequency value. Range to 0 and 255
: S J #							YY#			Inverse direction
: G J #							N/A			Return 1 if direction inversed
: B T #							N/A			Backlash test

#### M i r r o r  s h i f t  f e a t u r e s ####
: S A Y Y #						N/A			Set 1 to activate mirror shift feature, 0 for off
: S B Y Y Y Y #					N/A			Set the new mirror shift where XXXX is the five digit
											unsigned hex number
: S C Y Y # 					N/A			Set mirror shift direction. 1 for inward, 0 for outward
: S S Y Y # 					N/A			Enable or disable init shift at startup
: R E #							N/A			Run immediatly initshifting procedure
: G A #							00# or 01#	Return if mirror shift activavted
: G B #							YYYY# 		Return mirror shift value (integer number)
: G C #							00# or 01# 	Return 0 for outward, 1 for inward
: G S #							00# or 01# 	Return if shift at init is enable

#### B a c k l a s h  f e a t u r e s ####
: S K Y Y #						N/A			Set to 1 to enable backlash, 0 to disable
: S M Y Y Y Y #					N/A			Set the new inward backlash value
: S O Y Y Y Y #					N/A			Set the new outward backlash value
: G K #							00# or 01#	Return backlash enable status
: G M #							YYYY#		Return backlash value

Most command must be set when only the motor he is in sleeping mode.
*/

#include "moonduino.h"

/*******************************************************************************
 * 
 * 
 * 
 * 
*******************************************************************************/
int main(int argc, char *argv[])
{
	initHardware();
	readEEPROM(false);
	initTimers();
	initSerial();
	initShifting();

	// Main loop
	while(1)
	{
		// All data readed ?
		if (ISSET_OPTION(Option::DATA))
		{
			UNSET_OPTION(Option::DATA);

			switch (ENCODE(cmd))
			{
				//--------------------------------------------------------------
				// Return current position
				//--------------------------------------------------------------
				case DECODE('g', 'p'):
				{
					sendString(currentPos, Digits::FIVE_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// Return the focuser options status
				//--------------------------------------------------------------
				case DECODE('g', 'w'):
				{
					sendString(options, Digits::FOUR_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// set position to reach
				//--------------------------------------------------------------
				case DECODE('s', 'n'):
				{
					// Only set new position if not running
					if (ISNOTSET_OPTION(Option::RUNNING) && PARAM_NOTEMPTY())
					{
						pos_t pos = HexToInt();

						if (pos != targetPos)
						{
							setTargetPosition(pos);
						}
					}

					break;
				}

				//--------------------------------------------------------------
				// turn motor on
				//--------------------------------------------------------------
				case DECODE('f', 'g'):
				{
					if(ISNOTSET_OPTION(Option::RUNNING))
					{
						SET_OPTION(Option::RUNNING);
						UNSET_OPTION(Option::POSREACH);

						ENABLE_TIMER2();

						if (ISSET_OPTION(Option::LED))
						{
							FOCUSERLEDON();

							if (ISSET_OPTION(Option::LEDBLINK) && focuserLedFreqBlk > 0)
							{
								timer1_counter = 0;
								ENABLE_TIMER0();
							}
						}
					}
					
					break;
				}

				//--------------------------------------------------------------
				// get is running.
				// Return 0 if not running, 1 running
				//--------------------------------------------------------------
				case DECODE( 'g', 'i' ):
				{
					sendString(ISSET_OPTION(Option::RUNNING), Digits::TWO_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// get firmware version
				//--------------------------------------------------------------
				case DECODE('g', 'v'):
				{
					uint8_t response[4];

					response[0] = '0' + Version::MAJOR;
					response[1] = '0' + Version::MINOR;
					response[2] = SpecialChars::HASH;
					response[3] = SpecialChars::SZERO;

					SerialTransmitString(response);

					break;
				}

				//--------------------------------------------------------------
				// Return the led status : 0 : off, 2 : on : 3 : on and blink
				//--------------------------------------------------------------
				case DECODE( 'g', 'l' ):
				{
					/*
					* Compute led status like this :
					* +----+----+----+----+----+----+----+----+
					* | -- | -- | -- | -- | -- | -- | ls | bs |
					* +----+----+----+----+----+----+----+----+
					* 
					* bit 7-2 not used
					* bit 1   led status
					*	the ls bit is set (one) when the focuser led is on. Clear this bit
					*	for off the led.
					* bit 0   blinking status
					*	the bs bit is set (one) when blinking led is on. Clear this bit
					*	for stop blinking led (the led may stay on).
					*/
					uint8_t gl = lowByte(
									ISSET_OPTION(Option::LED) << 1 
									|
									ISSET_OPTION(Option::LEDBLINK)
								);

					sendString(gl, Digits::TWO_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// set power led. 0 inactive, 1 active, 3 active & blink
				//--------------------------------------------------------------
				case DECODE( 's', 'l' ):
				{
					if (PARAM_EMPTY())
						break;

					bool bit;

					/*
					* Compute led status like this :
					* bit 1 - ls : led status
					*	the ls bit is set (one) when the focuser led is on. Clear this bit
					*	for off the led.
					* bit 0 - bs : blinking status
					*	the bs bit is set (one) when blinking led is on. Clear this bit
					*	for stop blinking led (the led may stay on).
					*/
					uint8_t sl = lowByte(HexToInt());

					// Led status
					bit = IS_BIT_SET(sl, FOCUSERLEDBIT);

					// If changed status
					if (ISSET_OPTION(Option::LED) != bit)
					{
						SET_OPTION2(Option::LED, bit);

						if (!bit)
							FOCUSERLEDOFF();
						else
							FOCUSERLEDON();

						eeprom_update_byte((uint8_t *)EepromIndex::E_LED, bit);
					}

					// Blinking status
					bit = IS_BIT_SET(sl, FOCUSERLEDBLINKBIT);

					if (ISSET_OPTION(Option::LEDBLINK) != bit)
					{
						SET_OPTION2(Option::LEDBLINK, bit);

						eeprom_update_byte((uint8_t *)EepromIndex::E_LEDBLINK, bit);
					}

					break;
				}

				//--------------------------------------------------------------
				// set led frequency.
				// 
				//--------------------------------------------------------------
				case DECODE( 's', 'f' ):
				{
					if (PARAM_NOTEMPTY())
					{
						uint16_t freq = CAST16_T(HexToInt());

						if (freq != focuserLedFreqBlk || freq == 0)
						{
							focuserLedFreqBlk = freq;
							timer1_counter = 0;

							eeprom_write_word((uint16_t *)EepromIndex::E_LEDFREQ, freq);
						}

						if (freq == 0)
							UNSET_OPTION(Option::LEDBLINK);
					}

					break;
				}

				//--------------------------------------------------------------
				// Return the 4 digits led frequency
				//--------------------------------------------------------------
				case DECODE( 'g', 'f'):
				{
					sendString(focuserLedFreqBlk, Digits::FOUR_DIGITS);

					break;
				}

/*
				//--------------------------------------------------------------
				// Return the 5 digits new motor position
				//--------------------------------------------------------------
				case DECODE( 'g', 'n'):
				{
					sendString(targetPos, Digits::FIVE_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// get motor direction. 0 outward, 1 inward
				//--------------------------------------------------------------
				case DECODE( 'g', 'd' ):
				{
					sendString(GET_OPTION_DIR(Option::FOCUSDIR), Digits::TWO_DIGITS);

					break;
				}
*/
				//--------------------------------------------------------------
				// get power motor status. 0 off, 1 on
				//--------------------------------------------------------------
				case DECODE( 'g', 't' ):
				{
					sendString(ISSET_OPTION(Option::POWER), Digits::TWO_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// get shift status
				//--------------------------------------------------------------
				case DECODE( 'g', 'a' ):
				{
					sendString(ISSET_OPTION(Option::SHIFT), Digits::TWO_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// get shift value
				//--------------------------------------------------------------
				case DECODE( 'g', 'b' ):
				{
					sendString(shiftValue, Digits::FOUR_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// get shift direction
				//--------------------------------------------------------------
				case DECODE( 'g', 'c' ):
				{
					sendString(GET_OPTION_DIR(Option::SHIFTDIR), Digits::TWO_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// get status shifting at init
				//--------------------------------------------------------------
				case DECODE( 'g', 's' ):
				{
					sendString(ISSET_OPTION(Option::SHIFTINIT), Digits::TWO_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// stop motor
				//--------------------------------------------------------------
				case DECODE( 'f', 'q' ):
				{
					if(ISSET_OPTION(Option::RUNNING))
					{
						targetPos = currentPos;

						UNSET_OPTION(Option::RUNNING);
						UNSET_OPTION(Option::SHIFTREQ);

						SET_OPTION(Option::POSREACH);
					}

					break;
				}
				
				//--------------------------------------------------------------
				// set direction. "00" for OUTWARD, "01" for INWARD
				//--------------------------------------------------------------
/*
				case DECODE( "sd" ):
				{
					if( ISNOTSET_OPTION( Option::RUNNING ) )
					{
						bool sd = BOOL(HexToInt());

							SET_OPTION2(Option::FOCUSDIR, sd);
					}

					break;
				}
*/					
				//--------------------------------------------------------------
				// set position
				// We not use get/set currentPos because motor doesn't run and
				// value doesn't change
				//--------------------------------------------------------------
				case DECODE( 's', 'p' ):
				{
					if (ISNOTSET_OPTION(Option::RUNNING) && PARAM_NOTEMPTY())
					{
						pos_t sp = HexToInt();

						if( sp != currentPos )
						{
							targetPos = currentPos = sp;

							eeprom_update_dword((pos_t *) EepromIndex::E_CURRENTPOS, sp);
						}
					}

					break;
				}

				//--------------------------------------------------------------
				// set power motor
				//--------------------------------------------------------------
				case DECODE( 's', 't' ):
				{
					if( ISNOTSET_OPTION( Option::RUNNING ) && PARAM_NOTEMPTY())
					{
						bool power = BOOL(HexToInt());

						if (ISSET_OPTION(Option::POWER) != power)
						{
							SET_OPTION2(Option::POWER, power);

							SETMOTORSTATE();

							eeprom_update_byte((uint8_t *)EepromIndex::E_POWER, power);
						}
					}
					
					break;
				}

				//--------------------------------------------------------------
				// set shift compensate
				//--------------------------------------------------------------
				case DECODE( 's', 'a' ):
				{
					if( ISNOTSET_OPTION( Option::RUNNING ) && PARAM_NOTEMPTY())
					{
						bool shift = BOOL(HexToInt());

						if (ISSET_OPTION(Option::SHIFT) != shift)
						{
							SET_OPTION2(Option::SHIFT, shift);

							eeprom_update_byte((uint8_t *)EepromIndex::E_SHIFT, shift);
						}
					}

					break;
				}

				//--------------------------------------------------------------
				// set shift compensate value
				//--------------------------------------------------------------
				case DECODE( 's', 'b' ):
				{
					if( ISNOTSET_OPTION( Option::RUNNING ) && PARAM_NOTEMPTY())
					{
						uint16_t _shiftValue = CAST16_T(HexToInt());

						if(_shiftValue != shiftValue)
						{
							shiftValue = _shiftValue;

							eeprom_update_word((uint16_t *)EepromIndex::E_SHIFTVALUE, _shiftValue);
						}
					}

					break;
				}

				//--------------------------------------------------------------
				// set shift direction
				//--------------------------------------------------------------
				case DECODE( 's', 'c' ):
				{
					if( ISNOTSET_OPTION( Option::RUNNING ) && PARAM_NOTEMPTY())
					{
						bool dir = BOOL(HexToInt());

						if (ISSET_OPTION(Option::SHIFTDIR) != dir)
						{
							SET_OPTION2(Option::SHIFTDIR, dir);

							eeprom_write_byte((uint8_t *)EepromIndex::E_SHIFTDIR, dir);
						}
					}

					break;
				}

				//--------------------------------------------------------------
				// enable / disable shifting at init
				//--------------------------------------------------------------
				case DECODE( 's', 's' ):
				{
					if (PARAM_NOTEMPTY())
					{
						bool ss = BOOL(HexToInt());

						if (ISSET_OPTION(Option::SHIFTINIT) != ss)
						{
							SET_OPTION2(Option::SHIFTINIT, ss);

							eeprom_write_byte((uint8_t *)EepromIndex::E_SHIFTINIT, ss);
						}
					}

					break;
				}

				//---------------------------------------------------
				// Special feature RESET arduino
				//---------------------------------------------------
				case DECODE('r', 'a'):
				{
					readEEPROM(true);

					break;
				}

				//--------------------------------------------------------------
				// Enable/disable backlash
				//--------------------------------------------------------------
				case DECODE('s', 'k'):
				{
					// Only set new position if not running
					if(ISNOTSET_OPTION(Option::RUNNING) && PARAM_NOTEMPTY())
					{
						bool backlash = BOOL(HexToInt());

						if (ISSET_OPTION(Option::BACKLASH) != backlash)
						{
							SET_OPTION2(Option::BACKLASH, backlash);

							eeprom_write_byte((uint8_t *)EepromIndex::E_BACKLASH, backlash);
						}
					}

					break;
				}

				//--------------------------------------------------------------
				// set backlash value inward
				//--------------------------------------------------------------
				case DECODE('s', 'm'):
				{
					if (ISNOTSET_OPTION(Option::RUNNING) && PARAM_NOTEMPTY())
					{
						uint16_t backlashInward = CAST16_T(HexToInt());

						if(backlashInward != backlashValueInward)
						{
							backlashValueInward = backlashInward;

							eeprom_write_word((uint16_t *)EepromIndex::E_BACKLASHIN, backlashInward);
						}
					}

					break;
	
				}

				//--------------------------------------------------------------
				// set backlash value outward
				//--------------------------------------------------------------
				case DECODE('s', 'o'):
				{
					if (ISNOTSET_OPTION(Option::RUNNING) && PARAM_NOTEMPTY())
					{
						uint16_t backlashOutward = CAST16_T(HexToInt());

						if (backlashOutward != backlashValueOutward)
						{
							backlashValueOutward = backlashOutward;

							eeprom_write_word((uint16_t *)EepromIndex::E_BACKLASHOUT, backlashOutward);
						}
					}

					break;
				}

				//--------------------------------------------------------------
				// Return 1 if backlash is enable, 0 if disable
				//--------------------------------------------------------------
				case DECODE('g', 'k'):
				{
					sendString(ISSET_OPTION(Option::BACKLASH), Digits::TWO_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// Return the backlash value
				//--------------------------------------------------------------
				case DECODE('g', 'm'):
				{
					sendString(backlashValueInward, Digits::FOUR_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// Return the backlash value
				//--------------------------------------------------------------
				case DECODE('g', 'o'):
				{
					sendString(backlashValueOutward, Digits::FOUR_DIGITS);

					break;
				}

				//--------------------------------------------------------------
				// Set motor step. 0 half step, 1 full step (2 coils)
				//--------------------------------------------------------------
				case DECODE('s', 'h'):
				{
					if(ISNOTSET_OPTION(Option::RUNNING) && PARAM_NOTEMPTY())
					{
						motorStep = BOOL(HexToInt());

						if (BOOL(GET_OPTION(Option::FULLSTEP)) != motorStep)
						{
							SET_OPTION2(Option::FULLSTEP, motorStep);

							eeprom_write_byte((uint8_t *)EepromIndex::E_MOTORSTEP, motorStep);
						}
					}

					break;
				}

				//--------------------------------------------------------------
				// Return the motor speed
				//--------------------------------------------------------------
				case DECODE('g', 'h'):
				{
					sendString(ISSET_OPTION(Option::FULLSTEP), Digits::TWO_DIGITS);
					
					break;
				}

				//--------------------------------------------------------------
				// force init shifting procedure
				//--------------------------------------------------------------
				case DECODE('r', 'e'):
				{
					doShifting();

					break;
				}

				//--------------------------------------------------------------
				// Test backlash procedure
				//--------------------------------------------------------------
				case DECODE('t', 'b'):
				{
					break;
				}

				//--------------------------------------------------------------
				// Set inverse direction
				//--------------------------------------------------------------
				case DECODE('s', 'j'):
				{
/*					if (ISNOTSET_OPTION(Option::RUNNING))
					{
						bool inverse = BOOL(HexToInt());

						if(GET_OPTION(Option::INVERSE) != inverse)
						{
							SET_OPTION2(Option::INVERSE, inverse);

							eeprom_write_byte((uint8_t *)EepromIndex::E_INVERSE, inverse);
						}
					}
					
*/					break;
				}

				//--------------------------------------------------------------
				// Return inverse value
				//--------------------------------------------------------------
				case DECODE('g', 'j'):
				{
					sendString(ISSET_OPTION(Option::INVERSE), Digits::TWO_DIGITS);

					break;
				}
			} // end switch
		} // end if
	} // end while
	
	return 0;
}

/*******************************************************************************
* Compute target destination with backlash + shifting
* 
* 
*******************************************************************************/
void setTargetPosition(const pos_t target)
{
	if(target < MAX_STEP)
	{
		// Compute the new focuser direction
		// If newDirection == FOCUS_INWARD ==> 1 (true),
		// FOCUS_OUTWARD ==> 0 (false),
		// NOT_MOVE ==> 2
		Direction newDirection = computeDirection(currentPos, target);

		if (newDirection != Direction::NOT_MOVE || SHIFTING_REQUEST())
		{
			Direction isShiftInward = GET_OPTION_DIR(Option::SHIFTDIR); 

			// If request shifting ? 
			uint8_t isFocuserMoveInward = (newDirection == Direction::NOT_MOVE) ? 
				NOT(isShiftInward) : 
				newDirection & Direction::FOCUS_INWARD; // Force all bit to 0 except bit 0

#if defined(DEBUG)
	DEBUGMESSAGE("Option::SHIFTDIR=", BOOL(GET_OPTION(Option::SHIFTDIR)), TWO_DIGITS);
	DEBUGMESSAGE("isFocuserMoveInward=", isFocuserMoveInward, TWO_DIGITS);
#endif

			targetPos = target;

			// Determines the motor step increment. Only used on motor ISR
			stepIncrement = (isFocuserMoveInward ? -1 : 1);

			// Apply bascklash only when change direction. Move currentPos in this case
			if (ISSET_OPTION(Option::BACKLASH) && newDirection != GET_CURRENT_DIR())
			{
				if (isFocuserMoveInward)
					currentPos += backlashValueOutward;
				else
					currentPos -= backlashValueInward;
			}

			// Add shift value compensation if focus direction is different to the shift direction
			if (ISSET_OPTION(Option::SHIFT) && newDirection != isShiftInward)
			{
//				targetPos = isFocuserMoveInward ? -shiftValue : shiftValue;
				
				if (isFocuserMoveInward)
					targetPos -= shiftValue;
				else
					targetPos += shiftValue;

				SET_OPTION(Option::SHIFTREQ);
			}

			SET_OPTION2(Option::FOCUSDIR, isFocuserMoveInward);
		}
	}
}

/*******************************************************************************
* Timer0 (8 bits) for Led blinking
* Timer2 (8 bits) for generate interrupt for motor run
* This code was generated by 
* 	http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
*******************************************************************************/
void initTimers()
{
	cli(); // stop interrupts

	/**************************************************************************
	 * TIMER 0 : every ~100hz
	 **************************************************************************/
	TCCR0A = 0; // set entire TCCR0A register to 0
	TCCR0B = 0; // same for TCCR0B
	TCNT0 = 0;	// initialize counter value to 0

	// set compare match register for 100.16025641025641 Hz increments
	OCR0A = 155; // = 16000000 / (1024 * 100.16025641025641) - 1 (must be <256)

	// turn on CTC mode
	TCCR0B |= (1 << WGM01);

	// Set CS02, CS01 and CS00 bits for 1024 prescaler
	TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);

	// doesnt enable timer compare interrupt now
	//TIMSK0 |= (1 << OCIE0A);

	/**************************************************************************
	 * TIMER 2 for interrupt frequency 1000 Hz:
	 **************************************************************************/
	TCCR2A = 0; // set entire TCCR2A register to 0
	TCCR2B = 0; // same for TCCR2B
	TCNT2 = 0;	// initialize counter value to 0

	// set compare match register for 1000 Hz increments
	OCR2A = 249; // = 16000000 / (64 * 1000) - 1 (must be <256)

	// turn on CTC mode
	TCCR2B |= (1 << WGM21);

	// Set CS22, CS21 and CS20 bits for 64 prescaler
	TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20);

	// doesnt enable timer compare interrupt now
	//TIMSK2 |= (1 << OCIE2A);

	sei();
}

/*******************************************************************************
*
*
*
*******************************************************************************/
void initSerial()
{
	cli();

	// UBRR0 = ubbr;	// see table 19.12
	//  Split in 2 registers for compatability
	UBRR0H = highByte(UBRR_VALUE);
	UBRR0L = lowByte(UBRR_VALUE);

	// Enable receive et transmit
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);

	// Enable interrupt on receive
	UCSR0B |= _BV(RXCIE0);

	// 8 bits character size, 1 bit stop, no parity
	UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);

	sei();
}

/*******************************************************************************
*
*
* 
*******************************************************************************/
void initHardware()
{
	cli();

	// output mode for the four motor pins, build in led
	DDRB = ALLMOTORPIN_UP | BUILTINLED;
	
	// output mode for focuser led
	DDRD |= PIN_LED_MOTOR_RUN;

	// turn-off board led
	PORTB ^= ~BUILTINLED;

	// set moonduino led 
	PORTD ^= ~PIN_LED_MOTOR_RUN; 	// set pin 4 to low

	// Unset PUD to MUCR : pull-up is enable
	_BR(PUD, MCUCR);
	
	// turn off power of analog comparator
	ACSR |= _BV(ACD);
	_BR(ADEN, ADCSRA);
	
	// turn off brown-out
	MCUCR |= _BV(BODS) | _BV(BODSE);

	// Power Reduction Register. disable all (set to 1)
	// except Timer0(PRTIM0), TIMER2(PRTIM2), UART(PRUSART0)
	PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRSPI) | _BV(PRADC); // disable all

	// Set watchdog
	//wdt_reset();

	//WDTCSR |= _BV(WDCE) | _BV(WDE);
	//WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP1);
	//wdt_enable(WDTO_30MS);

	sei();
}

/*******************************************************************************
 *
 *
 *
 ******************************************************************************/
/*
ISR(WDT_vect)
{
	SerialTransmitString((uint8_t *)"watchdog reset");
}
*/
/*******************************************************************************
 *
 * Read command start by ':' and finished by '#'.
 * 
 ******************************************************************************/
ISR(USART_RX_vect)
{
	static bool isddot = false;
	static bool hasData = false;
	static uint8_t idx = 0;

	unsigned char data = _toLower(UDR0);

	// Check error ?
	if (!STA())
	{
		if (data == SpecialChars::DDOT)
		{
			isddot = true;
			hasData = false;

			UNSET_OPTION(Option::DATA);
			param[ 0 ] = SpecialChars::SZERO;

			idx = 0;
		}
		else if (data == SpecialChars::HASH) // char ending string
		{
			isddot = false;

			// Test if command wasn't only DDOT + HASH
			if (hasData)
			{
				SET_OPTION(Option::DATA);

				// Check if has param
				if(idx > CMDLEN)
				{
					// Force last char to 0
					param[ idx - CMDLEN ] = SpecialChars::SZERO;
				}

				hasData = false;
			}
		}
		else // other char
		{
			if (isddot && idx < (CMDLEN + PARAMLEN))
			{
				// Check if data is a part of command
				if (idx < CMDLEN)
				{
					// Commands must be always in lowercase
					cmd[ idx ] = data;
					hasData = true;
				}
				else
				{
					param[idx - CMDLEN] = data;
				}

				idx++;
			}
		}
	}
}

/*******************************************************************************
*
* Simple timer counter for led blinking
*
*******************************************************************************/
ISR(TIMER1_COMPA_vect)
{
	timer1_counter++;

	if(timer1_counter >= focuserLedFreqBlk)
	{
		FOCUSERLEDBLINK();
		timer1_counter = 0;
	}
}

/*******************************************************************************
*
* interrupt for stepper
*
*******************************************************************************/
ISR(TIMER2_COMPA_vect)
{
	static int8_t motorSeq = 0;

	if (ISSET_OPTION(Option::RUNNING) && ISNOTSET_OPTION(Option::POSREACH))
	{
		// currentPos reach targetPos ?
		if(currentPos != targetPos)
		{
			// compute the next index of the sequence motor. use the quick modulo 
			motorSeq &= paramsMotor[ motorStep ].seqSize;

			// Move motor of one (or half one) step
			PORTB = paramsMotor[ motorStep ].sequence[ motorSeq ];

			currentPos += stepIncrement;

			motorSeq += stepIncrement;
		}
		else
			SET_OPTION(Option::POSREACH);
	}
	else // Position reach ou stop command
	{
		if(SHIFTING_REQUEST())
		{
			Direction shiftDir = GET_OPTION_DIR(Option::SHIFTDIR);

			// Focus direction is different to shift direction
			if (GET_CURRENT_DIR() != shiftDir)
			{
				// focuser go outward and shift direction is inward
				if (shiftDir & Direction::FOCUS_INWARD)
				{
					targetPos = targetPos - shiftValue;
					stepIncrement = -1;
					SET_OPTION(Option::FOCUSDIR);
				}
				else // focuser go inward and shift outward
				{
					targetPos = targetPos + shiftValue;
					stepIncrement = 1;
					UNSET_OPTION(Option::FOCUSDIR);
				}
				
				// Need to add backlash ?
				if (ISSET_OPTION(Option::BACKLASH))
				{
					if (shiftDir & Direction::FOCUS_INWARD)
						currentPos += backlashValueOutward;
					else
						currentPos -= backlashValueInward;
				}

				UNSET_OPTION(Option::POSREACH);
				UNSET_OPTION(Option::SHIFTREQ);

				SET_OPTION(Option::RUNNING); // in case of fq command
			}
		}
		else
		{
			SETMOTORSTATE();
			
			// Store current position in eeprom
			eeprom_update_dword((pos_t *)EepromIndex::E_CURRENTPOS, currentPos);

			UNSET_OPTION(Option::RUNNING);
			SET_OPTION(Option::POSREACH);

			DISABLE_TIMER2();

			if(ISSET_OPTION(Option::LED))
			{
				FOCUSERLEDOFF();

				if(ISSET_OPTION(Option::LEDBLINK) && focuserLedFreqBlk > 0)
				{
					DISABLE_TIMER1();
				}
			}
		}
	}
}

/*******************************************************************************
 * special implementation to convert string 'param' hex number to integer value
 * only string with DigitHex digits
 * never begin with 0x 
*******************************************************************************/
pos_t HexToInt()
{
	uint8_t i = 0, v, c;
	pos_t r = 0;

	while( (c = param[i++]) != SpecialChars::SZERO)
	{
		r = r << 4;

		if (_isDigit(c))
			v = _V2V(c);
		else if(_isHexChar(c))
			v = _C2V(c);
		else	
			v = 0;

		if(v)
			r += v;
	}

	return r;
}

/*******************************************************************************
* readEEPROM is read only at the boot. By this way, only change state of 
* variable active. By default, all are initialized at 0
* 
*******************************************************************************/
void readEEPROM(const bool forceInit)
{
	cli();

	UNSET_OPTION(Option::SHIFTREQ);	// Force option shiftdone
	SET_OPTION(Option::POSREACH);

	if(forceInit)	// Init eeprom
	{
		initEEPROM();
	}
	else
	{
		// Add specific eeprom variables for the new version
		if (eeprom_read_byte((uint8_t *)E_MAGIC) != Version::CURRENT_VERSION_EE)
		{
			eeprom_write_word((uint16_t *)EepromIndex::E_BACKLASHOUT, 0);

			eeprom_write_byte((uint8_t *)EepromIndex::E_INVERSE, Bool::FALSE);

			eeprom_write_byte((uint8_t *)EepromIndex::E_MAGIC, Version::CURRENT_VERSION_EE);
		}

		readBasicEEPROM();

		SETMOTORSTATE();
	}

	sei();
}

/*******************************************************************************
*
*
* 
*******************************************************************************/
void initEEPROM()
{
	RESET_OPTIONS();
	SET_OPTION(Option::SHIFTDIR);
	UNSET_OPTION(Option::SHIFTREQ); // unset shift request

	currentPos			= DEFAULT_FOCUSER_POS;
	targetPos			= DEFAULT_FOCUSER_POS;
	focuserLedFreqBlk	= DELAY_PULSE_FOCUSER_LED;
	motorStep			= MotorStep::FULL_STEP_2_COILS;
	backlashValueInward	= 0;
	backlashValueOutward= 0;
	shiftValue			= 0;

	eeprom_write_byte((uint8_t *)	EepromIndex::E_MAGIC, 		Version::CURRENT_VERSION_EE);
	eeprom_write_byte((uint8_t *)	EepromIndex::E_POWER, 		Bool::INACTIVE);
	eeprom_write_dword((uint32_t *)	EepromIndex::E_CURRENTPOS, 	DEFAULT_FOCUSER_POS);

	// Mirror shift compensator
	eeprom_write_byte((uint8_t *)	EepromIndex::E_SHIFT, 		Bool::INACTIVE);
	eeprom_write_word((uint16_t *)	EepromIndex::E_SHIFTVALUE, 	0);
	eeprom_write_byte((uint8_t *)	EepromIndex::E_SHIFTDIR, 	Direction::FOCUS_OUTWARD);

	// Disable shift init procedure at init
	eeprom_write_byte((uint8_t *)	EepromIndex::E_SHIFTINIT, 	Bool::INACTIVE);

	// Focuser LED
	eeprom_write_byte((uint8_t *)	EepromIndex::E_LED, 		Bool::INACTIVE);
	eeprom_write_byte((uint8_t *)	EepromIndex::E_LEDBLINK, 	Bool::INACTIVE);
	eeprom_write_word((uint16_t *)	EepromIndex::E_LEDFREQ, 	DELAY_PULSE_FOCUSER_LED);

	// Disable backlash compensation
	eeprom_write_byte((uint8_t *)	EepromIndex::E_BACKLASH, 	Bool::INACTIVE);
	eeprom_write_word((uint16_t *)	EepromIndex::E_BACKLASHIN,	0);
	eeprom_write_word((uint16_t *)	EepromIndex::E_BACKLASHOUT,	0);

	eeprom_write_byte((uint8_t *)EepromIndex::E_INVERSE, Bool::FALSE);

	eeprom_write_byte((uint8_t *)EepromIndex::E_MOTORSTEP, MotorStep::FULL_STEP_2_COILS); // Default full speed
}

/*******************************************************************************
*
*
* 
*******************************************************************************/
void readBasicEEPROM()
{
	SET_OPTION2(Option::POWER, eeprom_read_byte((uint8_t *)EepromIndex::E_POWER));

	// Focuser current position
	currentPos = targetPos = eeprom_read_dword((uint32_t *)EepromIndex::E_CURRENTPOS);

	SET_OPTION2(Option::BACKLASH, eeprom_read_byte((uint8_t *)EepromIndex::E_BACKLASH));
	backlashValueInward  = eeprom_read_word((uint16_t *)EepromIndex::E_BACKLASHIN);
	backlashValueOutward = eeprom_read_word((uint16_t *)EepromIndex::E_BACKLASHOUT);

	// Shift compensate
	SET_OPTION2(Option::SHIFT, eeprom_read_byte((uint8_t *)EepromIndex::E_SHIFT));

	// Read shift value
	shiftValue = eeprom_read_word((uint16_t *)EepromIndex::E_SHIFTVALUE);

	// Set shift direction
	SET_OPTION2(Option::SHIFTDIR, eeprom_read_byte((uint8_t *)EepromIndex::E_SHIFTDIR));

	// Enable shift init procedure at init
	SET_OPTION2(Option::SHIFTINIT, eeprom_read_byte((uint8_t *)EepromIndex::E_SHIFTINIT));

	// Focuser led state
	SET_OPTION2(Option::LED		, eeprom_read_byte((uint8_t *)EepromIndex::E_LED));
	SET_OPTION2(Option::LEDBLINK, eeprom_read_byte((uint8_t *)EepromIndex::E_LEDBLINK));

	focuserLedFreqBlk = eeprom_read_word((uint16_t *)EepromIndex::E_LEDFREQ);

	SET_OPTION2(Option::FULLSTEP, eeprom_read_byte((uint8_t *)EepromIndex::E_MOTORSTEP));

	motorStep = ISSET_OPTION(Option::FULLSTEP) & MotorStep::FULL_STEP_2_COILS;

	SET_OPTION2(Option::INVERSE, eeprom_read_byte((uint8_t *)EepromIndex::E_INVERSE));
}

/*******************************************************************************
*
*
* 
*******************************************************************************/
void SerialTransmitString(string_t s)
{
	while(*s)
	{
		// waiting for serial buffer available. See 19.10.1
		while(!SBA());

		UDR0 = *s++; // write char
	}
}

/*******************************************************************************
*
* Convert to hexa string   
* 
*******************************************************************************/
void sendString(const pos_t v, const Digits d)
{
	// Add more space for ending digits #\0
	// Set to 8 is less expensive than d + 2
	uint8_t response[ 8 ];

	valueToHexaString(response, v, d);

	response[d    ] = SpecialChars::HASH;
	response[d + 1] = SpecialChars::SZERO;

	SerialTransmitString(response);
}

/*******************************************************************************
*
* Convert an unsigned long integer (uint32_t) number to hexa string   
*
* Exemple : Convert ABCD hexa value to a string "ABCD"
*	iter 1 : (ABCD & F000) ==> (A000 >> 12) = 10 -> 'A'
*	iter 2 : (ABCD &  F00) ==> ( B00 >>  8) = 11 -> 'B'
*	iter 3 : (ABCD &   F0) ==> (  C0 >>  4) = 12 -> 'C'
*	iter 4 : (ABCD &    F) ==> (   D >>  0) = 13 -> 'D'
*******************************************************************************/
void valueToHexaString(uint8_t * response, const pos_t v, const Digits d)
{
	uint8_t i = (Digits::MAX_DIGIT - d);
	uint8_t qi = 4 * i;

	uint8_t bShift = 16 - qi;
	
	uint32_t bMask = 0xF0000 >> qi;

	for (uint8_t j, k = 0; i < Digits::MAX_DIGIT; i++, k++)
	{
		j = (v & bMask) >> bShift;

		response[ k ] = toHexChar(j);

		bMask >>= 4;
		bShift -= 4;
	}
}

/*******************************************************************************
*
* Initialize mirror shifting
* 
*******************************************************************************/
void initShifting()
{
	if (ISSET_OPTION(Option::SHIFTINIT) && HAS_SHIFTING())
	{
		doShifting();
	}
}

/*******************************************************************************
 *
 * do mirror shifting
 *
 *******************************************************************************/
void doShifting()
{
	SET_OPTION(Option::SHIFTREQ);

	setTargetPosition(currentPos);

	SET_OPTION(Option::RUNNING);
	UNSET_OPTION(Option::POSREACH);

	if(ISSET_OPTION(Option::LED))
	{
		FOCUSERLEDON();

		if(ISSET_OPTION(Option::LEDBLINK) && focuserLedFreqBlk > 0)
			ENABLE_TIMER0();
	}

	ENABLE_TIMER2();
}

/*******************************************************************************
*
* Compute absolute value
* 
*******************************************************************************/
pos_t abs(int32_t a)
{
	return a - ((a+a) & (a >> 31));
}

/*******************************************************************************
 *
 * see : http://www.graphics.stanford.edu/~seander/bithacks.html#IntegerAbs
 *
 *******************************************************************************/
pos_t qabs(const int32_t v)
{
	int32_t mask = (v >> 31); // v >> sizeof(v) * CHAR_BIT - 1 => (v) >> (4 * 8)-1

	return (v + mask) ^ mask;
}

/*******************************************************************************
 *
 * Compute direction
 *
 *******************************************************************************/
Direction computeDirection(const pos_t current, const pos_t target)
{
	if(current < target)
		return Direction::FOCUS_OUTWARD;
	else if(current > target)
		return Direction::FOCUS_INWARD;

	return Direction::NOT_MOVE;
}

void doBacklashTest()
{
	uint8_t svg = GET_OPTION(Option::SHIFT);
	UNSET_OPTION(Option::SHIFT);

	setTargetPosition(currentPos);

	SET_OPTION(Option::RUNNING);
	UNSET_OPTION(Option::POSREACH);

	ENABLE_TIMER2();

	SET_OPTION2(Option::SHIFT, svg);
}