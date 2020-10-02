
#ifndef SVEA_ARDUINO
#define SVEA_ARDUINO 
#define DURATION_DEBUG 0 //!< Enable/Disable reciever duration readouts over serial port 
/*! @file svea_arduino_src.h*/ 

 /*!  
 * @defgroup PwmOutputChannels Output channels on the PWM board
 */
/*@{*/
const uint8_t PWM_STEER_CHANNEL = 14; //!< Pwm pin for steering
const uint8_t PWM_SPEED_CHANNEL = 15; //!< Pwm pin for velocity
const uint8_t PWM_GEAR_CHANNEL = 13;  //!< Pwm pin for transmission
const uint8_t PWM_FDIFF_CHANNEL = 12; //!< Pwm pin for front differential lock
const uint8_t PWM_RDIFF_CHANNEL = 11; //!< Pwm pin for rear differential lock
//! Array with mapping for the PWM channels
const uint8_t PWM_CHANNELS[5] = {
                              PWM_STEER_CHANNEL,
                              PWM_SPEED_CHANNEL,
                              PWM_GEAR_CHANNEL,
                              PWM_FDIFF_CHANNEL,
                              PWM_RDIFF_CHANNEL
                              };
/*@}*/

/*!
 * @defgroup ActuationToOutput Actuation value to output (board) pwm variables
 * 
 * NOTE: The real pwm frequency from the board will differ
 * between boards, even if the sam frequency is set.
 * That is why processPwm() exists. 
 */
 /*@{*/
const int PWM_RES = 4096;  //!< Resolution of the pwm board, 12 bit
const float DUTY_CYCLE_MIN    = 1.000; //!< Minimum duty cycle of the pwm board (us)
const float DUTY_CYCLE_MAX    = 2.000; //!< Maximum duty cycle of the pwm board (us)
const int8_t INPUT_MIN      = -127;    //!< Minimum actuation value 
const int8_t INPUT_NEUTRAL  = 0;       //!< Neutral actuation value 
const int8_t INPUT_MAX      = 127;     //!< Maximum actuation value 
const float PWM_FREQUENCY = 100.0;     //!< Wanted frequency of the pwm board (Hz)
/*! 
 * @brief The minimum value that can be sent to the pwm board
 * If tune_pwm_freq() is called this value is only used temporarily.
 */
const unsigned long PWM_MIN_TICK = DUTY_CYCLE_MIN*PWM_FREQUENCY*PWM_RES;
/*! 
 * @brief The maximum 12-bit value that can be sent to the pwm board
 * If tune_pwm_freq() is called this value is only used temporarily.
 */
const unsigned long PWM_MAX_TICK = DUTY_CYCLE_MAX*PWM_FREQUENCY*PWM_RES;
//! The 12-bit value corresponding to a neutral duty cycle
unsigned int PWM_NEUTRAL_TICK = int((PWM_MIN_TICK + (PWM_MAX_TICK - PWM_MIN_TICK)*0.5)/1000.0 + 0.5);
//! The scaling factor between actuation values and the 12-bit values sent to the pwm board.
float INPUT_SCALE = ((PWM_FREQUENCY*PWM_RES*DUTY_CYCLE_MIN)/1000.0 - PWM_NEUTRAL_TICK)/(float)INPUT_MIN;
/*@}*/



/*!  
 * @defgroup PwmInputConstants PWM input constants
 * Pin number are relative pin register B (pin 8...13)
 */
/*@{*/
/*!  
 * @defgroup RecieverPwmPins Pins used connected the reciever to the arduino
 * Pin number are relative pin register B (pin 8...13)
 */
/*@{*/
const uint8_t STEER_PIN = 0; //!< D8,  Steering, connect to channel 1 on the reciever
const uint8_t SPEED_PIN = 1; //!< D9,  Velocity, connect to channel 2 on the reciever
const uint8_t GEAR_PIN = 2;  //!< D10, Transmission, connect to channel 3 on the reciever
const uint8_t FDIFF_PIN = 3; //!< D11, Front differential, connect to channel 4 on the reciever
const uint8_t RDIFF_PIN = 4; //!< D12, Rear differential, connect to channel 5 on the reciever
/*@}*/
/*
 * Input PWM Buffer constants
 */
const uint8_t PWM_BUFFER_SIZE = 8;  //!< PWM buffer size, should be a power of 2
const uint8_t PWM_IX_MASK = PWM_BUFFER_SIZE - 1; //!< Modulo mask for buffer

//! Pin mask used for falling edge detection
const uint8_t PIN_MASK = bit(STEER_PIN) 
                       | bit(SPEED_PIN)
                       | bit(GEAR_PIN)
                       | bit(FDIFF_PIN)
                       | bit(RDIFF_PIN); 
/*@}*/

/*!  
 * @defgroup EncoderPins Pins used connected the wheel encoders.
 * Have to be in the range of PORTD (pin D0 to D7) and D0 to D3 are reserved.
 */
/*@{*/
const uint8_t ENCODER_PIN_R = 6; //!< D6,  Right wheel encoder tick pin
const uint8_t ENCODER_PIN_L = 7; //!< D7,  Left wheel encoder tick pin
/*@}*/

/*
 * Software and remote state constants
 */
const unsigned long SW_TIMEOUT = 200;  //!< Duration (ms) from last recieved computer 
                                       //!< message when the computer will count as idle 


/*  
 * Message type definitions and related constants
 */ 
typedef svea_arduino::lli_ctrl lli_ctrl_in; //!< Message type for incomming messages
typedef svea_arduino::lli_ctrl lli_ctrl_out; //!< Message type for outgoing messages'
typedef svea_arduino::lli_encoder lli_encoder;

/*!  
 * @defgroup MsgBitPositions Bit positions used for the trans_diff_ctrl field in messages
 */
/*@{*/
const uint8_t GEAR_BIT = 0;   //!< Bit used for gear value (0 unlocked, 1 locked)
const uint8_t FDIFF_BIT = 1;  //!< Bit used for front differential value (0 unlocked, 1 locked)
const uint8_t RDIFF_BIT = 2; //!< Bit used for rear differential value (0 unlocked, 1 locked)
//! Vector with the bit postitions in msg.gear_diff in order: gear, front diff, rear diff
const uint8_t ACT_BITS[3] = {GEAR_BIT, FDIFF_BIT, RDIFF_BIT}; 
//! Bit indicating if the GEAR_BIT value should be read from incoming messages
const uint8_t ENABLE_GEARCHANGE_BIT = 3;
//! Bit indicating if the front differential values should be read from incoming messages
const uint8_t ENABLE_FDIFCHANGE_BIT = 4;
//! Bit indicating if the rear differential values should be read from incoming messages
const uint8_t ENABLE_RDIFCHANGE_BIT = 5;
//! Vector with the enable change bits in order: gear, front diff, rear diff
const uint8_t ENABLE_ACT_CHANGE_BITS[3] = {ENABLE_GEARCHANGE_BIT, ENABLE_FDIFCHANGE_BIT, ENABLE_RDIFCHANGE_BIT};

//! Value that should be actuated if the corresponding bit in msg.gear_diff is not set
const int8_t MSG_TO_ACT_OFF[3] = {INPUT_MAX, INPUT_MAX-10, INPUT_MIN+10};
//! Value that should be actuated if the corresponding bit in msg.gear_diff is not set
const int8_t MSG_TO_ACT_ON[3] = {INPUT_MIN, INPUT_MIN+10, INPUT_MAX-10};
/*@}*/

/*
 * Storage variables
 */
Adafruit_PWMServoDriver external_pwm = Adafruit_PWMServoDriver(); //!< pwm board instance
/*!
 * @brief Duration between rising edges of the signal from the pwm board.
 * @see pwm_to_actuation()
 */
volatile long PWM_INTERVAL_MEASURED = 0;

/*!  
 * @defgroup ActuationValueStorage Actuation value storage
 * The order is Steering, velocity, gear, front differential, rear differential
 */
/*@{*/
//! Actuation values sent from the computer
int8_t SW_ACTUATION[5] = {0,0,MSG_TO_ACT_OFF[0],MSG_TO_ACT_OFF[1],MSG_TO_ACT_OFF[2]};
//! Actuation values sent from the remote
int8_t REM_ACTUATION[5] = {0,0,MSG_TO_ACT_OFF[0],MSG_TO_ACT_OFF[1],MSG_TO_ACT_OFF[2]};
//! Actuation values sent when both the remote and the computer are idle
//  Should correspond to gear in neutral and both differentials unlocked.
const int8_t IDLE_ACTUATION[5] = {0,0,0,MSG_TO_ACT_OFF[1],MSG_TO_ACT_OFF[2]}; 
/*@}*/

/*!  
 * @defgroup PwmMeasurtement Reciever pwm duty cycle measurement variables 
 */
/*@{*/
volatile long PWM_HIGH_TIME; //!< Time of the last rising edge in micro seconds
//! True if a rising edge has been observed since the latest values was sent to ROS
volatile bool HIGH_RECIEVED = false; 
//! Indicates which pins that are still high since the last rising edge
volatile uint8_t INT_PIN_STATUS = 0; 

volatile unsigned long PWM_T_BUFFER[PWM_BUFFER_SIZE]; //!< Falling edge interrupt time buffer
volatile uint8_t PWM_S_BUFFER[PWM_BUFFER_SIZE]; //!< Falling edge interrupt pin readig buffer
volatile uint8_t PWM_BUFFER_W_IX = 0; //!< Pwm buffers read index
volatile uint8_t PWM_BUFFER_R_IX = 0; //!< Pwm buffers write index
/*@}*/


/*!
 * @defgroup EncoderVariables Wheel encoder variables
 */
/*@{*/
uint8_t RIGHT_TICK_COUNT = 0; //!< Right wheel tick count
uint8_t LEFT_TICK_COUNT = 0;  //!< left wheel tick count
/*@}*/

/*!  
 * @defgroup StatusVariables Status variables
 */
/*@{*/
unsigned long SW_T_RECIEVED=millis(); //!< Time when last message was recieved from the computer
bool REM_IDLE = true; //!< True if the remote is considered idle
bool REM_OVERRIDE = false; //!< True if the remote should override computer inputs
bool SW_IDLE = true; //!< True if the computer is considered idle
/*@}*/

/* Function definitions */
inline void set_pwm_driver(uint8_t channel, int8_t in_value);
void actuate(const int8_t actuation_values[]);
inline uint8_t set_actuated_code();
void cb_ctrl_request(const lli_ctrl_in& data);
int8_t pwm_to_actuation(unsigned long duration);
inline void pwmEvent();
void processPwm();
void tune_pwm_freq();
void processEncoderTicks();

/*!
 * @defgroup ROSSetup Variables used by ROS
 */
/*@{*/
//! NodeHandle class definitions 
ros::NodeHandle_<ArduinoHardware, 1, 3, 100, 100> nh; //<! 2 pub, 1 sub, and 100 byte buffers 
lli_ctrl_out MSG_REMOTE; //!< Message used for sending the remote signals
lli_ctrl_out MSG_ACTUATED; //!< Message sending actuated messages
lli_encoder MSG_ENCODER; //!< Message used for outgoing wheel encoder messages
ros::Publisher remote_pub("/lli/remote", &MSG_REMOTE); //!< Remote message publisher
ros::Publisher ctrl_actuated_pub("/lli/ctrl_actuated", &MSG_ACTUATED); //!< Actuated control message publisher
ros::Publisher encoder_pub("/lli/encoder", &MSG_ENCODER); //!< Encoder reading publisher
ros::Subscriber<lli_ctrl_in> ctrl_request("/lli/ctrl_request", &cb_ctrl_request ); //!< Controll request subscriber

/*@}*/
#endif /* SVEA_ARDUINO */
