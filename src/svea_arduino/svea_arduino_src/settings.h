/*! @file settings.h*/ 
//! Sampling interval for the wheel encoders in micro seconds
const unsigned long ENCODER_SAMPLE_INTERVAL = 25000;

//! Baud rate for serial transmisions
const uint32_t SERIAL_BAUD_RATE = 250000;
//! Baud rate for i2c transmisions
const uint32_t I2C_FREQUENCY = 400000;

/*! 
 * @brief Duration (us) that decides when the remote should register as idle 
 * If the registred pwm singal on channel 5 from the reciever is SHORTER than 
 * this value (in microseconds), the remote is considered to be disconnected. 
 * 
 * The reciever will still send neutral steering and velocity even if the 
 * remote is of, but channel 4 and 5 (used for differentials) turns of completely.
 * When the remote is connected again it will take approxiamtely 7 seconds until
 * the reciever starts sending on channel 5 again, and the remote can be used again.
 */
const uint8_t PWM_REM_IDLE_DETECT = 200; // micro seconds

/*
 * @defgroup PwmInputVariables Input (reciever) pwm variables 
 */
 /*@{*/
//! Time in micro seconds between the detection of a pwm signal and until the dinformation is sent to ROS 
const unsigned long INTERRUPT_DEBOUNCE_LIMIT = 2200; // micro seconds
//! Desired minimum duty cycle for the pwm board (micro seconds) 
const unsigned long PWM_LOW_LIMIT = 1006; 
//! Desired maximum duty cycle for the pwm board (micro seconds) 
const unsigned long PWM_HIGH_LIMIT = 2000;
/*@}*/

const int8_t STEERING_CLOCKWISE = 1;
const int8_t STEERING_COUNTERCLOCKWISE = -1;
//! Sets the steering direction that is sent and recieved from ROS
const int8_t STEERING_DIRECTION = STEERING_COUNTERCLOCKWISE;

/* 
 * Actuation constants
 */
//!< Minimum duration (ms) between gear changes, not used
const uint16_t GEAR_TIME_LIMIT = 500; 
const int8_t DEAD_ZONE = 2; //!< Deadzone (actuation value) for actuation signals 
