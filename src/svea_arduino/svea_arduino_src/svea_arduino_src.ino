#include <ros.h>
#include <svea_arduino/lli_ctrl.h>
#include <svea_arduino/lli_encoder.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "settings.h"
#include "svea_arduino_src.h"
/*! @file svea_arduino_src.ino*/ 

/*
 * ACTUATION FUNCTIONS
 */

/*! 
 * @brief Set actuation PWM
 * convert a 8 bit actuation value to a pwm signal and send it to the pwm board. 
 * The value gets scaled to a duration that suits the servos (approximately 
 * 1 to 2 milli seconds). The scalling uses frequency measurements from pin
 * 10 on the pwm board. 
 * @see INPUT_SCALE
 * @see PWM_NEUTRAL_TICK
 * To avoid servo jitter at neutral a small dead zone exists around 0. 
 * @see DEAD_ZONE
 * @param channel The channel (pin) of the pwm board to send to. 
 * @param in_value Value, between -127 and 127. to send.
 */ 
inline void set_pwm_driver(uint8_t channel, int8_t in_value){
  if (in_value > 0 && in_value <= DEAD_ZONE
          ||in_value < 0 && in_value >= -DEAD_ZONE) {
      in_value = 0;
    }
  uint16_t off_tick = PWM_NEUTRAL_TICK + INPUT_SCALE*in_value;
  external_pwm.setPWM(channel, 0, off_tick);
}

/*! @brief Send settings to the pwm board through set_pwm_driver()
 * 
 * @see set_pwm_driver
 * @param actuation_values array containg 5 values. 
 */
void actuate(const int8_t actuation_values[]){
  /* Set steering and velocity */
  static int8_t previous_setting[5] = {IDLE_ACTUATION[0], 
                                       IDLE_ACTUATION[1], 
                                       IDLE_ACTUATION[2], 
                                       IDLE_ACTUATION[3], 
                                       IDLE_ACTUATION[4]};
  int8_t has_changed = 0;
  for (int i=0; i<5; i++){
    if (actuation_values[i] != previous_setting[i] && actuation_values[i] != -128) {
      set_pwm_driver(PWM_CHANNELS[i], actuation_values[i]);
      previous_setting[i] = actuation_values[i];
      has_changed++;
    }
  }
  /* 
  Send actuated values to ROS
  */
  if (has_changed > 0) {
    MSG_ACTUATED.steering = STEERING_DIRECTION*previous_setting[0];
    MSG_ACTUATED.velocity = previous_setting[1];
    MSG_ACTUATED.trans_diff = bit(ENABLE_GEARCHANGE_BIT)
                            | bit(ENABLE_FDIFCHANGE_BIT)
                            | bit(ENABLE_RDIFCHANGE_BIT);
    for (int i=0; i<3; i++){
      MSG_ACTUATED.trans_diff += previous_setting[i+2] == MSG_TO_ACT_ON[i] ? bit(i):0;
    }
    MSG_ACTUATED.ctrl = set_actuated_code();
    
    ctrl_actuated_pub.publish(&MSG_ACTUATED);
  }
}

/*!
 * @brief set the control code in messages sent to the computer
 */
inline uint8_t set_actuated_code() {
  return SW_IDLE | REM_IDLE << 1 | REM_OVERRIDE << 2;
}
// END OF ACTUATION FUNCTIONS

/*
 * SETUP ROS
 */

/*!
 * @brief Callback function for control requests from ROS
 * Interprets the message and sends the values to the pwm board
 * through actuate().
 * 
 * 
 * @param data Message to be evaluated
 */
void cb_ctrl_request(const lli_ctrl_in& data){  
  SW_ACTUATION[0] = STEERING_DIRECTION * data.steering;
  SW_ACTUATION[1] = data.velocity;
  
  // Set the on/off values
  for (int i=0; i<3; i++){
    // Only change gear/diff settings if the corresponding enable change bit is set
    if(data.trans_diff & bit(ENABLE_ACT_CHANGE_BITS[i])){ 
      SW_ACTUATION[i+2] = data.trans_diff & bit(ACT_BITS[i]) ? MSG_TO_ACT_ON[i]:MSG_TO_ACT_OFF[i]; 
    }
    else { // Otherwise use the previous value
      SW_ACTUATION[i+2] = -128; // Do
    }
  }
  
  SW_IDLE = false; 
  SW_T_RECIEVED = millis();
  if (!REM_OVERRIDE){
    actuate(SW_ACTUATION);
  }
}

// END OF ROS SETUP

/*
 * PWM RECEPTION FUNCTIONS
 */

/*!
 * @brief Handle pwm events 
 *  
 *  Reads all the values from #PWM_T_BUFFER and 
 *  #PWM_S_BUFFER that has been added since the last loop 
 *  and translates the pwm signal to actuation values.
 *  Calls pwmEvent() 3 ms after the last observed rising edge.
 *  
 *  Button value directions:
 *  Ctrl  / Value
 *  Throttle pulled -> Possitve value
 *  Steering forward -> Possitive value
 *  Gear switch up -> 0 (Low gear)
 *  Gear switch down -> 1 (High Gear)
 *  Switch back -> forward diff 0 (unlocked), back diff 0 (unlocked) 
 *  Switch middle -> forward diff 1 (locked), back diff 0 (unlocked)
 *  Switch middle -> forward diff 1 (locked), back diff 1 (locked) and REM_OVERRIDE = true
 */
void processPwm(){
  const static int pwm_middle = PWM_LOW_LIMIT + (PWM_HIGH_LIMIT-PWM_LOW_LIMIT)*0.5;
  
  if(!HIGH_RECIEVED){
    /*
    if( micros() - PWM_HIGH_TIME > 500000){
      REM_IDLE = true;
      digitalWrite(7, LOW);
    }
    */
    return; // Don't do anything if a rising edge has
            // not been detected yet.
  }
  /* Get pwm timings*/
  while (PWM_BUFFER_R_IX != PWM_BUFFER_W_IX){
    
    unsigned long duration = PWM_T_BUFFER[PWM_BUFFER_R_IX & PWM_IX_MASK];
    uint8_t change = PWM_S_BUFFER[PWM_BUFFER_R_IX & PWM_IX_MASK];
    if(bit(STEER_PIN) & change) {
      REM_ACTUATION[0] = pwm_to_actuation(duration);
      #if DURATION_DEBUG
      Serial.print("S");
      Serial.println(duration);
      #endif /* DURATION DEBUG */
    }
    if (bit(SPEED_PIN) & change) {
      REM_ACTUATION[1] = pwm_to_actuation(duration);
      #if DURATION_DEBUG
      Serial.print("V");
      Serial.println(duration);
      #endif /* DURATION DEBUG */
    }
    if (bit(GEAR_PIN) & change) {
      REM_ACTUATION[2] = duration < pwm_middle ? MSG_TO_ACT_ON[0] : MSG_TO_ACT_OFF[0];
      #if DURATION_DEBUG
      Serial.print("G");
      Serial.println(duration);
      #endif /* DURATION DEBUG */
    }
    if (bit(FDIFF_PIN) & change) {
      // Yes, the front differential should be reversed
      REM_ACTUATION[3] = duration < pwm_middle ? MSG_TO_ACT_ON[1] : MSG_TO_ACT_OFF[1]; 
      #if DURATION_DEBUG
      Serial.print("F");
      Serial.println(duration);
      #endif /* DURATION DEBUG */
    }
    if (bit(RDIFF_PIN) & change) {
      REM_ACTUATION[4] = duration > pwm_middle ? MSG_TO_ACT_ON[2] : MSG_TO_ACT_OFF[2];
      #if DURATION_DEBUG
      Serial.print("R");
      Serial.println(duration);
      #endif /* DURATION DEBUG */
      if (duration < PWM_REM_IDLE_DETECT) {
        REM_IDLE = true;
      }
      else {
        REM_IDLE = false;
      }
    }
    PWM_BUFFER_R_IX++;
  }
  /* If the duration since last rising edge is more than 3 ms:
   * Transmit recieved information to the computer
   * (or directly to thepwm board) and allow new readings.
   */
  if(micros() - PWM_HIGH_TIME > INTERRUPT_DEBOUNCE_LIMIT) {
    pwmEvent();
    HIGH_RECIEVED = false;
  }
}


/*!
 * @brief Called when a pwm signal from the remote has been recieved
 * Takes the actuation values from the remote and sends them to ROS.
 * The frequency of the ROS messages is around 100 Hz, based on the 
 * frequency of the reciever. 
 * If the computer is not sending any signals or the #REM_OVERRIDE 
 * flag is set (by pushing the deifferential switch to the formost position)
 * the actuation values will also be sent to the pwm board through actuate().
 */
inline void pwmEvent(){  
  if (REM_ACTUATION[4] != MSG_TO_ACT_ON[2] || REM_IDLE) {
    REM_OVERRIDE = false;
  }
  else {
    REM_OVERRIDE = true;
  }
  if (!REM_IDLE){
    // Use differential settings from the computer 
    // if connected and remote override is active 
    if (!SW_IDLE && REM_OVERRIDE){
      REM_ACTUATION[3] = SW_ACTUATION[3];
      REM_ACTUATION[4] = SW_ACTUATION[4];
    }
    /* 
    Send remote values to ROS
    */
    MSG_REMOTE.steering = STEERING_DIRECTION*REM_ACTUATION[0];
    MSG_REMOTE.velocity = REM_ACTUATION[1];
    MSG_REMOTE.trans_diff = bit(ENABLE_GEARCHANGE_BIT)
                            | bit(ENABLE_FDIFCHANGE_BIT)
                            | bit(ENABLE_RDIFCHANGE_BIT);
    for (int i=0; i<3; i++){
      MSG_REMOTE.trans_diff += REM_ACTUATION[i+2] == MSG_TO_ACT_ON[i] ? bit(i):0;
    }
    
    MSG_REMOTE.ctrl = set_actuated_code();
    remote_pub.publish(&MSG_REMOTE);
    
    // Take direct control if the computer is disconnected
    // or override is active.
    if (SW_IDLE || REM_OVERRIDE) {
      actuate(REM_ACTUATION);
    }
  }
}


/*!
 * Convert a pwm duration to an actuation value
 * Usese measurements of pin 10 on the pwm board.
 * @see tune_pwm_freq()
 * @param duration Duration of the high part of the pwm signal in micro seconds.
 */
int8_t pwm_to_actuation(unsigned long duration){
  const static float actuation_scaling = 255.5/(PWM_HIGH_LIMIT-PWM_LOW_LIMIT);
  if (duration < PWM_LOW_LIMIT) {
    return INPUT_MIN;
  }
  if (duration > PWM_HIGH_LIMIT) {
    return INPUT_MAX;
  }
  duration -= PWM_LOW_LIMIT;
  int8_t actuation = (duration*actuation_scaling - INPUT_MAX);
  return actuation;
}

/*!  
 * @brief Tune the frequency of the pwm board
 * Measure the frequency from the pwm board on pin 10  
 * and adjust the frequency to match #PWM_FREQUENCY
 * Will then measure the adjusted value and futher 
 * tune #PWM_NEUTRAL_TICK and #INPUT_SCALE.
 * If Pin D3 is not connected to pin 10 on the pwm board
 * weirdnes might ensue.   
 */
void tune_pwm_freq() {
  // Setup interrupt for frequency measurement
  const uint8_t adj_pwm_pin = 10;
  EICRA |= bit(ISC10) | bit(ISC11); // Set rising edge interrupts on pin D3 
  EIMSK |= bit (INT1);     // enable interrupt 1 pin D3

  // Start pwm and measure frequency
  external_pwm.setPWM(adj_pwm_pin, 0, 2000); // Start pwm signal on pin 10
  const float smoothing = 5.0; // number of measurements to use
  // Measure frequency
  float interval_sum = 0;
  for (int i = 0; i < smoothing; i++){
    delay(30);
    interval_sum += PWM_INTERVAL_MEASURED;
  }
  // Set frequency as close to the desired as possible
  if (interval_sum > 1000 && interval_sum < 1e6){ // detect pin 10 connection
    float freq_observed = (1000000.0*smoothing)/(interval_sum);
    float alpha = PWM_FREQUENCY/freq_observed;
    external_pwm.setPWMFreq(alpha*PWM_FREQUENCY);
    // Measure frequency again and adjust PWM ticks accordingly
    // needed since the frequency prescaler only has 8 bit resolution
    interval_sum = 0;
    delay(30);
    for (int i = 0; i < smoothing; i++){
      delay(30);
      interval_sum += PWM_INTERVAL_MEASURED;
    }
    freq_observed = (1000000.0*smoothing)/(interval_sum);
    unsigned long pwm_min_tick = DUTY_CYCLE_MIN*freq_observed*PWM_RES;
    unsigned long pwm_max_tick = DUTY_CYCLE_MAX*freq_observed*PWM_RES;
    PWM_NEUTRAL_TICK = int((pwm_min_tick + (pwm_max_tick - pwm_min_tick)*0.5)/1000.0 + 0.5);
    INPUT_SCALE = ((freq_observed*PWM_RES*DUTY_CYCLE_MIN)/1000.0 - PWM_NEUTRAL_TICK)/(float)INPUT_MIN;
  }
  else {;}
  EIMSK &= ~bit (INT1); // Disable interrupt on D3
}

/*!
 * @brief Count the encoder ticks since last call and send the result to ROS
 * 
 * The time_delta is given in 10 micro seconds resolution.
 */
void processEncoderTicks(){
  static unsigned long last_call_time = micros();
  static uint8_t r_last_tick_count = RIGHT_TICK_COUNT;
  static uint8_t l_last_tick_count = LEFT_TICK_COUNT;

  // Calculate and record duration
  unsigned long curr_time = micros();
  unsigned long dt = curr_time - last_call_time;
  if (dt > ENCODER_SAMPLE_INTERVAL){
    PCICR &= ~bit(PCIE2); // Temporarily disable interrupts on encoder pins
    // Calculate and record tick changes
    MSG_ENCODER.right_ticks = RIGHT_TICK_COUNT - r_last_tick_count;
    MSG_ENCODER.left_ticks = LEFT_TICK_COUNT - l_last_tick_count;
    r_last_tick_count = RIGHT_TICK_COUNT;
    l_last_tick_count = LEFT_TICK_COUNT;
    
    last_call_time = curr_time;
    MSG_ENCODER.time_delta = (uint16_t)(dt/10);  // (10 us)
    PCICR |= bit(PCIE2); // Enable interrupts on the encoder pins again
    encoder_pub.publish(&MSG_ENCODER); // Publish tick count
  }
}

/*
 * INTERRUPT HANDLING ROUTINES
 */
 
/*! 
 * @brief Interrupt on D2, for detecting rising edges in the 
 * pwm signal from the reciever. 
 * 
 * Called on rising edge of steering
 * The rising edges of all pwm signals from the reciever
 * are synced, so only one start time is necessary.
 */
ISR(INT0_vect) {
  unsigned long current_time = micros();
  if (HIGH_RECIEVED == false){
    PWM_HIGH_TIME = current_time;
    INT_PIN_STATUS = PIN_MASK;
    HIGH_RECIEVED = true;
  }
  else {;}
}

/*! 
 * @brief Interrupt on D3, measures frequency of the pwm board.
 * Used for tuning the pwm frequency.
 * D3 should be connected to pin 10 on the pwm board.
 */
ISR(INT1_vect) {
  static unsigned long adj_last_recieved = micros();
  unsigned long current_time = micros();
  PWM_INTERVAL_MEASURED = current_time - adj_last_recieved;
  adj_last_recieved = current_time;
}

/*! 
 * @brief Detects falling edges from the reciever pwm.
 * Interrupt on pins D8 ... D12 (PORTB)
 * Called on pin change on any reciever channel
 * Store time and the pins that have changed in
 * buffers on falling edges and ignores rising edges.
 */
ISR (PCINT0_vect) {
  unsigned long duration = micros() - PWM_HIGH_TIME;
  if (INT_PIN_STATUS) {
    // Get high pins that have not been low during this cycle
    uint8_t curr_read = PINB & PIN_MASK & INT_PIN_STATUS;
    uint8_t change = INT_PIN_STATUS ^ curr_read;  // Compare to previous read
    // Store time and change in buffers
    if (change != 0){
      PWM_T_BUFFER[PWM_BUFFER_W_IX & PWM_IX_MASK] = duration;
      PWM_S_BUFFER[PWM_BUFFER_W_IX & PWM_IX_MASK] = change;
      PWM_BUFFER_W_IX++;
      INT_PIN_STATUS = curr_read;
    }
  }
}

/*! 
 * @brief Detect ticks from the wheel encoders.
 * Interrupt on pins D0 ... D7 (PORTD)
 * Called on ticks from eithr encoder.
 * Detects changes in pin values of pin D6 (right wheel)
 * and D7 (left wheel). And increments the tickcounts
 * RIGHT_TICK_COUNT and LEFT_TICK_COUNT accordingly.
 */
ISR (PCINT2_vect) {
  const static uint8_t right_pin_mask = bit(6);
  const static uint8_t left_pin_mask = bit(7);
  const static uint8_t pin_mask = bit(6) | bit(7);
  static uint8_t prev_reading = 0; // Pin status from the previous change interrupt
  uint8_t reading = PIND & pin_mask; //Read current status of input pins
  uint8_t change = reading^prev_reading; // Compare to previous reading with xor
  if (change & right_pin_mask) RIGHT_TICK_COUNT++;
  if (change & left_pin_mask) LEFT_TICK_COUNT++;
  prev_reading = reading;
} 
// END OF INTERRUPTS

//! Arduino setup function
void setup() {
  /* ROS setup */
  nh.getHardware()->setBaud(SERIAL_BAUD_RATE);
  nh.initNode();
  // NOTE: Putting advertise before subscribe destroys 
  //       EVERYTHING :DDDD~~~~~
  nh.subscribe(ctrl_request);
  nh.advertise(remote_pub);
  nh.advertise(ctrl_actuated_pub);
  nh.advertise(encoder_pub);
  /* Configure pwm board */
  external_pwm.begin();
  Wire.setClock(I2C_FREQUENCY); // Must be set after external_pwm.begin() 
  external_pwm.setPWMFreq(PWM_FREQUENCY);
  tune_pwm_freq(); // Adjust PWM frequency

  /* Enable pullups on pwm reading pins to avoid noise if reciever is not connected*/
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(STEER_PIN + 8, INPUT_PULLUP);
  pinMode(SPEED_PIN + 8, INPUT_PULLUP);
  pinMode(GEAR_PIN + 8, INPUT_PULLUP);
  pinMode(FDIFF_PIN + 8, INPUT_PULLUP);
  pinMode(RDIFF_PIN + 8, INPUT_PULLUP);
  /* Same thing for the encoder pins */
  pinMode(ENCODER_PIN_R, INPUT_PULLUP);
  pinMode(ENCODER_PIN_L, INPUT_PULLUP);
  
  /* Confuigure interrupts */
  // Setings for interrupt 0, detect rising edges from the reciever on pin D2
  EICRA &= ~(bit(ISC00) | bit(ISC01) | bit(ISC10) | bit (ISC11));  // clear existing flags
  EICRA |= bit(ISC00) | bit(ISC01);  // set wanted flags (rising level interrupt)
  EIMSK |= bit (INT0);     // enable interrupt 0 pin D2
  
  // Settings for pin change interrupts for detecting falling pwm edges
  // from the reciever.
  // Enable change interrupts on pin 8, 9, 10, 11 and 12
  PCMSK0 |= bit(PCINT0) 
          | bit(PCINT1) 
          | bit(PCINT2) 
          | bit(PCINT3)
          | bit(PCINT4); 
  PCICR |= bit(PCIE0); // enable pin change interrupts on pin 8..13 (PORTB)
  
  // Settings for pin change interrupts for detecting wheel encoder ticks
  // Enable change interrupts on D6 and D7
  PCMSK2 |= bit(PCINT6) 
          | bit(PCINT7);
  PCICR |= bit(PCIE2); // enable pin change interrupts on pin 0..7 (PORTD)
}

//! Main loop
void loop() {  
  processEncoderTicks();
  processPwm();
  int idle_status = nh.spinOnce();
  if (idle_status!=ros::SPIN_OK || millis() - SW_T_RECIEVED > SW_TIMEOUT) {
    SW_IDLE = true;
  } 
  if (REM_IDLE && SW_IDLE) {
    actuate(IDLE_ACTUATION);
  }
  else {;}
}
