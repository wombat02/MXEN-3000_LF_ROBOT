//Curtin University
//Mechatronics Engineering

// This driver handles serial transmission of data.
// Each command or response is sent as a four byte package, formatted as follows:
// <startByte><commandByte><dataByte><checkByte>
// The start byte identifies the first byte of a new message. Ideally this value would be reserved for the start byte 
// but as that is not possible the build of the message will be confirmed using a check sum.
// The command byte allows selection of which port to interact with.
// The data byte is the value set to or read from a port, this is the main information that needs to be transmitted.
// The check sum allows for confirmation that the correct four bytes have been received as a message. This is useful as it 
// confirm that the end of the message has been reached and that no bytes were lost during the transmission.


//#################################################################################################//
//                                      MEMORY ALLOCATION
//#################################################################################################//


//      GLOBALS

/// @warning enabling this bit will disable regular robot communications!
/// This will force the robot to only transmit the sensors data in a readible form for calibration
#define DEBUG_OVERRIDE 0

//Declare variables for storing the port values. 
byte _output1 = 255;
byte _output2 = 255;
byte _sensor_l  = 0;
byte _sensor_r  = 0;

//Declare variables for each byte of the message.
byte _start_byte   = 0;
byte _id_byte = 0;
byte _data_byte    = 0;
byte _check_byte   = 0;

//Declare variable for calculating the check sum which is used to confirm that the correct bytes were identified as the four message bytes.
byte checkSum = 0;


//      SERIAL CONFIG

#define SERIAL_BAUD 115200

// Serial communication Start codes
const byte START_CMD  = 0xFF;
const byte START_DATA = 0xFE;

enum SERIAL_CMD_MSG_ID
{
  STOP,
  HEARTBEAT,
  DAC0,
  DAC1
};

enum SERIAL_DATA_MSG_ID
{
  SENSOR_L,
  SENSOR_R,
  FSM_STATE
};


// robot will enter FSM STATE DISABLE if heartbeat msg is not received within this time period
#define HEARTBEAT_RX_TIMEOUT_MS 500
uint32_t _last_heartbeat_rx_time_ms = 0;

// MSG transmission rate definitions
#define TX_RATE_FAST_MS 50
#define TX_RATE_MEDI_MS 80
#define TX_RATE_SLOW_MS 100

uint32_t _next_fast_msg_tx_time_ms = 0;
uint32_t _next_medi_msg_tx_time_ms = 0;
uint32_t _next_slow_msg_tx_time_ms = 0;

// MSG type transmission rates
#define SENSOR_DATA_MSG_TX_PERIOD_MS    TX_RATE_FAST_MS
#define MTR_CMD_DATA_MSG_TX_PERIOD_MS   TX_RATE_MEDI_MS
#define FSM_STATE_DATA_MSG_TX_PERIOD_MS TX_RATE_SLOW_MS

#define PACKET_SIZE_BYTES 4
byte tx_buf [PACKET_SIZE_BYTES] = { 0 };

//      PINS

const byte DACPIN1[8] = { 2, 3, 4, 5, 9, 8, 7, 6 };
const byte DACPIN2[8] = { A2, A3, A4, A5, A1, A0, 11, 10 };
const byte PIN_SENSOR_L = A6;
const byte PIN_SENSOR_R = A7;


//      TUNING

// define constants to stop the DAC for each output
const byte DAC1_OP_STOP = 167;
const byte DAC2_OP_STOP = 145;



// FSM 
/**
 * DISABLE -> robot is stopped and will blink LED to indicate loss of communication
 * IDLE    -> communication with GUI established, start streaming data to GUI while waiting for CMD
 * AUTO    -> AUTOMATIC CONTROL use data byte to determine auto mode (fwd, rev etc)
 * MANUAL  -> respond to joystick cmd messages to manually control the robot for tuning and validation purposes. also :)
 */
enum FSM_STATES { DISABLE, IDLE, AUTO, MANUAL };
FSM_STATES __FSM_STATE; // instance of FSM enum to keep track of the robot's current state

#define DISABLE_LED_BLINK_PERIOD_MS 300


//#################################################################################################//
//                                      FUNCTION PRE-DECLARATIONS
//#################################################################################################//

//    INIT
void init_DACs ();
void init_sensors ();

//    ROBOT CONTROLS
void stop_robot ();
void output_DAC_1 ( byte data );
void output_DAC_2 ( byte data );

void poll_sensors ();
bool check_heartbeat_valid ( const uint32_t &time_ms );

//    SERIAL COMMS
void create_msg_buf ( byte start, byte id, byte data );
void handle_serial_rx  ( const uint32_t &time_ms );
void handle_cmd_msg_rx ( const uint32_t &time_ms );

void transmit_sensor_data ();
void transmit_fsm_state ();


//    FSM 
void fsm_loop_disable ( const uint32_t &time_ms );
void fsm_loop_idle    ( const uint32_t &time_ms );
void fsm_loop_manual  ( const uint32_t &time_ms );
void fsm_loop_auto    ( const uint32_t &time_ms );

//#################################################################################################//
//                                      PROGRAM ENTRY POINT
//#################################################################################################//
void setup() 
{
  // ensure the robot is started in the disabled state
  __FSM_STATE = FSM_STATES::DISABLE;

  // setup arduino pins
  init_DACs ();
  init_sensors ();

  pinMode ( LED_BUILTIN, OUTPUT );

  // ensure the robot is stopped on power on
  // DACs will go to full throttle immediately
  stop_robot ();

  // initialise Serial stream at the default baud rate
  Serial.begin ( SERIAL_BAUD );
}

//#################################################################################################//
//                                      MAIN LOOP
//#################################################################################################//
void loop() 
{
  // get the current millisecond
  uint32_t time_ms = millis ();
  
#if DEBUG_OVERRIDE

  poll_sensors ();
  delay ( 1 );

  byte start = START_DATA;
  byte id = SENSOR_L;
  byte data = _sensor_l;
  byte cs = start + id + data;


  // TEXT BASED TX FOR READING OVER SERIAL MONITOR
  char buf [64] = { 0 };
  sprintf ( buf, "%d %d\n", _sensor_l, _sensor_r );
  Serial.write ( buf );

  delay ( 10 );

#else

  /// NORMAL ROBOT OPERATION
  if ( !Serial )
  {
    // ensure the robot stops if no serial connection is established
    // this will prevent runaway if connection to the GUI is lost
    stop_robot ();
    __FSM_STATE = DISABLE; // ensure that the state reflects the loss of COMS
  }
  else
  {
    // handle parsing received data over Serial
    handle_serial_rx ( time_ms );

    if ( !check_heartbeat_valid ( time_ms ) )
      __FSM_STATE = DISABLE; // ANY -> DISABLE disable robot under any circumstances if heartbeat is not received within the timeout
    

    // handle FSM control modes
    switch ( __FSM_STATE )
    {
      case DISABLE:
        fsm_loop_disable ( time_ms );
        break;

      case IDLE:
        fsm_loop_idle ( time_ms );
        break;

      case AUTO:
        // check heartbeat timing for loss of comms
        // send sensor data
        // send state heartbeat
        break;

      case MANUAL:
        // check heartbeat timing for loss of comms
        // send sensor data
        // send state heartbeat
        break;
    }
  }

  #endif
}

//#################################################################################################//
//                                      FUNCTION DEFINITIONS
//#################################################################################################//

/// @brief Initialise PINMODE for ALL DAC PINs
void init_DACs() 
{
  for( int i = 0; i<=7; i++ )
  {
    pinMode( DACPIN1[i] , OUTPUT);
    pinMode( DACPIN2[i] , OUTPUT);
  }
}

/// @brief Initialise PINMODE for ALL SENSOR PINs
void init_sensors ()
{
  pinMode ( PIN_SENSOR_L, INPUT );
  pinMode ( PIN_SENSOR_R, INPUT );
}

/// @brief Send the 0V command to both DACs 
void stop_robot ()
{
  // SEND THE 0V command to both DACs to ensure the motors are stopped
  output_DAC_1 ( DAC1_OP_STOP );
  output_DAC_2 ( DAC2_OP_STOP );
}

/// @brief Send an OPCODE to DAC1 (LHS motor)
/// @param data OPCODE (0 - 255)
void output_DAC_1( byte data ) //loop through lookup table of Arduino pins connected to DAC and write correct bit to each
{
  for( int i = 0; i<=7; i++ )
  {
    digitalWrite ( DACPIN1[i] , ( (data>>i)&1 ? HIGH : LOW ) );
  }
}

/// @brief Send an OPCODE to DAC2 (RHS motor)
/// @param data OPCODE (0 - 255)
void output_DAC_2( byte data ) //loop through lookup table of Arduino pins connected to DAC and write correct bit to each
{
  for( int i = 0; i<=7; i++ )
  {
    digitalWrite( DACPIN2[i] , ( (data>>i)&1 ? HIGH : LOW ) );
  }
}

/// @brief Read both left and right sensor pins and crunch 10 bit ADC range into 1 byte for efficient communication
void poll_sensors ()
{
  _sensor_l = (byte)(analogRead ( PIN_SENSOR_L ) / 4);
  _sensor_r = (byte)(analogRead ( PIN_SENSOR_R ) / 4);
}

bool check_heartbeat_valid ( const uint32_t &time_ms )
{
  return time_ms - _last_heartbeat_rx_time_ms <= HEARTBEAT_RX_TIMEOUT_MS;
}

//#################################################################################################//
//                                      SERIAL COMMUNICATION
//#################################################################################################//

void create_msg_buf ( byte start, byte id, byte data )
{
  tx_buf [0] = start;
  tx_buf [1] = id;
  tx_buf [2] = data;
  tx_buf [3] = start + id + data;  
}

void handle_serial_rx ( const uint32_t &time_ms )
{
  if (Serial.available() >= 4) // Check that a full package of four bytes has arrived in the buffer.
  {
    _start_byte = Serial.read (); // Get the first available byte from the buffer, assuming that it is the start byte.

    if ( _start_byte == START_CMD || _start_byte == START_DATA )
    {
      //Read the remaining three bytes of the package into the respective variables.
      _id_byte     = Serial.read();
      _data_byte   = Serial.read();
      _check_byte  = Serial.read();

      // Calculate the check sum, this is also calculated in visual studio and is sent as he final byte of the package.
      checkSum = _start_byte + _id_byte + _data_byte; 
    
      if ( checkSum == _check_byte ) // ensure packet is valid before continuing to process
      {
        if ( _start_byte == START_CMD )
        {
          handle_cmd_msg_rx ( time_ms );
        }
        else if ( _start_byte == START_DATA ) 
        {
          // HANDLE DATA MSG RX
        }
      }
    }
  }
}

void handle_cmd_msg_rx ( const uint32_t &time_ms )
{
  switch ( _id_byte )
  {
    case DAC0:
      _output1 = _data_byte;
      output_DAC_1 ( _output1 );
    break;

    case DAC1:
      _output2 = _data_byte;
      output_DAC_2 ( _output2 );
    break;

    case STOP:
      stop_robot ();
    break;

    case HEARTBEAT:
      _last_heartbeat_rx_time_ms = time_ms;
    break;
  }
}

void transmit_sensor_data ()
{
  /// @todo might be worth combining both sensor values into 1 byte by crunching into 4 bit range / chanel

  // TX L CH
  create_msg_buf ( (byte)START_DATA, (byte)SENSOR_L, _sensor_l );
  Serial.write ( tx_buf, PACKET_SIZE_BYTES );

  // FIX MAGIC STUFFZ
  delay ( 1 );

  // TX R CH
  create_msg_buf ( (byte)START_DATA, (byte)SENSOR_R, _sensor_r );
  Serial.write ( tx_buf, PACKET_SIZE_BYTES );
}

void transmit_fsm_state ()
{
  create_msg_buf ( (byte)START_DATA, (byte)FSM_STATE, (byte)__FSM_STATE );
  Serial.write ( tx_buf, PACKET_SIZE_BYTES );
}




//#################################################################################################//
//                                      FSM LOOP METHODS
//#################################################################################################//


void fsm_loop_disable ( const uint32_t &time_ms )
{
  static uint32_t next_led_blink_time_ms;

  // blink on-board LED
  if ( time_ms >= next_led_blink_time_ms )
  {
    next_led_blink_time_ms = time_ms + DISABLE_LED_BLINK_PERIOD_MS;
    digitalWrite ( LED_BUILTIN, !digitalRead ( LED_BUILTIN ) );
  }

  // ensure motors are stopped so robot doesn't move when not instructed
  stop_robot ();

  if ( check_heartbeat_valid ( time_ms ) )
    __FSM_STATE = IDLE; // DISABLE -> IDLE State transition on receiving valid heartbeat msg
}

void fsm_loop_idle ( const uint32_t &time_ms )
{
  // ensure onboard led is disabled to indicate robot is in idle state
  digitalWrite ( LED_BUILTIN, 0 );

  // poll sensors
  poll_sensors ();

  // MSG TRANSMISSION
  if ( time_ms >= _next_fast_msg_tx_time_ms )
  {
    transmit_sensor_data ();
    _next_fast_msg_tx_time_ms = time_ms + TX_RATE_FAST_MS;
  }

  if ( time_ms >= _next_slow_msg_tx_time_ms )
  {
    transmit_fsm_state ();
    _next_slow_msg_tx_time_ms = time_ms + TX_RATE_SLOW_MS;
  }
}

void fsm_loop_manual ( const uint32_t &time_ms )
{

}

void fsm_loop_auto ( const uint32_t &time_ms )
{

}
