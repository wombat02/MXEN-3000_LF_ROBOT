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

#define SERIAL_BAUD 9600

// Serial communication Start codes
const byte START_DATA = 0xFF;
const byte START_CMD  = 0xFE;

enum SERIAL_CMD_MSG_ID
{
  DAC0,
  DAC1,
  STOP
};

enum SERIAL_DATA_MSG_ID
{
  SENSOR_L,
  SENSOR_R,
  FSM_STATE
};

uint32_t next_tx_time_ms = 0;
const uint16_t tx_time_interval_ms = 25;


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

enum FSM_STATES { DISABLE, IDLE, AUTO, MANUAL };
FSM_STATES _fsm_state; // instance of FSM enum to keep track of the robot's current state




//#################################################################################################//
//                                      FUNCTION PRE-DECLARATIONS
//#################################################################################################//

void init_DACs ();
void init_sensors ();

void stop_robot ();
void output_DAC_1 ( byte data );
void output_DAC_2 ( byte data );

byte bitFlip ( byte value );

void handle_serial_comms ();
void handle_valid_packet ();
void transmit_input_command_response ( int reading, byte CMD );


//#################################################################################################//
//                                      PROGRAM ENTRY POINT
//#################################################################################################//
void setup() 
{
  // ensure the robot is started in the disabled state
  _fsm_state = FSM_STATES::DISABLE;

  // setup arduino pins
  init_DACs ();
  init_sensors ();

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
  const uint32_t time_ms = millis ();

  // STOP CAR IF NO COMMUNICATIONS
  // NOT WORKING WHILE MAKING CONNECTIONS
  // FIND ANOTHER WAY TO ENSURE THE ROBOT IS STOPPED WHILE INITIALISING SERIAL CONNECTIONS
  if ( !Serial )
  {
    // ensure the robot stops if no serial connection is established
    // this will prevent runaway if connection to the GUI is lost
    stop_robot ();
    _fsm_state = FSM_STATES::DISABLE; // ensure that the state reflects the loss of COMS
  }
  else
  {

    // handle parsing received data over Serial
    handle_serial_rx ();


    switch ( _fsm_state )
    {
      case DISABLE:
        // blink on-board LED
        // ensure motors are stopped
      break;

      case IDLE:
        // check timing of last received heartbeat msg time to determine if comms are still operational
        // handle sending sensor data
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
}

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
    digitalWrite( DACPIN1[i] , ((data>>i)&1 ? HIGH : LOW));
  }
}

/// @brief Send an OPCODE to DAC2 (RHS motor)
/// @param data OPCODE (0 - 255)
void output_DAC_2( byte data ) //loop through lookup table of Arduino pins connected to DAC and write correct bit to each
{
  for( int i = 0; i<=7; i++ )
  {
    digitalWrite( DACPIN2[i] , ((data>>i)&1 ? HIGH : LOW));
  }
}

//Function to reverse the order of the bits.
byte bitFlip(byte value)
{
  byte bFlip = 0;
  byte j=7;
  for (byte i=0; i<8; i++) { 
    bitWrite(bFlip, i, bitRead(value, j));
    j--;
  }
  return bFlip;
}

//#################################################################################################//
//                                      SERIAL COMMUNICATION
//#################################################################################################//


void handle_serial_rx ()
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
    
      if ( checkSum == _check_byte )
      {
        if ( _start_byte == START_CMD )
        {
          // HANDLE CMD MSG
          handle_cmd_msg_rx ();
        }
        else 
        {
          // HANDLE DATA MSG

          // IS THIS ACTUALLY NEEDED HERE? 

          // WHAT DATA WOULD BE SENT TO THE NANO FROM THE GUI?
        }
      }
    }
  }
}

void handle_cmd_msg_rx ()
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
  }
}

/// @brief Send a packet back to the GUI containing a sensor reading
/// @param reading ADC sensor reading to transmit ( 1 byte )
/// @param CMD Command byte to echo back ( see SERIAL_PORT_ID )
void transmit_input_command_response ( int reading, byte CMD )
{
  const int checkSum = START + _id_byte + reading; //Calculate the check sum.

  Serial.write(START);       //Send the start byte indicating the start of a package.
  Serial.write(CMD);         //Echo the command byte to inform Visual Studio which port value is being sent.
  Serial.write(reading);     //Send the value read.
  Serial.write(checkSum);    //Send the check sum.
}


void transmit_sensor_data ()
{
  
}


void poll_sensors ()
{
  int sensor_l_raw = analogRead ( PIN_SENSOR_L );
  int sensor_r_raw = analogRead ( PIN_SENSOR_R );

  // crunch 10bit adc values into 1 byte

}
