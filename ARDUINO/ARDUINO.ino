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
byte output1 = 255;
byte output2 = 255;
byte input1  = 0;
byte input2  = 0;

//Declare variables for each byte of the message.
byte startByte   = 0;
byte commandByte = 0;
byte dataByte    = 0;
byte checkByte   = 0;

//Declare variable for calculating the check sum which is used to confirm that the correct bytes were identified as the four message bytes.
byte checkSum = 0;


//      SERIAL CONFIG


//Declare a constant for the start byte ensuring that the value is static.
const byte START = 255;

#define SERIAL_BAUD 9600

uint32_t next_tx_time_ms = 0;
const uint16_t tx_time_interval_ms = 25;

// Declare PORT ID enumeration
enum SERIAL_PORT_ID
{
  INPUT1,
  INPUT2,
  OUTPUT1,
  OUTPUT2
};

//      PINS

const byte DACPIN1[8] = { 2, 3, 4, 5, 9, 8, 7, 6 };
const byte DACPIN2[8] = { A2, A3, A4, A5, A1, A0, 11, 10 };
const byte SENSOR1 = A6;
const byte SENSOR2 = A7;



//      TUNING


// define constants to stop the DAC for each output
const byte DAC1_OP_STOP = 167;
const byte DAC2_OP_STOP = 145;



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
  }
  else
  {
    // serial connection is established, handle communications with GUI
    handle_serial_comms ();

    int in1 = analogRead ( SENSOR1 );
    int in2 = analogRead ( SENSOR2 );

    if ( time_ms >= next_tx_time_ms )
    {
      next_tx_time_ms = time_ms + tx_time_interval_ms;
    
      char buf [255] = { 0 };
      
      sprintf ( buf, "1: [ %d ]\t2: [ %d ]\n", in1, in2 );
      Serial.write ( buf );
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
  pinMode ( SENSOR1, INPUT );
  pinMode ( SENSOR2, INPUT );
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

/// @brief handle reading the serial stream, parsing the START byte and confirming the checksum to ensure packets are valid
void handle_serial_comms ()
{
  if (Serial.available() >= 4) // Check that a full package of four bytes has arrived in the buffer.
  {
    startByte = Serial.read (); // Get the first available byte from the buffer, assuming that it is the start byte.

    if ( startByte == START ) // Confirm that the first byte was the start byte, otherwise begin again checking the next byte.
    {
      //Read the remaining three bytes of the package into the respective variables.
      commandByte = Serial.read();
      dataByte    = Serial.read();
      checkByte   = Serial.read();

      checkSum = startByte + commandByte + dataByte; // Calculate the check sum, this is also calculated in visual studio and is sent as he final byte of the package.

      if(checkByte == checkSum) //Confirm that the calculated and sent check sum match, if so it is safe to process the data.
      {
        handle_valid_packet ();
      }
    }    
  }
}

/// @brief Parse a valid packet and perform relevent actions based on the received command byte
void handle_valid_packet ()
{
  //Check the command byte to determine which port is being called and respond accordingly.           
  switch ( commandByte )
  {
    case INPUT1: //In the case of Input 1 the value is read from pin A5 and sent back in the same four byte package format.
    {
      input1 = digitalRead ( SENSOR1 );
      transmit_input_command_response ( input1, commandByte );
    }          
    break;

    case INPUT2: //Input 2 is the same as Input 1, but read from pin A4
    {
      input2 = digitalRead ( SENSOR2 );
      transmit_input_command_response ( input2, commandByte );
    }               
    break;
    
    case OUTPUT1: //For Output 1 the value of the data byte is written to pins in DACPIN1.
    {
      output1 = dataByte;   
      output_DAC_1(output1); 
    } 
    break;
    
    case OUTPUT2: //For Output 2 the value of the data byte is written to pins in DACPIN2.
    {
      output2 = dataByte;  
      output_DAC_2(output2);
    }         
    break;

    // unrecognised commands are discarded
    default: 
      break;
  }
}

/// @brief Send a packet back to the GUI containing a sensor reading
/// @param reading ADC sensor reading to transmit ( 1 byte )
/// @param CMD Command byte to echo back ( see SERIAL_PORT_ID )
void transmit_input_command_response ( int reading, byte CMD )
{
  const int checkSum = START + commandByte + reading; //Calculate the check sum.

  Serial.write(START);       //Send the start byte indicating the start of a package.
  Serial.write(CMD);         //Echo the command byte to inform Visual Studio which port value is being sent.
  Serial.write(reading);     //Send the value read.
  Serial.write(checkSum);    //Send the check sum.
}
