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

//Declare variables for storing the port values. 
byte output1 = 255;
byte output2 = 255;
byte input1 = 0;
byte input2 = 0;

//Declare variables for each byte of the message.
byte startByte = 0;
byte commandByte = 0;
byte dataByte = 0;
byte checkByte = 0;

//Declare variable for calculating the check sum which is used to confirm that the correct bytes were identified as the four message bytes.
byte checkSum = 0;

//Declare a constant for the start byte ensuring that the value is static.
const byte START = 255;

//Declare constants to enumerate the port values.
const byte INPUT1  = 0;
const byte INPUT2  = 1;
const byte OUTPUT1 = 2;
const byte OUTPUT2 = 3;

const byte DACPIN1[8] = { 2, 3, 4, 5, 9, 8, 7, 6 };
const byte DACPIN2[8] = {A2, A3, A4, A5, A1, A0, 11, 10};
const byte SENSOR1 = A6;
const byte SENSOR2 = A7;

void outputToDAC1( byte data ) //loop through lookup table of Arduino pins connected to DAC and write correct bit to each
{
  for( int i = 0; i<=7; i++ )
  {
    digitalWrite( DACPIN1[i] , ((data>>i)&1 ? HIGH : LOW));
  }
}

void outputToDAC2( byte data ) //loop through lookup table of Arduino pins connected to DAC and write correct bit to each
{
  for( int i = 0; i<=7; i++ )
  {
    digitalWrite( DACPIN2[i] , ((data>>i)&1 ? HIGH : LOW));
  }
}

void initDACs() //loop through lookup table of Arduino pins connected to DAC and set to output mode
{
  for( int i = 0; i<=7; i++ )
  {
    pinMode( DACPIN1[i] , OUTPUT);
    pinMode( DACPIN2[i] , OUTPUT);
  }
}

//Setup initialises pins as inputs or outputs and begins the serial.
void setup() 
{
  Serial.begin(9600);
  initDACs();
}

//Main program manages setting/reading of ports via serial.
void loop() 
{

  if (Serial.available() >= 4) // Check that a full package of four bytes has arrived in the buffer.
  {
    startByte = Serial.read(); // Get the first available byte from the buffer, assuming that it is the start byte.

    if(startByte == START) // Confirm that the first byte was the start byte, otherwise begin again checking the next byte.
    {
      //Read the remaining three bytes of the package into the respective variables.
      commandByte = Serial.read();
      dataByte = Serial.read();
      checkByte = Serial.read();

      checkSum = startByte + commandByte + dataByte; // Calculate the check sum, this is also calculated in visual studio and is sent as he final byte of the package.

      if(checkByte == checkSum) //Confirm that the calculated and sent check sum match, if so it is safe to process the data.
      {
        //Check the command byte to determine which port is being called and respond accordingly.           
        switch(commandByte)
        {
          case INPUT1: //In the case of Input 1 the value is read from pin A5 and sent back in the same four byte package format.
          {
            input1 = digitalRead(SENSOR1);
                        
            Serial.write(START); //Send the start byte indicating the start of a package.
            Serial.write(commandByte); //Echo the command byte to inform Visual Studio which port value is being sent.
            Serial.write(input1); //Send the value read.
            int checkSum1 = START + commandByte + input1; //Calculate the check sum.
            Serial.write(checkSum1); //Send the check sum.
          }          
          break;
          case INPUT2: //Input 2 is the same as Input 1, but read from pin A4
          {
            input2 = digitalRead(SENSOR2);
            
            Serial.write(START); //Send the start byte indicating the start of a package.
            Serial.write(commandByte); //Echo the command byte to inform Visual Studio which port value is being sent.
            Serial.write(input2); //Send the value read.
            int checkSum2 = START + commandByte + input2; //Calculate the check sum.
            Serial.write(checkSum2); //Send the check sum.
          }               
          break;
          case OUTPUT1: //For Output 1 the value of the data byte is written to pins in DACPIN1.
          {
            output1 = dataByte;    //The value of the data byte is stored in variable output 1, this step is redundant as the value could be written directly to the port.       
            outputToDAC1(output1);       //However keeping the data in a variable could prove useful if further processing was done on the arduino side.
  
          } 
          break;
          case OUTPUT2: //For Output 1 the value of the data byte is written to pins in DACPIN2.
          {
            output2 = dataByte;  
            outputToDAC2(output2);
          }         
          break;
        }
      }
    }    
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
