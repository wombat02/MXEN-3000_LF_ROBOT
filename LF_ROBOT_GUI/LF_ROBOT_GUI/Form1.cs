using RJCP.IO.Ports;
using System.Numerics;
using System.Runtime.Serialization;


namespace LF_ROBOT_GUI
{
    public partial class Form1 : Form
    {

        // behavior constants
        const double VREF = 15.0;           // Reference voltage 
        const double V_CMP_MIN = 1.4;       // compare setpoint minimum
        const double V_CMP_MAX = 12.15;     // 
        const double VCC = 15.0;            // Magnitude of the supply voltage


        const int OP_CMP_MIN_1 = 79;
        const int OP_CMP_MAX_1 = 255;

        const int OP_CMP_MIN_2 = 36;
        const int OP_CMP_MAX_2 = 255;

        // SERIAL COMMUNICATION

        // rx / tx buffers
        byte[] Outputs = new byte[4];
        byte[] Inputs = new byte[4];

        // data constants 
        const byte START = 255;
        const byte ZERO = 0;

        private SerialPortStream serial; // nuGet package for .NET >= 5.0

        /*------------------------------------------------------------------------------------*/
        //                          FORM CONSTRUCTOR, OPEN & CLOSE
        /*------------------------------------------------------------------------------------*/



        public Form1()
        {
            InitializeComponent();

            // begin timer for regular updates 
            timer1.Start();
        }

        /// <summary>
        /// Function is called on successfull instantiation of the form object
        /// User interface initialisation
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_Load(object sender, EventArgs e)
        {
            // display a welcome message on the gui
            statusLabel.Text = DateTime.Now.ToShortDateString() + " Welcome to the LF_GUI. Make a serial connection to arduino board to get started!";
        }

        private void Form1_OnClosing(Object sender, FormClosingEventArgs e)
        {
            if (serial != null && serial.IsOpen)
            {

                // close active stream
                if (serial.IsOpen)
                {
                    serial.Close();
                }

                // give back resources to system
                serial.Dispose();
            }
        }



        /*------------------------------------------------------------------------------------*/
        //                          MAIN CONTROL LOOP
        /*------------------------------------------------------------------------------------*/


        /// <summary>
        /// Timer 1 is used to control timing of the main control loop and communications with the robot
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void timer1_Tick(object sender, EventArgs e)
        {
            if (serial != null)
            {
                if (serial.IsOpen)
                {
                    // SERIAL PORT IS OPEN
                    //statusLabel.Text = serial.PortName.ToString() + ": CONNECTED";

                    if (joystickControl.Enabled)
                    {
                        // send joystick commands to the robot for manual control
                        ManualControlLoop();
                    }
                    else
                    {
                        /// TODO AUTOMATIC CONTROL / IDLE STATE

                        msgTx (0, 0);


                        if (serial.BytesToRead >= 4)
                        {
                            byte[] rxBuf = { 0, 0, 0, 0 };
                            
                            try
                            {
                                // full packet available
                                int bytes_read = serial.Read(rxBuf, 0, rxBuf.Length);
                                //statusLabel.Text = bytes_read.ToString();
                            }
                            catch (Exception ex)
                            {
                                statusLabel.Text = ex.Message;
                            }

                            if (rxBuf[0] == START)
                            {
                                // confirm checksum
                                int checksum = START + rxBuf[1] + rxBuf[2];

                                if (checksum == rxBuf[3])
                                {
                                    // packet is valid

                                    statusLabel.Text = rxBuf[2].ToString();
                                }
                                else
                                {
                                    statusLabel.Text = "CHECKSUM ERROR: INVALID RX";
                                }

                                statusLabel.Text = rxBuf[2].ToString();
                            }
                        }
                    }
                }
                else
                {
                    // SERIAL PORT IS CLOSED
                    statusLabel.Text = "SERIAL CLOSED";
                }
            }
        }

        private void ManualControlLoop()
        {
            /// TODO
            /// L / R motor mixing with x / y joystick chanels

            // Tx joystick y chanel as target pwm
            // interpolate cmp_voltage from CMP_MIN to CMP_MAX as duty ranges from 0 to 1


            Vector2 joyPos = joystickControl.getPosNormalised();

            float ch_1 = joyPos.Y / 2.0f + 0.5f;
            float ch_2 = joyPos.Y / 2.0f + 0.5f;

            int opcode_1 = (int)(ch_1 * (OP_CMP_MAX_1 - OP_CMP_MIN_1)) + OP_CMP_MIN_1;
            int opcode_2 = (int)(ch_2 * (OP_CMP_MAX_2 - OP_CMP_MIN_2)) + OP_CMP_MIN_2;

            // convert interpolated compare voltage to an opcode
            //int opcode = (int)(255.0 * (cmp_voltage / VREF));


            // send OPCode to DAC outputs
            msgTx(2, (byte)opcode_1);
            msgTx(3, (byte)opcode_2);

            // create message text to display joystick params on the status label
            String msg = String.Format(
                "X: {0}     Y: {1}    PORT2: {2}    PORT3: {3}",
                joyPos.X.ToString("0.00"),
                joyPos.Y.ToString("0.00"),
                opcode_1,
                opcode_2
            );
            statusLabel.Text = msg;
        }

        /// <summary>
        /// Opens a SerialPortStream object to communicate with ARDUINO based on 
        /// used input text fields. 
        /// </summary>
        private void InitializeSerialPortStream()
        {
            string port = textBox_portSelect.Text.ToString();
            int baud = 9600; // default baud rate

            // parse baud rate from text input
            try 
            {
                int.Parse(textBox_baudSelect.Text);
            }
            catch (Exception e)
            {
                statusLabel.Text = "Unable to read baud rate: " + e.Message;
            }

            // check if a previously made port stream is still open or using resources before opening a new port stream
            if (serial != null)
            {
                if (serial.IsOpen)
                    serial.Close();

                serial.Dispose();
            }

            // instantiate new port stream
            serial = new SerialPortStream(port, baud, 8, Parity.None, StopBits.One);

            // subscribe RX fn to event handler
            serial.DataReceived += SerialPortStream_DataReceived;

            // attempt to open serial port
            try
            {
                serial.Open();
            }
            catch (Exception e)
            {
                statusLabel.Text = "Unable to connect to " + port + ": " + e.Message;
                button_sc2.Checked = false;
            }
        }

        private void SerialPortStream_DataReceived(Object sender, SerialDataReceivedEventArgs e)
        {
            // check and calculate checksum to determine data validity
        }

        // Send a four byte message to the Arduino via serial.

        private void msgTx(byte PORT, byte DATA)
        {
            Outputs[0] = START;                         //Set the first byte to the start value that indicates the beginning of the message.
            Outputs[1] = PORT;                          //Set the second byte to represent the port where, Input 1 = 0, Input 2 = 1, Output 1 = 2 & Output 2 = 3. This could be enumerated to make writing code simpler... (see Arduino driver)
            Outputs[2] = DATA;                          //Set the third byte to the value to be assigned to the port. This is only necessary for outputs, however it is best to assign a consistent value such as 0 for input ports.
            Outputs[3] = (byte)(START + PORT + DATA);   //Calculate the checksum byte, the same calculation is performed on the Arduino side to confirm the message was received correctly.

            try
            {
                if (serial != null)
                {
                    if (serial.IsOpen)
                    {
                        serial.Write(Outputs, 0, 4);         //Send all four bytes to the IO card.                      
                    }
                }
            }
            catch (Exception e)
            {
                // display communication error on the status bar
                statusLabel.Text = e.Message;
            }
        }

        /*------------------------------------------------------------------------------------*/
        //                          SERIAL COMMUNICATION CONTROLS
        /*------------------------------------------------------------------------------------*/

        private void button_sc1_Click(object sender, EventArgs e)
        {
            // close serial connection if exists
            if (serial != null)
            {
                serial.Close();
            }

            // ensure the open connection button is off
            button_sc2.Checked = false;
        }

        private void button_sc2_CheckedChanged(object sender, EventArgs e)
        {
            if (button_sc2.Checked)
            {
                // open serial port
                InitializeSerialPortStream();

                // send stop command
                if ( serial != null )
                {
                    if (serial.IsOpen )
                    { 
                        SendStopCommand();
                    }
                }
            }
        }

        private void button_setVoltage_Click(object sender, EventArgs e)
        {
            // parse voltage request
            double voltage = 0;

            try
            {
                voltage = double.Parse(textBox_voltage.Text);
            }
            catch (Exception ex)
            {
                statusLabel.Text = ex.Message;
            }


            float duty_cycle = (float)((voltage + VCC) / (2 * VCC));

            double cmp_voltage = duty_cycle * (V_CMP_MAX - V_CMP_MIN) + V_CMP_MIN;


            // convert output voltage reqest to its corresponding compare voltage
            // float cmp_interpollated = (float)((voltage - (-VCC)) * (V_CMP_MAX - V_CMP_MIN) / (2 * VCC) + V_CMP_MIN);

            statusLabel.Text = "CMP: " + cmp_voltage;
            int opcode = (int)(255.0 * (cmp_voltage / VREF));

            // send OPCode
            msgTx(2, (byte)opcode);
        }

        private void button_pwm_Click(object sender, EventArgs e)
        {
            double duty = 0;

            // parse pwm %
            try
            {
                duty = double.Parse(textBox_pwmDuty.Text);

            }
            catch (Exception ex)
            {
                statusLabel.Text = ex.Message;
            }

            // Ensure the duty cycle is in appropriate range before calculations
            duty = double.Clamp(duty, 0.0, 100.0);

            // interpolate cmp_voltage from CMP_MIN to CMP_MAX as duty ranges from 0 to 1
            double cmp_voltage = (duty / 100.0) * (V_CMP_MAX - V_CMP_MIN) + V_CMP_MIN;

            // convert interpolated compare voltage to an opcode
            int opcode = (int)(255.0 * (cmp_voltage / VREF));

            // send OPCode
            // TEST use PORT 2 to output on ARDUINO's DAC 1
            msgTx(2, (byte)opcode);
        }

        private void button_trackpadEnable_Click(object sender, EventArgs e)
        {
            // toggle enable flag for the joystick element
            joystickControl.Enabled = !joystickControl.Enabled;

            // update enable button text
            button_trackpadEnable.Text = joystickControl.Enabled ? "Disable" : "Enable";
        }


        /// <summary>
        /// Clicking the button will send a DAC OP code in which the code can be input between 0 and 255
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button_byteOutput_Click(object sender, EventArgs e)
        {
            // parse byte code
            int code = 0;

            try
            {
                code = int.Parse(textBox_byteOutput.Text);

                if (code < 0 || code > 255)
                {
                    code = 127; // reset code to 1/2 
                }

            }
            catch (Exception ex)
            {
                statusLabel.Text = ex.Message;
            }

            // send OPCode
            // TEST use PORT 2 to output on ARDUINO's DAC 1
            msgTx(2, (byte)code);
        }



        /// <summary>
        /// TX to both DAC PORTS the 50% duty cycle command to stop the robot.
        /// </summary>
        void SendStopCommand ()
        {
            int opcode_1 = (int)(0.5f * (OP_CMP_MAX_1 - OP_CMP_MIN_1)) + OP_CMP_MIN_1;
            int opcode_2 = (int)(0.5f * (OP_CMP_MAX_2 - OP_CMP_MIN_2)) + OP_CMP_MIN_2;

            msgTx(2, (byte) opcode_1);
            msgTx(3, (byte) opcode_2);
        }
    }
}
