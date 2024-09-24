using RJCP.IO.Ports;
using System.Numerics;
using System.Runtime.Serialization;


namespace LF_ROBOT_GUI
{
    public partial class Form1 : Form
    {

        // behavior constants
        const double VREF      = 15.0; // Reference voltage 
        const double V_CMP_MIN = 1.4;
        const double V_CMP_MAX = 12.15;
        const double VCC       = 18.0;

        // SERIAL COMMUNICATION

        // rx / tx buffers
        byte[] Outputs = new byte[4];
        byte[] Inputs  = new byte[4];

        // data constants 
        const byte START = 255;
        const byte ZERO  =   0;

        private SerialPortStream serial; // nuGet package for .NET >= 5.0

        public Form1()
        {
            InitializeComponent();
            timer1.Start();
        }

        // CALLED UPON SUCESSFULLY LOADING FORM
        private void Form1_Load(object sender, EventArgs e)
        {
            statusLabel.Text = DateTime.Now.ToString() + " Welcome to the LF_GUI. Make a serial connection to arduino board to get started!";
        }

        private void Form1_OnClosing(Object sender, FormClosingEventArgs e)
        {
            // need to close serial port before exiting program

            if (serial != null && serial.IsOpen)
            {
                serial.Close();
            }
        }

        // FORM UPDATE FN
        private void timer1_Tick(object sender, EventArgs e)
        {
            Vector2 joystickPos = joystickControl.getPosNormalised();

            if (serial != null)
            {
                if (serial.IsOpen)
                {
                    // SERIAL PORT IS OPEN
                    statusLabel.Text = serial.PortName.ToString() + ": CONNECTED";

                    if (joystickControl.Enabled)
                    {
                        // Tx joystick y chanel as target pwm
                        // interpolate cmp_voltage from CMP_MIN to CMP_MAX as duty ranges from 0 to 1
                        double duty = joystickControl.getPosNormalised().Y / 2.0 + 0.5;
                        double cmp_voltage = duty * (V_CMP_MAX - V_CMP_MIN) + V_CMP_MIN;

                        // convert interpolated compare voltage to an opcode
                        int opcode = (int)(255.0 * (cmp_voltage / VREF));

                        // send OPCode
                        msgTx(2, (byte)opcode);

                        statusLabel.Text = duty.ToString();
                    }
                }
                else
                {
                    // SERIAL PORT IS CLOSED
                    statusLabel.Text = serial.PortName.ToString() + ": NO CONNECTION";
                }
            }
        }

        private void InitializeSerialPortStream()
        {
            // ensure port is closed

            string port = textBox_portSelect.Text.ToString();
            int baud = int.Parse(textBox_baudSelect.Text);

            if (serial != null)
            {
                serial.Close();
            }

            // instantiate object
            serial = new SerialPortStream(port, baud, 8, Parity.None, StopBits.One);

            // subscribe RX fn to handler
            serial.DataReceived += SerialPortStream_DataReceived;

            // attempt to open serial port

            try
            {
                serial.Open();

                // send command for 50% duty cycle
                double cmp_voltage = 0.5 * (V_CMP_MAX - V_CMP_MIN) + V_CMP_MIN;
                int opcode = (int)(255.0 * (cmp_voltage / VREF));
                msgTx(2, (byte)opcode);
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

            if (serial != null)
            {
                if (serial.IsOpen)
                {
                    serial.Write(Outputs, 0, 4);         //Send all four bytes to the IO card.                      
                }
            }
        }


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
            //float cmp_interpollated = (float)((voltage - (-VCC)) * (V_CMP_MAX - V_CMP_MIN) / (2 * VCC) + V_CMP_MIN);

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
            double cmp_voltage = (duty/100.0) * (V_CMP_MAX - V_CMP_MIN) + V_CMP_MIN;

            // convert interpolated compare voltage to an opcode
            int opcode = (int)( 255.0 * (cmp_voltage / VREF) );

            // send OPCode
            // TEST use PORT 2 to output on ARDUINO's DAC 1
            msgTx ( 2, (byte)opcode );
        }

        private void button_trackpadEnable_Click(object sender, EventArgs e)
        {
            // toggle enable flag for the joystick element
            joystickControl.Enabled = !joystickControl.Enabled; 

            // update enable button text
            button_trackpadEnable.Text = joystickControl.Enabled ? "Disable" : "Enable";
        }

        private void button_byteOutput_Click(object sender, EventArgs e)
        {
            // parse byte code
            int code = 0;

            try
            {
                code = int.Parse(textBox_byteOutput.Text);

            }
            catch (Exception ex)
            {
                statusLabel.Text = ex.Message;
            }

            // send OPCode
            // TEST use PORT 2 to output on ARDUINO's DAC 1
            msgTx(2, (byte)code);
        }
    }
}
