using RJCP.IO.Ports;
using System.Numerics;
using System.Runtime.Serialization;
using System.Timers;



namespace LF_ROBOT_GUI
{
    public partial class Form1 : Form
    {

        /// <summary>
        /// Represents the possible configurations which can be represented with the two reflective sensors
        /// </summary>
        enum SensorStates
        {
            NEITHER_ON_LINE,
            RIGHT_ON_LINE,
            LEFT_ON_LINE,
            BOTH_ON_LINE,
        }

        /// <summary>
        /// Represents the possible states which should determine how the robot should behave
        /// </summary>
        enum RobotStates
        {
            ON_TRACK,
            OFF_LEFT,
            OFF_RIGHT
        }

        // behavior constants
        const double VREF = 15.0;           // Reference voltage 
        const double V_CMP_MIN = 1.4;       // compare setpoint minimum
        const double V_CMP_MAX = 12.15;     // 
        const double VCC = 15.0;            // Magnitude of the supply voltage

        // coefficient for the right belt to overcome tension issues
        const double RIGHT_BELT_COMP_FACTOR = 0.2;

        const int OP_CMP_MIN_1 = 79;
        const int OP_CMP_MAX_1 = 255;

        const int OP_CMP_MIN_2 = 36;
        const int OP_CMP_MAX_2 = 255;

        double sensor_threshold = 9;
        double sensor_left_raw, sensor_right_raw;

        double duty_sp = 0.75;
        double small_duty_sp_delta = 0.075;
        double large_duty_sp_delta = 0.2;

        double K_P = 0.25;
        double K_I = 0.03;

        double error_sum_left = 0.0;
        double error_sum_right = 0.0;
        double dt = 0.0;    // Time since last update of the DC in the PI control loop
        double curr_dc_left = 0.5; // Current DC for left motor
        double curr_dc_right = 0.5; // Current DC for right motor


        double goal_duty_left = 0.0;
        double goal_duty_right = 0.0;

        // Previous and current sensor states and time in state - will only update when state changes
        SensorStates prev_sensor_state = SensorStates.NEITHER_ON_LINE;
        SensorStates curr_sensor_state = SensorStates.NEITHER_ON_LINE;
        RobotStates robot_state = RobotStates.ON_TRACK;

        // SERIAL COMMUNICATION

        // used to indicate when the rx buffer is overfilling indicating the gui cannot handle packets quickly enough
        const int RX_BUFF_FULL_THRESH = 32;

        // rx / tx buffers
        byte[] tx_buf = new byte[4];
        // MSG TYPES
        byte[] rx_buf = new byte[4];

        const byte CMD_MSG_START = 0xFF;
        const byte DATA_MSG_START = 0xFE;


        /// <summary>
        /// Data messages are used by the robot and gui to communicate sensor and motor information
        /// </summary>
        enum DATA_MSG_ID
        {
            STOP,
            SENSORS,
            DAC0,
            DAC1
        }

        /// <summary>
        /// Command messages are used by the robot and gui to communicate information about state
        /// </summary>
        enum CMD_MSG_ID
        {
            HEARTBEAT,
            FSM_STATE
        }

        enum ROBOT_FSM_STATES
        {
            DISABLE,
            IDLE,
            AUTO,
            MANUAL
        }

        ROBOT_FSM_STATES robot_fsm_state = ROBOT_FSM_STATES.DISABLE;


        // REMOVE AND MAKE ENUM TO REPRESENT GUI STATE 
        // USE BUTTONS TO CHANGE GUI STATE
        bool auto_mode_en = false;


        /// <summary>
        /// SerialPortStream is provided as a NuGet package for .NET versions >= 5.0
        /// </summary>
        private SerialPortStream serial;


        //      DISPLAY PROPERTIES
        Color LINE_DISPLAY_DEFAULT = Color.DarkGray;
        Color LINE_DISPLAY_ACTIVE = Color.WhiteSmoke;
        Color FSM_DISPLAY_DEFAULT = Color.WhiteSmoke;


        /*------------------------------------------------------------------------------------*/
        //                          FORM CONSTRUCTOR, OPEN & CLOSE
        /*------------------------------------------------------------------------------------*/



        public Form1()
        {
            InitializeComponent();

            // begin timers for regular updates 
            timer1.Start();
            timer2.Start();
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

            textBox_duty_sp.Text = duty_sp.ToString();
            textBox_small_duty_sp_delta.Text = small_duty_sp_delta.ToString();
            textBox_large_duty_sp_delta.Text = large_duty_sp_delta.ToString();

            textBox_kp.Text = K_P.ToString();
            textBox_ki.Text = K_I.ToString();

            prev_sensor_state = SensorStates.BOTH_ON_LINE;
            curr_sensor_state = SensorStates.BOTH_ON_LINE;
            robot_state = RobotStates.ON_TRACK;

            //RESET LINE DISPLAY
            resetLineDisplayColours();


            textBox_sensor_thresh.Text = sensor_threshold.ToString();
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

        void TransmitStopCMD()
        {
            TransmitDataMessage(DATA_MSG_ID.STOP, 0);
        }

        private void handleLinePositionDisplay()
        {
            textBox_sensor_l.Text = sensor_left_raw.ToString();
            textBox_sensor_r.Text = sensor_right_raw.ToString();

            resetLineDisplayColours();
            switch (robot_state)
            {
                case RobotStates.ON_TRACK:

                    if (curr_sensor_state == SensorStates.BOTH_ON_LINE)
                    {
                        panel_mid.BackColor = LINE_DISPLAY_ACTIVE;
                    }
                    else if (curr_sensor_state == SensorStates.LEFT_ON_LINE)
                    {
                        panel_left_sensor.BackColor = LINE_DISPLAY_ACTIVE;
                    }
                    else
                    {
                        panel_right_sensor.BackColor = LINE_DISPLAY_ACTIVE;
                    }

                    break;

                case RobotStates.OFF_LEFT:

                    panel_off_right.BackColor = LINE_DISPLAY_ACTIVE;
                    break;

                case RobotStates.OFF_RIGHT:
                    panel_off_left.BackColor = LINE_DISPLAY_ACTIVE;
                    break;
            }
        }

        private void resetLineDisplayColours()
        {
            panel_mid.BackColor = LINE_DISPLAY_DEFAULT;
            panel_off_left.BackColor = LINE_DISPLAY_DEFAULT;
            panel_off_right.BackColor = LINE_DISPLAY_DEFAULT;
            panel_left_sensor.BackColor = LINE_DISPLAY_DEFAULT;
            panel_right_sensor.BackColor = LINE_DISPLAY_DEFAULT;
        }

        private void resetFSMPanelColours()
        {
            panel_fsm_disconnected.BackColor = FSM_DISPLAY_DEFAULT;
            panel_fsm_idle.BackColor = FSM_DISPLAY_DEFAULT;
            panel_fsm_automatic.BackColor = FSM_DISPLAY_DEFAULT;
            panel_fsm_manual.BackColor = FSM_DISPLAY_DEFAULT;
        }

        private void resetAllPIVars ()
        {
            /// RESET ALL PI CONTROL VARS
            error_sum_left = 0.0;
            error_sum_right = 0.0;
            dt = 0.0;    // Time since last update of the DC in the PI control loop
            curr_dc_left = 0.5; // Current DC for left motor
            curr_dc_right = 0.5; // Current DC for right motor


            goal_duty_left = 0.0;
            goal_duty_right = 0.0;

            prev_sensor_state = SensorStates.BOTH_ON_LINE;
            curr_sensor_state = SensorStates.BOTH_ON_LINE;
            robot_state = RobotStates.ON_TRACK;
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
            // Need to increment these every tick
            dt += timer1.Interval;  // Will get reset to zero again if joystick enabled

            if (serial != null)
            {
                if (serial.IsOpen)
                {
                    //statusLabel.Text = serial.PortName.ToString() + ": CONNECTED";

                    // visually display if rx buffer overflowing
                    int buf_indicator_val = (int)(100 * (double)serial.BytesToRead / (double)RX_BUFF_FULL_THRESH);
                    buf_indicator_val = int.Clamp(buf_indicator_val, 0, 100);
                    rx_buf_indicator.Value = buf_indicator_val;


                    // HANDLE LINE STATE DISPLAY
                    handleLinePositionDisplay();


                    // HANDLE CONTROL LOOP ARBITRATION
                    if (auto_mode_en)
                    {
                        // tell robot to enter auto state
                        TransmitCommandMessage(CMD_MSG_ID.FSM_STATE, (byte)ROBOT_FSM_STATES.AUTO);

                        /// TODO AUTOMATIC CONTROL / IDLE STATE
                        AutomaticControlLoop();

                        // reset dt
                        dt = 0.0;
                    }
                    else
                    {
                        if (joystickControl.Enabled)
                        {
                            // tell robot to enter manual state
                            TransmitCommandMessage(CMD_MSG_ID.FSM_STATE, (byte)ROBOT_FSM_STATES.MANUAL);

                            // send joystick commands to the robot for manual control
                            ManualControlLoop();

                            dt = 0.0; // Want to make sure dt isn't incrementing when not in auto mode
                        }
                        else
                        {
                            // tell robot to enter idle state
                            TransmitCommandMessage(CMD_MSG_ID.FSM_STATE, (byte)ROBOT_FSM_STATES.IDLE);

                            // tell robot to stop
                            TransmitStopCMD();
                        }
                    }
                }
            }
        }

        /// <summary>
        /// timer2 is used for sending heartbeat and handling low frequency UI updates
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void timer2_Tick(object sender, EventArgs e)
        {
            if (serial != null)
            {
                if (serial.IsOpen)
                {
                    // handle indication of robot FSM state
                    resetFSMPanelColours();
                    switch (robot_fsm_state)
                    {
                        case ROBOT_FSM_STATES.IDLE:

                            panel_fsm_idle.BackColor = Color.ForestGreen;
                            break;

                        case ROBOT_FSM_STATES.AUTO:

                            panel_fsm_automatic.BackColor = Color.ForestGreen;
                            break;

                        case ROBOT_FSM_STATES.MANUAL:

                            panel_fsm_manual.BackColor = Color.ForestGreen;
                            break;
                    }

                    // send heartbeat message
                    TransmitCommandMessage(CMD_MSG_ID.HEARTBEAT, 0);
                }
                else
                {
                    // INDICATE ROBOT IS DISCONNECTED FROM GUI
                    resetFSMPanelColours();
                    panel_fsm_disconnected.BackColor = Color.DarkRed;
                }
            }
            else
            {
                // INDICATE ROBOT IS DISCONNECTED FROM GUI
                resetFSMPanelColours();
                panel_fsm_disconnected.BackColor = Color.DarkRed;
            }
        }

        /*------------------------------------------------------------------------------------*/
        //                          MANUAL CONTROL LOOP
        /*------------------------------------------------------------------------------------*/

        private void ManualControlLoop()
        {
            /// TODO
            /// L / R motor mixing with x / y joystick chanels

            // Tx joystick y chanel as target pwm
            // interpolate cmp_voltage from CMP_MIN to CMP_MAX as duty ranges from 0 to 1


            Vector2 joyPos = joystickControl.getPosNormalised();

            /*
            float y_scaled = joyPos.Y / 2.0f + 0.5f;
            float x_scaled = joyPos.X / 2.0f + 0.5f;
            */

            float ch_1_mix = float.Clamp(joyPos.Y - joyPos.X, -1, 1);
            float ch_2_mix = float.Clamp(joyPos.Y + joyPos.X, -1, 1);

            float ch_1 = ch_1_mix / 2.0f + 0.5f;
            float ch_2 = ch_2_mix / 2.0f + 0.5f;

            int opcode_1 = (int)((1.0f - ch_1) * (OP_CMP_MAX_1 - OP_CMP_MIN_1)) + OP_CMP_MIN_1;
            int opcode_2 = (int)(ch_2 * (OP_CMP_MAX_2 - OP_CMP_MIN_2)) + OP_CMP_MIN_2;


            // send OPCode to DAC outputs
            TransmitDataMessage(DATA_MSG_ID.DAC0, (byte)opcode_1);
            TransmitDataMessage(DATA_MSG_ID.DAC1, (byte)opcode_2);

            // create message text to display joystick params on the status label
            String msg = String.Format(
                "CH1: {0}     CH2: {1}    PORT2: {2}    PORT3: {3}",
                ch_1.ToString("0.00"),
                ch_2.ToString("0.00"),
                opcode_1,
                opcode_2
            );
            statusLabel.Text = msg;
        }

        /*------------------------------------------------------------------------------------*/
        //                          AUTOMATIC CONTROL LOOP
        /*------------------------------------------------------------------------------------*/

        private void AutomaticControlLoop()
        {
            double curr_error_left;
            double curr_error_right;

            SensorStates this_sensor_state = SensorStates.NEITHER_ON_LINE;

            // apply thresholding to sensor values to treat readings as either 0 = ON LINE OR 1 = OFF LINE
            bool sensor_left  = sensor_left_raw  >= sensor_threshold;
            bool sensor_right = sensor_right_raw >= sensor_threshold;


            // Determine sensor state in this iteration of the control loop

            if (sensor_left)
            {
                // Case when sensors on either side of line OR robot off track
                if (sensor_right)
                {
                    this_sensor_state = SensorStates.NEITHER_ON_LINE;
                }
                // Case when right sensor on line
                else
                {
                    this_sensor_state = SensorStates.RIGHT_ON_LINE;
                }
            }
            else
            {
                // Case when left sensor on line
                if (sensor_right)
                {
                    this_sensor_state = SensorStates.LEFT_ON_LINE;
                }
                // Case when both sensors on the line
                else
                {
                    this_sensor_state = SensorStates.BOTH_ON_LINE;
                }
            }


            // Check if sensor state has changed since previous iteration of the control loop
            if (this_sensor_state != curr_sensor_state)
            {
                prev_sensor_state = curr_sensor_state;
                curr_sensor_state = this_sensor_state;

                // OFF TRACK DETECTION
                if (curr_sensor_state == SensorStates.NEITHER_ON_LINE)
                {
                    // Determine which way the robot left the track
                    if (prev_sensor_state == SensorStates.RIGHT_ON_LINE)
                    {
                        robot_state = RobotStates.OFF_LEFT;
                    }
                    else if (prev_sensor_state == SensorStates.LEFT_ON_LINE)
                    {
                        robot_state = RobotStates.OFF_RIGHT;
                    }
                    else if ( prev_sensor_state == SensorStates.BOTH_ON_LINE )
                    {
                        // guess which way we should go based on current duty cycles

                        if (curr_dc_left > curr_dc_right)
                        {
                            // L > R => OFF RIGHT
                            robot_state = RobotStates.OFF_RIGHT;
                        }
                        else
                        {
                            // R > L => OFF LEFT
                            robot_state = RobotStates.OFF_LEFT;
                        }
                    }
                }
                else
                {
                    // robot must be on the track if 1 or more sensors detect the track
                    robot_state = RobotStates.ON_TRACK;
                }
            }


            // arbitrate goal duty for L / R chanel based on robot & sensor state
            switch (robot_state)
            {
                case RobotStates.OFF_LEFT:

                    // sharp turn right
                    goal_duty_left  = 0.5 + large_duty_sp_delta;
                    goal_duty_right = 0.5 - (1.0 + RIGHT_BELT_COMP_FACTOR) * large_duty_sp_delta;

                    break;

                case RobotStates.OFF_RIGHT:

                    // sharp turn left
                    goal_duty_left  = 0.5 - large_duty_sp_delta;
                    goal_duty_right = 0.5 + (1.0 + RIGHT_BELT_COMP_FACTOR) * large_duty_sp_delta;

                    break;

                case RobotStates.ON_TRACK:

                    if (curr_sensor_state == SensorStates.BOTH_ON_LINE)
                    {
                        // FULL PISS AHEAD
                        goal_duty_left = goal_duty_right = duty_sp;
                    }
                    else
                    {
                        // ONE sensor is OFF the track
                        // determine which direction to turn
                        if (curr_sensor_state == SensorStates.LEFT_ON_LINE)
                        {
                            // continue turning left slowly
                            goal_duty_right = duty_sp - small_duty_sp_delta;
                            goal_duty_left  = 0.5;
                        }
                        else if (curr_sensor_state == SensorStates.RIGHT_ON_LINE)
                        {
                            // continue turning right slowly
                            goal_duty_left  = duty_sp - small_duty_sp_delta;
                            goal_duty_right = 0.5;
                        }
                    }

                    break;
            }

            // Calculate error at current step
            curr_error_left  = goal_duty_left  - curr_dc_left;
            curr_error_right = goal_duty_right - curr_dc_right;

            // Add current error multiplied by time since last step to error sum - integral part of control
            error_sum_left  += curr_error_left  * dt / 1000;
            error_sum_right += curr_error_right * dt / 1000;

            // PI control: use error sum instead of integral error
            curr_dc_left  += curr_error_left  * K_P + error_sum_left  * K_I;
            curr_dc_right += curr_error_right * K_P + error_sum_right * K_I;

            // Clamp the duty cycles to [0.0, 1.0] just in case
            curr_dc_left  = double.Clamp(curr_dc_left,  0.0, 1.0);
            curr_dc_right = double.Clamp(curr_dc_right, 0.0, 1.0);

            // Convert desired duty cycle to binary code for DACs
            int opcode_left = (int)((curr_dc_left) * (OP_CMP_MAX_1 - OP_CMP_MIN_1)) + OP_CMP_MIN_1;
            int opcode_right = (int)((1.0f - curr_dc_right) * (OP_CMP_MAX_2 - OP_CMP_MIN_2)) + OP_CMP_MIN_2;


            // SEND MOTOR COMMANDS
            TransmitDataMessage(DATA_MSG_ID.DAC0, (byte)opcode_left);
            TransmitDataMessage(DATA_MSG_ID.DAC1, (byte)opcode_right);

            // create message text to display joystick params on the status label
            String msg = String.Format(
                "PORT2: {0}    PORT3: {1}    L: {2}    R: {3}    SS: C: {4} P: {5}  dt: {6}  goal_dc_left {7}  goal_dc_right {8}",
                opcode_left,
                opcode_right,
                sensor_left,
                sensor_right,
                curr_sensor_state,
                prev_sensor_state,
                dt.ToString(),
                goal_duty_left.ToString(),
                goal_duty_right.ToString()
            );
            statusLabel.Text = msg;
        }


        /*------------------------------------------------------------------------------------*/
        //                          SERIAL COMMUNICATION
        /*------------------------------------------------------------------------------------*/

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
                baud = int.Parse(textBox_baudSelect.Text);
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

        /// <summary>
        /// This function is run on a seperate thread to the GUI
        /// Take advantage of this and use a while loop to ensure all RX data is processed ensuring data
        /// doesn't end up accumulating in the serial buffer
        /// </summary>
        /// <param name="s"></param>
        /// <param name="e"></param>
        private void SerialPortStream_DataReceived(object? s, SerialDataReceivedEventArgs e)
        {
            while (serial.BytesToRead >= 4)
            {
                // read first byte and check if it matches a start ID
                serial.Read(rx_buf, 0, 1);

                if (rx_buf[0] == CMD_MSG_START || rx_buf[0] == DATA_MSG_START)
                {
                    // read remaining bytes into rx_buf
                    serial.Read(rx_buf, 1, rx_buf.Length - 1);

                    // confirm checksum
                    // C# byte arithmetic is promoted to int
                    // simulate overflow by masking with 0xFF
                    byte checksum = (byte)((rx_buf[0] + rx_buf[1] + rx_buf[2]) & 0xFF);
                    if (checksum == rx_buf[3])
                    {
                        if (rx_buf[0] == CMD_MSG_START)
                        {
                            HandleCommandMessageRX();
                        }
                        else if (rx_buf[0] == DATA_MSG_START)
                        {
                            HandleDataMessageRX();
                        }
                    }
                }
            }
        }

        private void HandleCommandMessageRX()
        {
            CMD_MSG_ID msg_id = (CMD_MSG_ID)rx_buf[1];

            switch (msg_id)
            {
                case CMD_MSG_ID.HEARTBEAT:

                    // TODO USE HEARTBEAT FROM ROBOT TO ALLOW GUI TO RECOGNISE ARDUINO HALT
                    break;

                case CMD_MSG_ID.FSM_STATE:

                    // upate cached robot fsm state
                    robot_fsm_state = (ROBOT_FSM_STATES)rx_buf[2];
                    break;
            }
        }

        private void HandleDataMessageRX()
        {
            DATA_MSG_ID msg_id = (DATA_MSG_ID)rx_buf[1];

            switch (msg_id)
            {
                case DATA_MSG_ID.SENSORS:

                    // parse sensor values
                    byte data = rx_buf[2];

                    // mask out relevant bits for each sensor value
                    sensor_left_raw = (data >> 4) & 0x0F;
                    sensor_right_raw = data & 0x0F;

                    break;

                default: break; // do nothing with other robot data msgs
            }
        }

        private void TransmitCommandMessage(CMD_MSG_ID msgID, byte data)
        {
            tx_buf[0] = CMD_MSG_START;
            tx_buf[1] = (byte)msgID;
            tx_buf[2] = data;

            // C# byte arithmetic is promoted to int
            // simulate overflow by masking with 0xFF
            tx_buf[3] = (byte)((tx_buf[0] + tx_buf[1] + tx_buf[2]) & 0xFF);

            // write constructed buffer
            serial.Write(tx_buf, 0, tx_buf.Length);
        }

        private void TransmitDataMessage(DATA_MSG_ID msgID, byte data)
        {
            tx_buf[0] = DATA_MSG_START;
            tx_buf[1] = (byte)msgID;
            tx_buf[2] = data;

            // C# byte arithmetic is promoted to int
            // simulate overflow by masking with 0xFF
            tx_buf[3] = (byte)((tx_buf[0] + tx_buf[1] + tx_buf[2]) & 0xFF);

            // write constructed buffer
            serial.Write(tx_buf, 0, tx_buf.Length);
        }

        /*------------------------------------------------------------------------------------*/
        //                          SERIAL COMMUNICATION GUI CONTROLS
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
                if (serial != null)
                {
                    if (serial.IsOpen)
                    {
                        TransmitStopCMD();
                    }
                }
            }
        }

        /*------------------------------------------------------------------------------------*/
        //                          OUTPUT OVERRIDE BUTTONS
        /*------------------------------------------------------------------------------------*/

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

            // send OPCodes
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

            // send OPCodes
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

            // send OPCodes
        }


        private void textBox_duty_sp_TextChanged(object sender, EventArgs e)
        {
            // parse pwm %
            try
            {
                duty_sp = double.Parse(textBox_duty_sp.Text);
                resetAllPIVars();
            }
            catch (Exception ex)
            {
                statusLabel.Text = ex.Message;
            }
        }

        private void textBox_small_duty_sp_delta_TextChanged(object sender, EventArgs e)
        {
            // parse pwm %
            try
            {
                small_duty_sp_delta = double.Parse(textBox_small_duty_sp_delta.Text);
                resetAllPIVars();
            }
            catch (Exception ex)
            {
                statusLabel.Text = ex.Message;
            }
        }

        private void textBox_large_duty_sp_delta_TextChanged(object sender, EventArgs e)
        {
            // parse pwm %
            try
            {
                large_duty_sp_delta = double.Parse(textBox_large_duty_sp_delta.Text);
                resetAllPIVars();
            }
            catch (Exception ex)
            {
                statusLabel.Text = ex.Message;
            }
        }

        private void button_auto_engage_Click(object sender, EventArgs e)
        {
            // toggle state
            auto_mode_en = !auto_mode_en;
            resetAllPIVars();

            if (auto_mode_en)
            {
                button_auto_engage.Text = "STOP AUTO";
            }
            else
            {
                button_auto_engage.Text = "START AUTO";
            }
        }

        private void textBox_kp_TextChanged(object sender, EventArgs e)
        {
            try
            {
                K_P = double.Parse(textBox_kp.Text);
                resetAllPIVars();
            }
            catch (Exception ex)
            {
                statusLabel.Text = ex.Message;
            }
        }

        private void textBox_ki_TextChanged(object sender, EventArgs e)
        {
            try
            {
                K_I = double.Parse(textBox_ki.Text);
                resetAllPIVars();
            }
            catch (Exception ex)
            {
                statusLabel.Text = ex.Message;
            }
        }

        private void textBox_sensor_thresh_TextChanged(object sender, EventArgs e)
        {
            try
            {
                sensor_threshold = int.Parse(textBox_sensor_thresh.Text);
            }
            catch (Exception ex)
            {
                statusLabel.Text = ex.Message;
            }
        }
    }
}  