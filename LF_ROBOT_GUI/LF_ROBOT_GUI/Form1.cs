using RJCP.IO.Ports;
using System.Numerics;
using System.Runtime.Serialization;
using System.Timers;



namespace LF_ROBOT_GUI
{
    public partial class Form1 : Form
    {
        // enum for use in remembering the previous sensor state
        enum SensorState
        {
            NEITHER_ON_LINE,
            RIGHT_ON_LINE,
            LEFT_ON_LINE,
            BOTH_ON_LINE,
        }

        // behavior constants
        const double VREF = 15.0;           // Reference voltage 
        const double V_CMP_MIN = 1.4;       // compare setpoint minimum
        const double V_CMP_MAX = 12.15;     // 
        const double VCC = 15.0;            // Magnitude of the supply voltage


        const int OP_CMP_MIN_1 = 79;
        const int OP_CMP_MAX_1 = 255;

        const int OP_CMP_MIN_2 = 36;
        const int OP_CMP_MAX_2 = 255;

        double sensor_threshold = 150;
        double sensor_left_raw, sensor_right_raw;

        double MAX_DUTY_CYCLE = 0.65;
        double TURN_DC_CHANGE = 0.15;

        // New approximate constants based on dt = 10 milliseconds
        double K_P = 0.05;
        double K_I = 0.01;

        double error_sum_left = 0.0;
        double error_sum_right = 0.0;
        double dt = 0.0;    // Time since last update of the DC in the PI control loop
        double curr_dc_left = 0.5; // Current DC for left motor
        double curr_dc_right = 0.5; // Current DC for right motor


        double goal_duty_left = 0.0;
        double goal_duty_right = 0.0;

        // Previous and current sensor states and time in state - will only update when state changes
        SensorState prev_sensor_state = SensorState.NEITHER_ON_LINE;
        SensorState curr_sensor_state = SensorState.NEITHER_ON_LINE;
        double time_in_prev_state = 0.0;
        double time_in_curr_state = 0.0;

        // SERIAL COMMUNICATION

        // rx / tx buffers
        byte[] Outputs = new byte[4];
        byte[] Inputs = new byte[4];

        // MSG TYPES
        const byte CMD_MSG_START = 0xFF;
        const byte DATA_MSG_START = 0xFE;



        enum DATA_MSG_ID
        {
            SENSOR_L,
            SENSOR_R,
            FSM_STATE
        }

        enum CMD_MSG_ID
        {
            STOP,
            HEARTBEAT,
            DAC0,
            DAC1
        }

        enum ROBOT_FSM_STATES
        {
            DISABLE,
            IDLE,
            AUTO,
            MANUAL
        }

        ROBOT_FSM_STATES robot_fsm_state = ROBOT_FSM_STATES.DISABLE;

        bool auto_mode_en = false;


        private SerialPortStream serial; // nuGet package for .NET >= 5.0

        /*------------------------------------------------------------------------------------*/
        //                          FORM CONSTRUCTOR, OPEN & CLOSE
        /*------------------------------------------------------------------------------------*/



        public Form1()
        {
            InitializeComponent();

            // begin timer for regular updates 
            timer1.Start();
            /* dt = timer1.Interval; 
                --> REMOVED SINCE dt SHOULD BE TIME SINCE LAST UPDATE OF THE DUTY CYCLE
                --> IF THE TIMER TICKS AND THERE ARE NO NEW VALID PACKETS, DC WON'T BE UPDATED
            */
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

            textBox_duty_sp.Text = MAX_DUTY_CYCLE.ToString();
            textBox_duty_sp_delta_max.Text = TURN_DC_CHANGE.ToString();

            textBox_kp.Text = K_P.ToString ();
            textBox_ki.Text = K_I.ToString();

            prev_sensor_state = SensorState.NEITHER_ON_LINE;
            curr_sensor_state = SensorState.NEITHER_ON_LINE;
            time_in_prev_state = 0.0;
            time_in_curr_state = 0.0;
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

        /// <summary>
        /// TX to both DAC PORTS the 50% duty cycle command to stop the robot.
        /// </summary>
        void SendStopCommand()
        {
            int opcode_1 = (int)(0.5f * (OP_CMP_MAX_1 - OP_CMP_MIN_1)) + OP_CMP_MIN_1;
            int opcode_2 = (int)(0.5f * (OP_CMP_MAX_2 - OP_CMP_MIN_2)) + OP_CMP_MIN_2;

            // tuning is also done on cart
            // consider having both ways to do this and only use stop data on cart for no coms?
            TransmitCommandMessage(CMD_MSG_ID.STOP, 0);
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
            time_in_prev_state += timer1.Interval;
            time_in_curr_state += timer1.Interval;

            if (serial != null)
            {
                if (serial.IsOpen)
                {
                    /*
                    statusLabel.Text = String.Format(
                        "RX BUF: {0}  READ BUF: {1}  {2}  {3} {4}",
                        serial.BytesToRead.ToString (), Inputs[0], Inputs[1], Inputs[2], Inputs[3]
                    );
                    */

                    //          HANDLE RX   -----------------------------
                    if (serial.BytesToRead >= 4)
                    {
                        try
                        {
                            // full packet available
                            int bytes_read = serial.Read(Inputs, 0, Inputs.Length);
                        }
                        catch (Exception ex)
                        {
                            statusLabel.Text = ex.Message;
                        }

                        // confirm checksum
                        // C# byte arithmetic is promoted to int
                        // simulate overflow by masking with 0xFF
                        byte checksum = (byte)((Inputs[0] + Inputs[1] + Inputs[2]) & 0xFF);
                        if (checksum == Inputs[3])
                        {
                            if (Inputs[0] == CMD_MSG_START)
                            {
                                HandleCommandMessageRX();
                            }
                            else if (Inputs[0] == DATA_MSG_START)
                            {
                                HandleDataMessageRX();
                            }
                        }
                    }
                    //          HANDLE RX   -----------------------------



                    //statusLabel.Text = serial.PortName.ToString() + ": CONNECTED";

                    // send heartbeat message
                    TransmitCommandMessage(CMD_MSG_ID.HEARTBEAT, 0);

                    String msg = String.Format(
                        "STATE: {0}    L: {1}    R: {2}",
                        robot_fsm_state.ToString(),
                        sensor_left_raw,
                        sensor_right_raw
                    );
                    statusLabel.Text = msg;

                    if (auto_mode_en)
                    {
                        /// TODO AUTOMATIC CONTROL / IDLE STATE
                        AutomaticControlLoop();

                        dt = 0.0;
                    }
                    else
                    {
                        if (joystickControl.Enabled)
                        {
                            // send joystick commands to the robot for manual control
                            ManualControlLoop();

                            dt = 0.0; // Want to make sure dt isn't incrementing when not in auto mode

                        }
                        else
                        {
                            // tell robot to stop
                            SendStopCommand();
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

            float ch_1 = joyPos.Y / 2.0f + 0.5f;
            float ch_2 = joyPos.Y / 2.0f + 0.5f;

            int opcode_1 = (int)((1.0 - ch_1) * (OP_CMP_MAX_1 - OP_CMP_MIN_1)) + OP_CMP_MIN_1;
            int opcode_2 = (int)(ch_2 * (OP_CMP_MAX_2 - OP_CMP_MIN_2)) + OP_CMP_MIN_2;

            // convert interpolated compare voltage to an opcode
            //int opcode = (int)(255.0 * (cmp_voltage / VREF));


            // send OPCode to DAC outputs
            TransmitCommandMessage(CMD_MSG_ID.DAC0, (byte)opcode_1);
            TransmitCommandMessage(CMD_MSG_ID.DAC1, (byte)opcode_2);

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

        /*------------------------------------------------------------------------------------*/
        //                          AUTOMATIC CONTROL LOOP
        /*------------------------------------------------------------------------------------*/

        private void AutomaticControlLoop()
        {
            // Duty cycle just below the value required to overcome load voltage and begin movement
            const double robot_overshoot_stop_duty = 0.6;

            double curr_error_left;
            double curr_error_right;

            SensorState this_sensor_state = SensorState.NEITHER_ON_LINE;

            // apply thresholding to sensor values to treat readings as either 0 ON LINE OR 1 OFF LINE
            int sensor_left = (sensor_left_raw <= sensor_threshold ? 0 : 1);
            int sensor_right = (sensor_right_raw <= sensor_threshold ? 0 : 1);

            // Determine sensor state in this iteration of the control loop
            if (sensor_left == 1)
            {
                // Case when sensors on either side of line OR robot off track
                if (sensor_right == 1)
                {
                    this_sensor_state = SensorState.NEITHER_ON_LINE;
                }
                // Case when right sensor on line - turn right
                else
                {
                    this_sensor_state = SensorState.RIGHT_ON_LINE;
                }
            }
            else
            {
                // Case when left sensor on line - turn left
                if (sensor_right == 1)
                {
                    this_sensor_state = SensorState.LEFT_ON_LINE;
                }
                // Case when both sensors on the line - turn left/right depending on previous movement
                else
                {
                    this_sensor_state = SensorState.BOTH_ON_LINE;
                }
            }


            // Check if sensor state has changed since previous iteration of the control loop
            if (this_sensor_state != curr_sensor_state)
            {
                prev_sensor_state = curr_sensor_state;
                curr_sensor_state = this_sensor_state;

                time_in_prev_state = time_in_curr_state;
                time_in_curr_state = 0.0;
            }

            //switch (curr_sensor_state)
            //{
            //    // Robot either over the line (go straight) or off the line
            //    case SensorState.NEITHER_ON_LINE:
            //        goal_duty_left = MAX_DUTY_CYCLE;
            //        goal_duty_right = MAX_DUTY_CYCLE;
            //        break;

            //    // Right sensor on the line
            //    case SensorState.RIGHT_ON_LINE:
            //        // Turn right
            //        goal_duty_left = MAX_DUTY_CYCLE + TURN_DC_CHANGE;
            //        goal_duty_right = MAX_DUTY_CYCLE;
            //        break;

            //    // Left sensor on the line
            //    case SensorState.LEFT_ON_LINE:
            //        // Turn left
            //        goal_duty_left = MAX_DUTY_CYCLE;
            //        goal_duty_right = MAX_DUTY_CYCLE + TURN_DC_CHANGE;
            //        break;

            //    // Both sensors on the line - need to determine which way currently turning
            //    case SensorState.BOTH_ON_LINE:
            //        goal_duty_left = MAX_DUTY_CYCLE;
            //        goal_duty_right = MAX_DUTY_CYCLE;
            //        break;
            //}
            switch (curr_sensor_state)
            {
                // Robot either over the line (go straight) or off the line
                case SensorState.NEITHER_ON_LINE:
                    // Robot came back to over the line OR is off line but is coming back over
                    if ((curr_dc_left > curr_dc_right && prev_sensor_state == SensorState.RIGHT_ON_LINE)
                        || (curr_dc_right > curr_dc_left && prev_sensor_state == SensorState.LEFT_ON_LINE)
                        || (curr_dc_left == curr_dc_right))
                    {
                        // ACCOUNT FOR SHARP CORNERS - MIGHT NEED TO CHANGE THIS
                        // If difference in duty cycle between left and right motors is greater
                        // than 0.8 times the value added/subtract from maximum duty cycle when tuning, this
                        // means that the robot is trying to take a sharp corner, so want to keep doing this
                        if ((curr_dc_left - curr_dc_right) > 0.8*TURN_DC_CHANGE)
                        {
                            // Turn to right sharply
                            goal_duty_left = MAX_DUTY_CYCLE + TURN_DC_CHANGE;
                            goal_duty_right = MAX_DUTY_CYCLE - TURN_DC_CHANGE;
                        }
                        else if ((curr_dc_right - curr_dc_left) > 0.8*TURN_DC_CHANGE)
                        {
                            // Turn to left sharply
                            goal_duty_left = MAX_DUTY_CYCLE - TURN_DC_CHANGE;
                            goal_duty_right = MAX_DUTY_CYCLE + TURN_DC_CHANGE;
                        }
                        // Otherwise if correctly straddling line
                        else
                        {
                            // Go straight
                            goal_duty_left = MAX_DUTY_CYCLE;
                            goal_duty_right = MAX_DUTY_CYCLE;
                        }
                    }
                    // Robot went off line to the right and is turning right
                    else if (curr_dc_left > curr_dc_right && prev_sensor_state == SensorState.LEFT_ON_LINE)
                    {
                        // Stop robot
                        curr_dc_left = robot_overshoot_stop_duty;
                        curr_dc_right = robot_overshoot_stop_duty;

                        // Turn to left
                        goal_duty_left = MAX_DUTY_CYCLE;
                        goal_duty_right = MAX_DUTY_CYCLE + TURN_DC_CHANGE;
                    }
                    // Robot went off line to the left and is turning left
                    else if (curr_dc_right > curr_dc_left && prev_sensor_state == SensorState.RIGHT_ON_LINE)
                    {
                        // Stop robot
                        curr_dc_left = robot_overshoot_stop_duty;
                        curr_dc_right = robot_overshoot_stop_duty;

                        // Turn to right sharpl
                        goal_duty_left = MAX_DUTY_CYCLE + TURN_DC_CHANGE;
                        goal_duty_right = MAX_DUTY_CYCLE;
                    }
                    // Unknown state
                    else
                    {
                        // Stop robot
                        curr_dc_left = robot_overshoot_stop_duty;
                        curr_dc_right = robot_overshoot_stop_duty;

                        // Spin on the spot slowly - hopefully find line again
                        goal_duty_left = MAX_DUTY_CYCLE - TURN_DC_CHANGE;
                        goal_duty_right = (1 - MAX_DUTY_CYCLE) + TURN_DC_CHANGE;
                    }
                    break;

                // Right sensor on the line
                case SensorState.RIGHT_ON_LINE:
                    // Turn right
                    goal_duty_left = MAX_DUTY_CYCLE + TURN_DC_CHANGE;
                    goal_duty_right = MAX_DUTY_CYCLE;
                    break;

                // Left sensor on the line
                case SensorState.LEFT_ON_LINE:
                    // Turn left
                    goal_duty_left = MAX_DUTY_CYCLE;
                    goal_duty_right = MAX_DUTY_CYCLE + TURN_DC_CHANGE;
                    break;

                // Both sensors on the line - need to determine which way currently turning
                case SensorState.BOTH_ON_LINE:
                    // Robot was previously turning left
                    if (prev_sensor_state == SensorState.LEFT_ON_LINE)
                    {
                        // Turn right
                        goal_duty_left = MAX_DUTY_CYCLE + TURN_DC_CHANGE;
                        goal_duty_right = MAX_DUTY_CYCLE;
                    }
                    // Robot was previously turning right
                    else if (prev_sensor_state == SensorState.RIGHT_ON_LINE)
                    {
                        // Turn left
                        goal_duty_left = MAX_DUTY_CYCLE;
                        goal_duty_right = MAX_DUTY_CYCLE + TURN_DC_CHANGE;
                    }
                    // Unlikely state, but handled anyway
                    else
                    {
                        goal_duty_left = MAX_DUTY_CYCLE;
                        goal_duty_right = MAX_DUTY_CYCLE;
                    }
                    break;
            }

            //goal_duty_left  = double.Clamp(goal_duty_left,  0.0, 1.0);
            //goal_duty_right = double.Clamp(goal_duty_right, 0.0, 1.0);

            // Calculate error at current step
            curr_error_left = goal_duty_left - curr_dc_left;
            curr_error_right = goal_duty_right - curr_dc_right;

            // Add current error multiplied by time since last step to error sum - integral part of control
            error_sum_left += curr_error_left * dt/1000;
            error_sum_right += curr_error_right * dt/1000;

            // PI control: use error sum instead of integral error
            curr_dc_left += curr_error_left * K_P + error_sum_left * K_I;
            curr_dc_right += curr_error_right * K_P + error_sum_right * K_I;

            // Clamp the duty cycles to [0.0, 1.0] just in case
            curr_dc_left = double.Clamp(curr_dc_left, 0.0, 1.0);
            curr_dc_right = double.Clamp(curr_dc_right, 0.0, 1.0);

            // Convert desired duty cycle to binary code for DACs
            int opcode_left = (int)(curr_dc_left * (OP_CMP_MAX_1 - OP_CMP_MIN_1)) + OP_CMP_MIN_1;
            int opcode_right = (int)((1.0f - curr_dc_right) * (OP_CMP_MAX_2 - OP_CMP_MIN_2)) + OP_CMP_MIN_2;


            // SEND MOTOR COMMANDS
            TransmitCommandMessage(CMD_MSG_ID.DAC0, (byte)opcode_left);
            TransmitCommandMessage(CMD_MSG_ID.DAC1, (byte)opcode_right);

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

        private void SerialPortStream_DataReceived(Object sender, SerialDataReceivedEventArgs e)
        {
            // check and calculate checksum to determine data validity


        }

        private void HandleCommandMessageRX()
        {

        }

        private void HandleDataMessageRX()
        {
            switch (Inputs[1])
            {
                case (byte)DATA_MSG_ID.SENSOR_L:
                    sensor_left_raw = Inputs[2];
                    break;

                case (byte)DATA_MSG_ID.SENSOR_R:
                    sensor_right_raw = Inputs[2];
                    break;

                case (byte)DATA_MSG_ID.FSM_STATE:
                    robot_fsm_state = (ROBOT_FSM_STATES)Inputs[2];
                    break;
            }
        }

        private void TransmitCommandMessage(CMD_MSG_ID msgID, byte data)
        {
            Outputs[0] = CMD_MSG_START;
            Outputs[1] = (byte)msgID;
            Outputs[2] = data;

            // C# byte arithmetic is promoted to int
            // simulate overflow by masking with 0xFF
            Outputs[3] = (byte)((Outputs[0] + Outputs[1] + Outputs[2]) & 0xFF);

            // write constructed buffer
            serial.Write(Outputs, 0, Outputs.Length);
        }

        private void TransmitDataMessage(DATA_MSG_ID msgID, byte data)
        {
            Outputs[0] = DATA_MSG_START;
            Outputs[1] = (byte)msgID;
            Outputs[2] = data;

            // C# byte arithmetic is promoted to int
            // simulate overflow by masking with 0xFF
            Outputs[3] = (byte)((Outputs[0] + Outputs[1] + Outputs[2]) & 0xFF);

            // write constructed buffer
            serial.Write(Outputs, 0, Outputs.Length);
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
                        SendStopCommand();
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
                MAX_DUTY_CYCLE = double.Parse(textBox_duty_sp.Text);
            }
            catch (Exception ex)
            {
                statusLabel.Text = ex.Message;
            }
        }

        private void textBox_duty_sp_delta_max_TextChanged(object sender, EventArgs e)
        {
            // parse pwm %
            try
            {
                TURN_DC_CHANGE = double.Parse(textBox_duty_sp_delta_max.Text);
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

            if (auto_mode_en)
            {
                button_auto_engage.Text = "STOP AUTO";
            }
            else
            {
                button_auto_engage.Text = "START AUTO";

                /// RESET ALL PI CONTROL VARS
                error_sum_left = 0.0;
                error_sum_right = 0.0;
                dt = 0.0;    // Time since last update of the DC in the PI control loop
                curr_dc_left = 0.5; // Current DC for left motor
                curr_dc_right = 0.5; // Current DC for right motor


                goal_duty_left = 0.0;
                goal_duty_right = 0.0;
            }
        }

        private void textBox_kp_TextChanged(object sender, EventArgs e)
        {
            try
            {
                K_P = double.Parse(textBox_kp.Text);
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
            }
            catch (Exception ex)
            {
                statusLabel.Text = ex.Message;
            }
        }
    }
}
