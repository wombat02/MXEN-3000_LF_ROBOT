using RJCP.IO.Ports;

namespace LF_ROBOT_GUI
{
    partial class Form1
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            components = new System.ComponentModel.Container();
            joystickControl = new JoystickControl();
            timer1 = new System.Windows.Forms.Timer(components);
            serialComsLabel = new Label();
            button_sc2 = new RadioButton();
            button_sc1 = new Button();
            statusStrip1 = new StatusStrip();
            statusLabel = new ToolStripStatusLabel();
            textBox_baudSelect = new TextBox();
            label_comSelect = new Label();
            label_baudSelect = new Label();
            textBox_portSelect = new TextBox();
            button_trackpadEnable = new Button();
            textBox_pwmDuty = new TextBox();
            textBox_voltage = new TextBox();
            button_pwm = new Button();
            button_setVoltage = new Button();
            label1 = new Label();
            textBox_byteOutput = new TextBox();
            button_byteOutput = new Button();
            textBox_duty_sp = new TextBox();
            textBox_small_duty_sp_delta = new TextBox();
            label2 = new Label();
            label3 = new Label();
            label4 = new Label();
            button_auto_engage = new Button();
            label5 = new Label();
            textBox_kp = new TextBox();
            label6 = new Label();
            textBox_ki = new TextBox();
            timer2 = new System.Windows.Forms.Timer(components);
            rx_buf_indicator = new ProgressBar();
            panel_off_left = new Panel();
            panel_left_sensor = new Panel();
            panel_mid = new Panel();
            panel_off_right = new Panel();
            panel_right_sensor = new Panel();
            label7 = new Label();
            textBox_sensor_r = new TextBox();
            textBox_sensor_l = new TextBox();
            textBox_sensor_thresh = new TextBox();
            label8 = new Label();
            label9 = new Label();
            label10 = new Label();
            label11 = new Label();
            textBox_large_duty_sp_delta = new TextBox();
            panel_fsm_disconnected = new Panel();
            label12 = new Label();
            label13 = new Label();
            label14 = new Label();
            panel_fsm_idle = new Panel();
            label15 = new Label();
            panel_fsm_automatic = new Panel();
            label16 = new Label();
            panel_fsm_manual = new Panel();
            statusStrip1.SuspendLayout();
            SuspendLayout();
            // 
            // joystickControl
            // 
            joystickControl.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
            joystickControl.BackColor = Color.Gray;
            joystickControl.Enabled = false;
            joystickControl.Location = new Point(796, 194);
            joystickControl.Name = "joystickControl";
            joystickControl.Size = new Size(500, 500);
            joystickControl.TabIndex = 1;
            // 
            // timer1
            // 
            timer1.Enabled = true;
            timer1.Interval = 10;
            timer1.Tick += timer1_Tick;
            // 
            // serialComsLabel
            // 
            serialComsLabel.AutoSize = true;
            serialComsLabel.BackColor = SystemColors.Info;
            serialComsLabel.Font = new Font("Segoe UI", 12F);
            serialComsLabel.Location = new Point(47, 37);
            serialComsLabel.Name = "serialComsLabel";
            serialComsLabel.Size = new Size(132, 21);
            serialComsLabel.TabIndex = 2;
            serialComsLabel.Text = "Serial Connection";
            // 
            // button_sc2
            // 
            button_sc2.AutoSize = true;
            button_sc2.Location = new Point(125, 113);
            button_sc2.Name = "button_sc2";
            button_sc2.Size = new Size(54, 19);
            button_sc2.TabIndex = 3;
            button_sc2.Text = "Open";
            button_sc2.UseVisualStyleBackColor = true;
            button_sc2.CheckedChanged += button_sc2_CheckedChanged;
            // 
            // button_sc1
            // 
            button_sc1.Location = new Point(47, 112);
            button_sc1.Name = "button_sc1";
            button_sc1.Size = new Size(49, 22);
            button_sc1.TabIndex = 4;
            button_sc1.Text = "close";
            button_sc1.UseVisualStyleBackColor = true;
            button_sc1.Click += button_sc1_Click;
            // 
            // statusStrip1
            // 
            statusStrip1.Items.AddRange(new ToolStripItem[] { statusLabel });
            statusStrip1.Location = new Point(0, 703);
            statusStrip1.Name = "statusStrip1";
            statusStrip1.Size = new Size(1308, 22);
            statusStrip1.TabIndex = 6;
            statusStrip1.Text = "statusStrip1";
            // 
            // statusLabel
            // 
            statusLabel.Name = "statusLabel";
            statusLabel.Size = new Size(0, 17);
            // 
            // textBox_baudSelect
            // 
            textBox_baudSelect.Location = new Point(130, 84);
            textBox_baudSelect.Name = "textBox_baudSelect";
            textBox_baudSelect.Size = new Size(49, 23);
            textBox_baudSelect.TabIndex = 7;
            textBox_baudSelect.Text = "115200";
            // 
            // label_comSelect
            // 
            label_comSelect.AutoSize = true;
            label_comSelect.Font = new Font("Yet R", 12F);
            label_comSelect.Location = new Point(47, 63);
            label_comSelect.Name = "label_comSelect";
            label_comSelect.Size = new Size(47, 17);
            label_comSelect.TabIndex = 8;
            label_comSelect.Text = "PORT";
            // 
            // label_baudSelect
            // 
            label_baudSelect.AutoSize = true;
            label_baudSelect.Font = new Font("Yet R", 12F);
            label_baudSelect.Location = new Point(47, 87);
            label_baudSelect.Name = "label_baudSelect";
            label_baudSelect.Size = new Size(47, 17);
            label_baudSelect.TabIndex = 10;
            label_baudSelect.Text = "BAUD";
            // 
            // textBox_portSelect
            // 
            textBox_portSelect.Location = new Point(130, 63);
            textBox_portSelect.Name = "textBox_portSelect";
            textBox_portSelect.Size = new Size(49, 23);
            textBox_portSelect.TabIndex = 9;
            textBox_portSelect.Text = "COM16";
            // 
            // button_trackpadEnable
            // 
            button_trackpadEnable.Location = new Point(1069, 83);
            button_trackpadEnable.Name = "button_trackpadEnable";
            button_trackpadEnable.Size = new Size(120, 80);
            button_trackpadEnable.TabIndex = 12;
            button_trackpadEnable.Text = "Enable";
            button_trackpadEnable.UseVisualStyleBackColor = true;
            button_trackpadEnable.Click += button_trackpadEnable_Click;
            // 
            // textBox_pwmDuty
            // 
            textBox_pwmDuty.Location = new Point(938, 73);
            textBox_pwmDuty.Name = "textBox_pwmDuty";
            textBox_pwmDuty.Size = new Size(50, 23);
            textBox_pwmDuty.TabIndex = 13;
            textBox_pwmDuty.Text = "0.0";
            // 
            // textBox_voltage
            // 
            textBox_voltage.Location = new Point(938, 112);
            textBox_voltage.Name = "textBox_voltage";
            textBox_voltage.Size = new Size(50, 23);
            textBox_voltage.TabIndex = 14;
            textBox_voltage.Text = "0.0";
            // 
            // button_pwm
            // 
            button_pwm.Location = new Point(832, 68);
            button_pwm.Name = "button_pwm";
            button_pwm.Size = new Size(100, 30);
            button_pwm.TabIndex = 15;
            button_pwm.Text = "PWM %";
            button_pwm.UseVisualStyleBackColor = true;
            button_pwm.Click += button_pwm_Click;
            // 
            // button_setVoltage
            // 
            button_setVoltage.Location = new Point(832, 107);
            button_setVoltage.Name = "button_setVoltage";
            button_setVoltage.Size = new Size(100, 30);
            button_setVoltage.TabIndex = 16;
            button_setVoltage.Text = "Voltage";
            button_setVoltage.UseVisualStyleBackColor = true;
            button_setVoltage.Click += button_setVoltage_Click;
            // 
            // label1
            // 
            label1.AutoSize = true;
            label1.BackColor = SystemColors.Info;
            label1.Font = new Font("Segoe UI", 14F);
            label1.Location = new Point(832, 28);
            label1.Name = "label1";
            label1.Size = new Size(147, 25);
            label1.TabIndex = 17;
            label1.Text = "Output Controls";
            // 
            // textBox_byteOutput
            // 
            textBox_byteOutput.Location = new Point(938, 150);
            textBox_byteOutput.Name = "textBox_byteOutput";
            textBox_byteOutput.Size = new Size(50, 23);
            textBox_byteOutput.TabIndex = 18;
            // 
            // button_byteOutput
            // 
            button_byteOutput.Location = new Point(832, 145);
            button_byteOutput.Name = "button_byteOutput";
            button_byteOutput.Size = new Size(100, 30);
            button_byteOutput.TabIndex = 19;
            button_byteOutput.Text = "Byte";
            button_byteOutput.UseVisualStyleBackColor = true;
            button_byteOutput.Click += button_byteOutput_Click;
            // 
            // textBox_duty_sp
            // 
            textBox_duty_sp.Location = new Point(148, 602);
            textBox_duty_sp.Name = "textBox_duty_sp";
            textBox_duty_sp.Size = new Size(58, 23);
            textBox_duty_sp.TabIndex = 20;
            textBox_duty_sp.TextChanged += textBox_duty_sp_TextChanged;
            // 
            // textBox_small_duty_sp_delta
            // 
            textBox_small_duty_sp_delta.Location = new Point(148, 631);
            textBox_small_duty_sp_delta.Name = "textBox_small_duty_sp_delta";
            textBox_small_duty_sp_delta.Size = new Size(58, 23);
            textBox_small_duty_sp_delta.TabIndex = 21;
            textBox_small_duty_sp_delta.TextChanged += textBox_small_duty_sp_delta_TextChanged;
            // 
            // textBox_large_duty_sp_delta
            // 
            textBox_large_duty_sp_delta.Location = new Point(148, 664);
            textBox_large_duty_sp_delta.Name = "textBox_large_duty_sp_delta";
            textBox_large_duty_sp_delta.Size = new Size(58, 23);
            textBox_large_duty_sp_delta.TabIndex = 44;
            textBox_large_duty_sp_delta.TextChanged += textBox_large_duty_sp_delta_TextChanged;
            // 
            // label2
            // 
            label2.AutoSize = true;
            label2.BackColor = SystemColors.Info;
            label2.Font = new Font("Segoe UI", 12F);
            label2.Location = new Point(13, 565);
            label2.Name = "label2";
            label2.Size = new Size(117, 21);
            label2.TabIndex = 22;
            label2.Text = "Control Params";
            // 
            // label3
            // 
            label3.AutoSize = true;
            label3.BackColor = SystemColors.Info;
            label3.Font = new Font("Segoe UI", 12F);
            label3.Location = new Point(13, 600);
            label3.Name = "label3";
            label3.Size = new Size(129, 21);
            label3.TabIndex = 23;
            label3.Text = "TARGET DUTY SP";
            // 
            // label4
            // 
            label4.AutoSize = true;
            label4.BackColor = SystemColors.Info;
            label4.Font = new Font("Segoe UI", 12F);
            label4.Location = new Point(13, 632);
            label4.Name = "label4";
            label4.Size = new Size(95, 21);
            label4.TabIndex = 24;
            label4.Text = "SMALL SP Δ";
            // 
            // button_auto_engage
            // 
            button_auto_engage.Location = new Point(505, 604);
            button_auto_engage.Name = "button_auto_engage";
            button_auto_engage.Size = new Size(120, 80);
            button_auto_engage.TabIndex = 25;
            button_auto_engage.Text = "start auto";
            button_auto_engage.UseVisualStyleBackColor = true;
            button_auto_engage.Click += button_auto_engage_Click;
            // 
            // label5
            // 
            label5.AutoSize = true;
            label5.BackColor = SystemColors.Info;
            label5.Font = new Font("Segoe UI", 12F);
            label5.Location = new Point(217, 602);
            label5.Name = "label5";
            label5.Size = new Size(28, 21);
            label5.TabIndex = 27;
            label5.Text = "KP";
            // 
            // textBox_kp
            // 
            textBox_kp.Location = new Point(255, 602);
            textBox_kp.Name = "textBox_kp";
            textBox_kp.Size = new Size(58, 23);
            textBox_kp.TabIndex = 26;
            textBox_kp.TextChanged += textBox_kp_TextChanged;
            // 
            // label6
            // 
            label6.AutoSize = true;
            label6.BackColor = SystemColors.Info;
            label6.Font = new Font("Segoe UI", 12F);
            label6.Location = new Point(217, 634);
            label6.Name = "label6";
            label6.Size = new Size(23, 21);
            label6.TabIndex = 29;
            label6.Text = "KI";
            // 
            // textBox_ki
            // 
            textBox_ki.Location = new Point(255, 634);
            textBox_ki.Name = "textBox_ki";
            textBox_ki.Size = new Size(58, 23);
            textBox_ki.TabIndex = 28;
            textBox_ki.TextChanged += textBox_ki_TextChanged;
            // 
            // timer2
            // 
            timer2.Interval = 250;
            timer2.Tick += timer2_Tick;
            // 
            // rx_buf_indicator
            // 
            rx_buf_indicator.ForeColor = SystemColors.MenuText;
            rx_buf_indicator.Location = new Point(47, 140);
            rx_buf_indicator.Name = "rx_buf_indicator";
            rx_buf_indicator.RightToLeft = RightToLeft.No;
            rx_buf_indicator.Size = new Size(132, 23);
            rx_buf_indicator.TabIndex = 30;
            // 
            // panel_off_left
            // 
            panel_off_left.BorderStyle = BorderStyle.Fixed3D;
            panel_off_left.Location = new Point(330, 71);
            panel_off_left.Name = "panel_off_left";
            panel_off_left.Size = new Size(60, 90);
            panel_off_left.TabIndex = 31;
            // 
            // panel_left_sensor
            // 
            panel_left_sensor.BorderStyle = BorderStyle.Fixed3D;
            panel_left_sensor.Location = new Point(422, 71);
            panel_left_sensor.Name = "panel_left_sensor";
            panel_left_sensor.Size = new Size(30, 90);
            panel_left_sensor.TabIndex = 32;
            // 
            // panel_mid
            // 
            panel_mid.BorderStyle = BorderStyle.Fixed3D;
            panel_mid.Location = new Point(458, 86);
            panel_mid.Name = "panel_mid";
            panel_mid.Size = new Size(50, 60);
            panel_mid.TabIndex = 33;
            // 
            // panel_off_right
            // 
            panel_off_right.BorderStyle = BorderStyle.Fixed3D;
            panel_off_right.Location = new Point(575, 71);
            panel_off_right.Name = "panel_off_right";
            panel_off_right.Size = new Size(50, 90);
            panel_off_right.TabIndex = 35;
            // 
            // panel_right_sensor
            // 
            panel_right_sensor.BorderStyle = BorderStyle.Fixed3D;
            panel_right_sensor.Location = new Point(514, 71);
            panel_right_sensor.Name = "panel_right_sensor";
            panel_right_sensor.Size = new Size(30, 90);
            panel_right_sensor.TabIndex = 34;
            // 
            // label7
            // 
            label7.AutoSize = true;
            label7.BackColor = SystemColors.Info;
            label7.Font = new Font("Segoe UI", 12F);
            label7.Location = new Point(433, 37);
            label7.Name = "label7";
            label7.Size = new Size(98, 21);
            label7.TabIndex = 36;
            label7.Text = "Line Position";
            // 
            // textBox_sensor_r
            // 
            textBox_sensor_r.Location = new Point(514, 180);
            textBox_sensor_r.Name = "textBox_sensor_r";
            textBox_sensor_r.Size = new Size(40, 23);
            textBox_sensor_r.TabIndex = 38;
            // 
            // textBox_sensor_l
            // 
            textBox_sensor_l.Location = new Point(412, 180);
            textBox_sensor_l.Name = "textBox_sensor_l";
            textBox_sensor_l.Size = new Size(40, 23);
            textBox_sensor_l.TabIndex = 39;
            // 
            // textBox_sensor_thresh
            // 
            textBox_sensor_thresh.Location = new Point(464, 180);
            textBox_sensor_thresh.Name = "textBox_sensor_thresh";
            textBox_sensor_thresh.Size = new Size(40, 23);
            textBox_sensor_thresh.TabIndex = 40;
            textBox_sensor_thresh.TextChanged += textBox_sensor_thresh_TextChanged;
            // 
            // label8
            // 
            label8.AutoSize = true;
            label8.BackColor = SystemColors.Info;
            label8.Font = new Font("Segoe UI", 12F);
            label8.Location = new Point(412, 215);
            label8.Name = "label8";
            label8.Size = new Size(145, 21);
            label8.TabIndex = 41;
            label8.Text = "L         THRESH      R";
            // 
            // label9
            // 
            label9.AutoSize = true;
            label9.BackColor = SystemColors.Info;
            label9.Font = new Font("Segoe UI", 16F);
            label9.Location = new Point(1024, 28);
            label9.Name = "label9";
            label9.Size = new Size(206, 30);
            label9.TabIndex = 42;
            label9.Text = "MANUAL OVERRIDE";
            // 
            // label10
            // 
            label10.AutoSize = true;
            label10.BackColor = SystemColors.Info;
            label10.Font = new Font("Segoe UI", 14F);
            label10.Location = new Point(468, 561);
            label10.Name = "label10";
            label10.Size = new Size(203, 25);
            label10.TabIndex = 43;
            label10.Text = "AUTOMATIC CONTROL";
            // 
            // label11
            // 
            label11.AutoSize = true;
            label11.BackColor = SystemColors.Info;
            label11.Font = new Font("Segoe UI", 12F);
            label11.Location = new Point(13, 665);
            label11.Name = "label11";
            label11.Size = new Size(93, 21);
            label11.TabIndex = 45;
            label11.Text = "LARGE SP Δ";
            
            // 
            // panel_fsm_disconnected
            // 
            panel_fsm_disconnected.BorderStyle = BorderStyle.Fixed3D;
            panel_fsm_disconnected.Location = new Point(46, 230);
            panel_fsm_disconnected.Name = "panel_fsm_disconnected";
            panel_fsm_disconnected.Size = new Size(30, 20);
            panel_fsm_disconnected.TabIndex = 46;
            // 
            // label12
            // 
            label12.AutoSize = true;
            label12.BackColor = SystemColors.Info;
            label12.Font = new Font("Segoe UI", 12F);
            label12.Location = new Point(39, 194);
            label12.Name = "label12";
            label12.Size = new Size(105, 21);
            label12.TabIndex = 47;
            label12.Text = "ROBOT STATE";
            // 
            // label13
            // 
            label13.AutoSize = true;
            label13.Font = new Font("Yet R", 12F, FontStyle.Regular, GraphicsUnit.Point, 129);
            label13.Location = new Point(82, 230);
            label13.Name = "label13";
            label13.Size = new Size(120, 17);
            label13.TabIndex = 48;
            label13.Text = "DISCONNECTED";
            // 
            // label14
            // 
            label14.AutoSize = true;
            label14.Font = new Font("Yet R", 12F, FontStyle.Regular, GraphicsUnit.Point, 129);
            label14.Location = new Point(82, 269);
            label14.Name = "label14";
            label14.Size = new Size(39, 17);
            label14.TabIndex = 50;
            label14.Text = "IDLE";
            // 
            // panel_fsm_idle
            // 
            panel_fsm_idle.BorderStyle = BorderStyle.Fixed3D;
            panel_fsm_idle.Location = new Point(46, 269);
            panel_fsm_idle.Name = "panel_fsm_idle";
            panel_fsm_idle.Size = new Size(30, 20);
            panel_fsm_idle.TabIndex = 49;
            // 
            // label15
            // 
            label15.AutoSize = true;
            label15.Font = new Font("Yet R", 12F, FontStyle.Regular, GraphicsUnit.Point, 129);
            label15.Location = new Point(82, 314);
            label15.Name = "label15";
            label15.Size = new Size(95, 17);
            label15.TabIndex = 52;
            label15.Text = "AUTOMATIC";
            // 
            // panel_fsm_automatic
            // 
            panel_fsm_automatic.BorderStyle = BorderStyle.Fixed3D;
            panel_fsm_automatic.Location = new Point(46, 314);
            panel_fsm_automatic.Name = "panel_fsm_automatic";
            panel_fsm_automatic.Size = new Size(30, 20);
            panel_fsm_automatic.TabIndex = 51;
            // 
            // label16
            // 
            label16.AutoSize = true;
            label16.Font = new Font("Yet R", 12F, FontStyle.Regular, GraphicsUnit.Point, 129);
            label16.Location = new Point(82, 361);
            label16.Name = "label16";
            label16.Size = new Size(68, 17);
            label16.TabIndex = 54;
            label16.Text = "MANUAL";
            // 
            // panel_fsm_manual
            // 
            panel_fsm_manual.BorderStyle = BorderStyle.Fixed3D;
            panel_fsm_manual.Location = new Point(46, 361);
            panel_fsm_manual.Name = "panel_fsm_manual";
            panel_fsm_manual.Size = new Size(30, 20);
            panel_fsm_manual.TabIndex = 53;
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(7F, 15F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1308, 725);
            Controls.Add(label16);
            Controls.Add(panel_fsm_manual);
            Controls.Add(label15);
            Controls.Add(panel_fsm_automatic);
            Controls.Add(label14);
            Controls.Add(panel_fsm_idle);
            Controls.Add(label13);
            Controls.Add(label12);
            Controls.Add(panel_fsm_disconnected);
            Controls.Add(label11);
            Controls.Add(textBox_large_duty_sp_delta);
            Controls.Add(label10);
            Controls.Add(label9);
            Controls.Add(label8);
            Controls.Add(textBox_sensor_thresh);
            Controls.Add(textBox_sensor_l);
            Controls.Add(textBox_sensor_r);
            Controls.Add(label7);
            Controls.Add(panel_off_right);
            Controls.Add(panel_right_sensor);
            Controls.Add(panel_mid);
            Controls.Add(panel_left_sensor);
            Controls.Add(panel_off_left);
            Controls.Add(rx_buf_indicator);
            Controls.Add(label6);
            Controls.Add(textBox_ki);
            Controls.Add(label5);
            Controls.Add(textBox_kp);
            Controls.Add(button_auto_engage);
            Controls.Add(label4);
            Controls.Add(label3);
            Controls.Add(label2);
            Controls.Add(textBox_small_duty_sp_delta);
            Controls.Add(textBox_duty_sp);
            Controls.Add(button_byteOutput);
            Controls.Add(textBox_byteOutput);
            Controls.Add(label1);
            Controls.Add(button_setVoltage);
            Controls.Add(button_pwm);
            Controls.Add(textBox_voltage);
            Controls.Add(textBox_pwmDuty);
            Controls.Add(button_trackpadEnable);
            Controls.Add(label_baudSelect);
            Controls.Add(textBox_portSelect);
            Controls.Add(label_comSelect);
            Controls.Add(textBox_baudSelect);
            Controls.Add(statusStrip1);
            Controls.Add(button_sc1);
            Controls.Add(button_sc2);
            Controls.Add(serialComsLabel);
            Controls.Add(joystickControl);
            Name = "Form1";
            Text = "LF_GUI";
            Load += Form1_Load;
            statusStrip1.ResumeLayout(false);
            statusStrip1.PerformLayout();
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion
        private JoystickControl joystickControl;
        private System.Windows.Forms.Timer timer1;
        private Label serialComsLabel;
        private RadioButton button_sc2;
        private Button button_sc1;
        private StatusStrip statusStrip1;
        private ToolStripStatusLabel statusLabel;
        private TextBox textBox_baudSelect;
        private Label label_comSelect;
        private Label label_baudSelect;
        private TextBox textBox_portSelect;
        private Button button_trackpadEnable;
        private TextBox textBox_pwmDuty;
        private TextBox textBox_voltage;
        private Button button_pwm;
        private Button button_setVoltage;
        private Label label1;
        private TextBox textBox_byteOutput;
        private Button button_byteOutput;
        private TextBox textBox_duty_sp;
        private TextBox textBox_small_duty_sp_delta;
        private Label label2;
        private Label label3;
        private Label label4;
        private Button button_auto_engage;
        private Label label5;
        private TextBox textBox_kp;
        private Label label6;
        private TextBox textBox_ki;
        private System.Windows.Forms.Timer timer2;
        private ProgressBar rx_buf_indicator;
        private Panel panel_off_left;
        private Panel panel_left_sensor;
        private Panel panel_mid;
        private Panel panel_off_right;
        private Panel panel_right_sensor;
        private Label label7;
        private TextBox textBox_sensor_r;
        private TextBox textBox_sensor_l;
        private TextBox textBox_sensor_thresh;
        private Label label8;
        private Label label9;
        private Label label10;
        private Label label11;
        private TextBox textBox_large_duty_sp_delta;
        private Panel panel_fsm_disconnected;
        private Label label12;
        private Label label13;
        private Label label14;
        private Panel panel_fsm_idle;
        private Label label15;
        private Panel panel_fsm_automatic;
        private Label label16;
        private Panel panel_fsm_manual;
    }
}
