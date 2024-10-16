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
            textBox_duty_sp_delta_max = new TextBox();
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
            statusStrip1.SuspendLayout();
            SuspendLayout();
            // 
            // joystickControl
            // 
            joystickControl.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
            joystickControl.BackColor = Color.Gray;
            joystickControl.Enabled = false;
            joystickControl.Location = new Point(615, 141);
            joystickControl.Name = "joystickControl";
            joystickControl.Size = new Size(300, 300);
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
            serialComsLabel.Location = new Point(12, 9);
            serialComsLabel.Name = "serialComsLabel";
            serialComsLabel.Size = new Size(132, 21);
            serialComsLabel.TabIndex = 2;
            serialComsLabel.Text = "Serial Connection";
            // 
            // button_sc2
            // 
            button_sc2.AutoSize = true;
            button_sc2.Location = new Point(67, 84);
            button_sc2.Name = "button_sc2";
            button_sc2.Size = new Size(54, 19);
            button_sc2.TabIndex = 3;
            button_sc2.Text = "Open";
            button_sc2.UseVisualStyleBackColor = true;
            button_sc2.CheckedChanged += button_sc2_CheckedChanged;
            // 
            // button_sc1
            // 
            button_sc1.Location = new Point(12, 84);
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
            statusStrip1.Location = new Point(0, 444);
            statusStrip1.Name = "statusStrip1";
            statusStrip1.Size = new Size(927, 22);
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
            textBox_baudSelect.Location = new Point(95, 56);
            textBox_baudSelect.Name = "textBox_baudSelect";
            textBox_baudSelect.Size = new Size(49, 23);
            textBox_baudSelect.TabIndex = 7;
            textBox_baudSelect.Text = "115200";
            // 
            // label_comSelect
            // 
            label_comSelect.AutoSize = true;
            label_comSelect.Font = new Font("Segoe UI", 9F);
            label_comSelect.Location = new Point(12, 35);
            label_comSelect.Name = "label_comSelect";
            label_comSelect.Size = new Size(35, 15);
            label_comSelect.TabIndex = 8;
            label_comSelect.Text = "PORT";
            // 
            // label_baudSelect
            // 
            label_baudSelect.AutoSize = true;
            label_baudSelect.Location = new Point(12, 59);
            label_baudSelect.Name = "label_baudSelect";
            label_baudSelect.Size = new Size(38, 15);
            label_baudSelect.TabIndex = 10;
            label_baudSelect.Text = "BAUD";
            // 
            // textBox_portSelect
            // 
            textBox_portSelect.Location = new Point(95, 35);
            textBox_portSelect.Name = "textBox_portSelect";
            textBox_portSelect.Size = new Size(49, 23);
            textBox_portSelect.TabIndex = 9;
            textBox_portSelect.Text = "COM16";
            // 
            // button_trackpadEnable
            // 
            button_trackpadEnable.Location = new Point(719, 92);
            button_trackpadEnable.Name = "button_trackpadEnable";
            button_trackpadEnable.Size = new Size(92, 23);
            button_trackpadEnable.TabIndex = 12;
            button_trackpadEnable.Text = "Enable";
            button_trackpadEnable.UseVisualStyleBackColor = true;
            button_trackpadEnable.Click += button_trackpadEnable_Click;
            // 
            // textBox_pwmDuty
            // 
            textBox_pwmDuty.Location = new Point(127, 330);
            textBox_pwmDuty.Name = "textBox_pwmDuty";
            textBox_pwmDuty.Size = new Size(45, 23);
            textBox_pwmDuty.TabIndex = 13;
            textBox_pwmDuty.Text = "0.0";
            // 
            // textBox_voltage
            // 
            textBox_voltage.Location = new Point(127, 369);
            textBox_voltage.Name = "textBox_voltage";
            textBox_voltage.Size = new Size(45, 23);
            textBox_voltage.TabIndex = 14;
            textBox_voltage.Text = "0.0";
            // 
            // button_pwm
            // 
            button_pwm.Location = new Point(12, 330);
            button_pwm.Name = "button_pwm";
            button_pwm.Size = new Size(109, 23);
            button_pwm.TabIndex = 15;
            button_pwm.Text = "PWM %";
            button_pwm.UseVisualStyleBackColor = true;
            button_pwm.Click += button_pwm_Click;
            // 
            // button_setVoltage
            // 
            button_setVoltage.Location = new Point(12, 369);
            button_setVoltage.Name = "button_setVoltage";
            button_setVoltage.Size = new Size(109, 23);
            button_setVoltage.TabIndex = 16;
            button_setVoltage.Text = "Voltage";
            button_setVoltage.UseVisualStyleBackColor = true;
            button_setVoltage.Click += button_setVoltage_Click;
            // 
            // label1
            // 
            label1.AutoSize = true;
            label1.BackColor = SystemColors.Info;
            label1.Font = new Font("Segoe UI", 12F);
            label1.Location = new Point(12, 290);
            label1.Name = "label1";
            label1.Size = new Size(122, 21);
            label1.TabIndex = 17;
            label1.Text = "Output Controls";
            // 
            // textBox_byteOutput
            // 
            textBox_byteOutput.Location = new Point(127, 407);
            textBox_byteOutput.Name = "textBox_byteOutput";
            textBox_byteOutput.Size = new Size(45, 23);
            textBox_byteOutput.TabIndex = 18;
            // 
            // button_byteOutput
            // 
            button_byteOutput.Location = new Point(12, 407);
            button_byteOutput.Name = "button_byteOutput";
            button_byteOutput.Size = new Size(109, 23);
            button_byteOutput.TabIndex = 19;
            button_byteOutput.Text = "Byte";
            button_byteOutput.UseVisualStyleBackColor = true;
            button_byteOutput.Click += button_byteOutput_Click;
            // 
            // textBox_duty_sp
            // 
            textBox_duty_sp.Location = new Point(355, 83);
            textBox_duty_sp.Name = "textBox_duty_sp";
            textBox_duty_sp.Size = new Size(58, 23);
            textBox_duty_sp.TabIndex = 20;
            textBox_duty_sp.TextChanged += textBox_duty_sp_TextChanged;
            // 
            // textBox_duty_sp_delta_max
            // 
            textBox_duty_sp_delta_max.Location = new Point(355, 112);
            textBox_duty_sp_delta_max.Name = "textBox_duty_sp_delta_max";
            textBox_duty_sp_delta_max.Size = new Size(58, 23);
            textBox_duty_sp_delta_max.TabIndex = 21;
            textBox_duty_sp_delta_max.TextChanged += textBox_duty_sp_delta_max_TextChanged;
            // 
            // label2
            // 
            label2.AutoSize = true;
            label2.BackColor = SystemColors.Info;
            label2.Font = new Font("Segoe UI", 12F);
            label2.Location = new Point(365, 9);
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
            label3.Location = new Point(281, 85);
            label3.Name = "label3";
            label3.Size = new Size(64, 21);
            label3.TabIndex = 23;
            label3.Text = "duty_sp";
            // 
            // label4
            // 
            label4.AutoSize = true;
            label4.BackColor = SystemColors.Info;
            label4.Font = new Font("Segoe UI", 12F);
            label4.Location = new Point(244, 112);
            label4.Name = "label4";
            label4.Size = new Size(105, 21);
            label4.TabIndex = 24;
            label4.Text = "duty_sp_delta";
            // 
            // button_auto_engage
            // 
            button_auto_engage.Location = new Point(377, 36);
            button_auto_engage.Name = "button_auto_engage";
            button_auto_engage.Size = new Size(95, 22);
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
            label5.Location = new Point(424, 83);
            label5.Name = "label5";
            label5.Size = new Size(28, 21);
            label5.TabIndex = 27;
            label5.Text = "KP";
            // 
            // textBox_kp
            // 
            textBox_kp.Location = new Point(462, 83);
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
            label6.Location = new Point(424, 115);
            label6.Name = "label6";
            label6.Size = new Size(23, 21);
            label6.TabIndex = 29;
            label6.Text = "KI";
            // 
            // textBox_ki
            // 
            textBox_ki.Location = new Point(462, 115);
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
            rx_buf_indicator.Location = new Point(12, 112);
            rx_buf_indicator.Name = "rx_buf_indicator";
            rx_buf_indicator.RightToLeft = RightToLeft.No;
            rx_buf_indicator.Size = new Size(100, 23);
            rx_buf_indicator.TabIndex = 30;
            // 
            // panel_off_left
            // 
            panel_off_left.Location = new Point(285, 174);
            panel_off_left.Name = "panel_off_left";
            panel_off_left.Size = new Size(26, 60);
            panel_off_left.TabIndex = 31;
            // 
            // panel_left_sensor
            // 
            panel_left_sensor.Location = new Point(342, 174);
            panel_left_sensor.Name = "panel_left_sensor";
            panel_left_sensor.Size = new Size(26, 60);
            panel_left_sensor.TabIndex = 32;
            // 
            // panel_mid
            // 
            panel_mid.Location = new Point(377, 174);
            panel_mid.Name = "panel_mid";
            panel_mid.Size = new Size(26, 60);
            panel_mid.TabIndex = 33;
            // 
            // panel_off_right
            // 
            panel_off_right.Location = new Point(462, 174);
            panel_off_right.Name = "panel_off_right";
            panel_off_right.Size = new Size(26, 60);
            panel_off_right.TabIndex = 35;
            // 
            // panel_right_sensor
            // 
            panel_right_sensor.Location = new Point(414, 174);
            panel_right_sensor.Name = "panel_right_sensor";
            panel_right_sensor.Size = new Size(26, 60);
            panel_right_sensor.TabIndex = 34;
            // 
            // label7
            // 
            label7.AutoSize = true;
            label7.BackColor = SystemColors.Info;
            label7.Font = new Font("Segoe UI", 12F);
            label7.Location = new Point(342, 150);
            label7.Name = "label7";
            label7.Size = new Size(98, 21);
            label7.TabIndex = 36;
            label7.Text = "Line Position";
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(7F, 15F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(927, 466);
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
            Controls.Add(textBox_duty_sp_delta_max);
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
        private TextBox textBox_duty_sp_delta_max;
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
    }
}
