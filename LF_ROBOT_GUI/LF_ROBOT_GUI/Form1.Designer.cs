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
            statusStrip1.SuspendLayout();
            SuspendLayout();
            // 
            // joystickControl
            // 
            joystickControl.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
            joystickControl.BackColor = Color.Gray;
            joystickControl.Enabled = false;
            joystickControl.Location = new Point(311, 76);
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
            statusStrip1.Location = new Point(0, 380);
            statusStrip1.Name = "statusStrip1";
            statusStrip1.Size = new Size(674, 22);
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
            textBox_baudSelect.Text = "9600";
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
            button_trackpadEnable.Location = new Point(418, 27);
            button_trackpadEnable.Name = "button_trackpadEnable";
            button_trackpadEnable.Size = new Size(92, 23);
            button_trackpadEnable.TabIndex = 12;
            button_trackpadEnable.Text = "Enable";
            button_trackpadEnable.UseVisualStyleBackColor = true;
            button_trackpadEnable.Click += button_trackpadEnable_Click;
            // 
            // textBox_pwmDuty
            // 
            textBox_pwmDuty.Location = new Point(205, 234);
            textBox_pwmDuty.Name = "textBox_pwmDuty";
            textBox_pwmDuty.Size = new Size(45, 23);
            textBox_pwmDuty.TabIndex = 13;
            textBox_pwmDuty.Text = "0.0";
            // 
            // textBox_voltage
            // 
            textBox_voltage.Location = new Point(205, 273);
            textBox_voltage.Name = "textBox_voltage";
            textBox_voltage.Size = new Size(45, 23);
            textBox_voltage.TabIndex = 14;
            textBox_voltage.Text = "0.0";
            // 
            // button_pwm
            // 
            button_pwm.Location = new Point(67, 234);
            button_pwm.Name = "button_pwm";
            button_pwm.Size = new Size(109, 23);
            button_pwm.TabIndex = 15;
            button_pwm.Text = "PWM %";
            button_pwm.UseVisualStyleBackColor = true;
            button_pwm.Click += button_pwm_Click;
            // 
            // button_setVoltage
            // 
            button_setVoltage.Location = new Point(67, 273);
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
            label1.Location = new Point(67, 194);
            label1.Name = "label1";
            label1.Size = new Size(122, 21);
            label1.TabIndex = 17;
            label1.Text = "Output Controls";
            // 
            // textBox_byteOutput
            // 
            textBox_byteOutput.Location = new Point(205, 311);
            textBox_byteOutput.Name = "textBox_byteOutput";
            textBox_byteOutput.Size = new Size(45, 23);
            textBox_byteOutput.TabIndex = 18;
            // 
            // button_byteOutput
            // 
            button_byteOutput.Location = new Point(67, 311);
            button_byteOutput.Name = "button_byteOutput";
            button_byteOutput.Size = new Size(109, 23);
            button_byteOutput.TabIndex = 19;
            button_byteOutput.Text = "Byte";
            button_byteOutput.UseVisualStyleBackColor = true;
            button_byteOutput.Click += button_byteOutput_Click;
            // 
            // Form1
            // 
            AutoScaleDimensions = new SizeF(7F, 15F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(674, 402);
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
    }
}
