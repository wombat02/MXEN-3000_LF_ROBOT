using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace LF_ROBOT_GUI
{
    public partial class JoystickControl : UserControl
    {
        private Point _center;
        private Point _knobPosition;
        
        private UInt16 _knobRadius= 30; // px
        private UInt16 _size = 500; // size of joystick (px)

        public Vector2 getPosRaw() { return new Vector2(_knobPosition.X, _knobPosition.Y); }
        
        /// <summary>
        /// Return a new Vector2 with X and Y ranging from -1 to 1
        /// </summary>
        /// <returns></returns>
        public Vector2 getPosNormalised ()
        {
            return new Vector2(_knobPosition.X - _center.X, _knobPosition.Y - _center.Y) / ( Math.Max ( Width, Height ) / 2 - _knobRadius );
        }

        public JoystickControl()
        {
            InitializeComponent();
            this.DoubleBuffered = true;                             // Reduce flicker during painting
            _center = new Point(this.Width / 2, this.Height / 2);
            _knobPosition = _center;                                // Start with knob at the center
            this.BackColor = Color.Gray;                            // Background color for the joystick area
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Graphics g = e.Graphics;

            // Draw joystick background (e.g., a circle)
            g.FillEllipse(Brushes.LightGray, 0, 0, this.Width, this.Height);

            // Draw joystick knob
            g.FillEllipse(Brushes.Blue, _knobPosition.X - _knobRadius, _knobPosition.Y - _knobRadius, 2*_knobRadius, 2*_knobRadius);
        }

        protected override void OnMouseDown(MouseEventArgs e)
        {
            base.OnMouseDown(e);
            UpdateKnobPosition(e.Location);
        }

        protected override void OnMouseMove(MouseEventArgs e)
        {
            base.OnMouseMove(e);
            if (e.Button == MouseButtons.Left)
            {
                UpdateKnobPosition(e.Location);
            }
        }

        protected override void OnMouseUp(MouseEventArgs e)
        {
            base.OnMouseUp(e);
            
            // Reset knob position to center when mouse is released
            _knobPosition = _center;
            Invalidate(); // Redraw the control
        }

        private void UpdateKnobPosition(Point mouseLocation)
        {
            // Calculate the new position of the knob
            _knobPosition = new Point(mouseLocation.X, mouseLocation.Y);

            // Ensure the knob stays within bounds
            _knobPosition.X = Math.Max(_knobRadius, Math.Min(_knobPosition.X, this.Width - _knobRadius));
            _knobPosition.Y = Math.Max(_knobRadius, Math.Min(_knobPosition.Y, this.Height - _knobRadius));

            Invalidate(); // Redraw the control
        }
    }
}
