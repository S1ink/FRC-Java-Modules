package frc.robot.modules.common;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation;

public class Input {

	public static interface AnalogSupplier { double get(); }
    public static interface DigitalSupplier { boolean get(); }


    public static class InputDevice extends GenericHID {

        private final PovButton[] buttons;

        public InputDevice(int p) { 
            super(p); 
            this.buttons = new PovButton[super.getButtonCount() + (super.getPOVCount() * 4)];
        }

        @Override
        public boolean getRawButton(int button) {
            int _bc = super.getButtonCount();
            if((button > _bc) && (button <= _bc + (super.getPOVCount() * 4))) {
                button = button - _bc;
                return (super.getPOV((button-1) / 4) / 90.0 + 1) == button;
            } else {
                return super.getRawButton(button);
            }
        }

        public PovButton getCallback(int button) {
            if(this.buttons[button-1] == null) {
                this.buttons[button-1] = new PovButton(this, button);
            }
            return this.buttons[button-1];
        }

    }

    /** Converts any pov's on a controller into button values past those that would normally be assigned */
	public static class PovButton extends Button {
		private final int 
			port, button;
		private final boolean
			use_pov;

		public PovButton(GenericHID device, int button) { this(device.getPort(), button); }
		public PovButton(int port, int button) {
			this.port = port;
            int _bc = DriverStation.getStickButtonCount(this.port), _pc = DriverStation.getStickPOVCount(this.port);
            this.use_pov = ((button > _bc) && (button <= _bc + (_pc * 4)));
            if (this.use_pov) { 
				this.button = button - _bc; 
			} else { 
				this.button = button; 
			}
		}
		@Override
        public boolean get() {
            if (this.use_pov) {
                return (DriverStation.getStickPOV(this.port, (this.button-1) / 4) / 90.0 + 1) == this.button;
            } else {
                return DriverStation.getStickButton(this.port, this.button);
            }
        }

	}


	public static class Xbox {
        public static enum Digital {
            LB(5), RB(6), LS(9), RS(10),
            A(1), B(2), X(3), Y(4),
            BACK(7), START(8),
            DT(11), DR(12), DB(13), DL(14);     // Dpad buttons (only valid with XButton objects)
        
            public final int value;
            private Digital(int value) { this.value = value; }
        }
        public static enum Analog {
            LX(0), RX(4), LY(1), RY(5), LT(2), RT(3);
        
            public final int value;
            private Analog(int value) { this.value = value; }
        }
    }
    public static class PlayStation {
        public static enum Digital {
            SQR(1), X(2), O(3), TRI(4),     // square, cross, circle, triangle
            LB(5), RB(6), L2(7), R2(8),     // right-bumper, left-bumper,   left-trigger, right-trigger (button mode)
            SHARE(9), OPT(10), PS(13),      // share, options, ps button
            TOUCH(14), LS(11), RS(12);      // touchpad, left-stick, right-stick

            public final int value;
            private Digital(int value) { this.value = value; }
        }
        public static enum Analog {
            LX(0), LY(1), RX(2), RY(5), LT(3), RT(4);

            public final int value;
            private Analog(int value) { this.value = value; }
        }
    }
    public static class Attack3 {
        public static enum Digital {
            TRI(1), TB(2), TT(3), TL(4), TR(5),             // ~ trigger, top-bottom, top-top, top-left, top-right
            B1(6), B2(7), B3(8), B4(9), B5(10), B6(11);     // ~ buttons on the base of the joystick (labeled)

            public final int value;
            private Digital(int value) { this.value = value; }
        }
        public static enum Analog {
            X(0), Y(1), S(2);   // ~ X-Axis, Y-Axis, Slider thing on the bottom

            public final int value;
            private Analog(int value) { this.value = value; }
        }
    }
    public static class Extreme3d {
        public static enum Digital {
            TRI(1), SIDE(2), TLB(3), TRB(4), TLT(5), TRT(6),    // trigger, side, top-left-bottom, top-right-bottom, top-left-top, top-right-top
            B7(7), B8(8), B9(9), B10(10), B11(11), B12(12);     // as printed on the actual joystick

            public final int value;
            private Digital(int value) { this.value = value; }
        }
        public static enum Analog {
            X(0), Y(1), Z(2), S(3);     // x-axis, y-axis, swivell-axis, slider-axis

            public final int value;
            private Analog(int value) { this.value = value; }
        }
    }

}