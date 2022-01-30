package frc.robot.modules.common;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation;

public class Input {

	public static interface AnalogSupplier { double get(); }
    public static interface DigitalSupplier { boolean get(); }


    public static class InputDevice extends GenericHID {

        private PovButton[] buttons = null;
        private boolean verifyInfo() {
            if(super.isConnected() && this.buttons == null) {
                this.buttons = new PovButton[super.getButtonCount() + (super.getPOVCount() * 4)];
                return true;
            } else if(!super.isConnected() && this.buttons != null) {
                this.buttons = null;
                return false;
            }
            return this.buttons != null;
        }

        public InputDevice(int p) { 
            super(p); 
            this.verifyInfo();
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

        public PovButton getCallback(int button) {  // returns null if joystick not connected
            if(this.verifyInfo()) {
                if(this.buttons[button-1] == null) {
                    this.buttons[button-1] = new PovButton(this, button);
                }
                return this.buttons[button-1];
            }
            return PovButton.dummy;
        }

    }

    /** Converts any pov's on a controller into button values past those that would normally be assigned */
	public static class PovButton extends Button {
        public static final PovButton dummy = new PovButton();
		private final int 
			port, button;
		private final boolean
			use_pov;

        private PovButton() {
            this.port = DriverStation.kJoystickPorts-1;
            this.button = 0;
            this.use_pov = false;
        }
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
        @Override public Button whenPressed(final Command command, boolean interruptible) {
            if(this != PovButton.dummy) { super.whenActive(command, interruptible); }
            return this;
        }
        @Override public Button whenPressed(final Command command) {
            if(this != PovButton.dummy) { super.whenActive(command); }
            return this;
        }
        @Override public Button whenPressed(final Runnable toRun, Subsystem... requirements) {
            if(this != PovButton.dummy) { super.whenActive(toRun, requirements); }
            return this;
        }
        @Override public Button whileHeld(final Command command, boolean interruptible) {
            if(this != PovButton.dummy) { super.whileActiveContinuous(command, interruptible); }
            return this;
        }
        @Override public Button whileHeld(final Command command) {
            if(this != PovButton.dummy) { super.whileActiveContinuous(command); }
            return this;
        }
        @Override public Button whileHeld(final Runnable toRun, Subsystem... requirements) {
            if(this != PovButton.dummy) { super.whileActiveContinuous(toRun, requirements); }
            return this;
        }
        @Override public Button whenHeld(final Command command, boolean interruptible) {
            if(this != PovButton.dummy) { super.whileActiveOnce(command, interruptible); }
            return this;
        }
        @Override public Button whenHeld(final Command command) {
            if(this != PovButton.dummy) { super.whileActiveOnce(command, true); }
            return this;
        }
        @Override public Button whenReleased(final Command command, boolean interruptible) {
            if(this != PovButton.dummy) { super.whenInactive(command, interruptible); }
            return this;
        }
        @Override public Button whenReleased(final Command command) {
            if(this != PovButton.dummy) { super.whenInactive(command); }
            return this;
        }
        @Override public Button whenReleased(final Runnable toRun, Subsystem... requirements) {
            if(this != PovButton.dummy) { super.whenInactive(toRun, requirements); }
            return this;
        }
        @Override public Button toggleWhenPressed(final Command command, boolean interruptible) {
            if(this != PovButton.dummy) { super.toggleWhenActive(command, interruptible); }
            return this;
        }
        @Override public Button toggleWhenPressed(final Command command) {
            if(this != PovButton.dummy) { super.toggleWhenActive(command); }
            return this;
        }
        @Override public Button cancelWhenPressed(final Command command) {
            if(this != PovButton.dummy) { super.cancelWhenActive(command); }
            return this;
        }
	}

    public static interface AnalogMap {
        boolean compatible(GenericHID i);
        boolean compatible(int p);
        double getValue(GenericHID i);
        double getValue(int p);
        AnalogSupplier getCallback(InputDevice i);
        AnalogSupplier getCallback(int p);
    }
    public static interface DigitalMap {
        boolean compatible(GenericHID i);
        boolean compatible(int p);
        boolean isPovBind(GenericHID i);
        boolean isPovBind(int p);
        int getPovBind(GenericHID i);
        int getPovBind(int p);
        PovButton getButton(InputDevice i);
        PovButton getButton(GenericHID i);
        PovButton getButton(int p);
        boolean getValue(GenericHID i);
        boolean getValue(int p);
        DigitalSupplier getCallback(GenericHID i);
        DigitalSupplier getCallback(int p);
    }


	public static class Xbox {
        public static enum Analog implements AnalogMap {
            LX(0), RX(4), LY(1), RY(5), LT(2), RT(3), 
            TOTAL(6);
        
            public final int value;
            private Analog(int value) { this.value = value; }

            public boolean compatible(GenericHID i) { return i.getAxisCount() == TOTAL.value; }
            public boolean compatible(int p) { return DriverStation.getStickAxisCount(p) == TOTAL.value; }
            public double getValue(GenericHID i) {
                if(this.compatible(i)) {
                    return i.getRawAxis(this.value);
                }
                return 0.0;
            }
            public double getValue(int p) {
                if(this.compatible(p)) {
                    return DriverStation.getStickAxis(p, this.value);
                }
                return 0.0;
            }
            public AnalogSupplier getCallback(InputDevice i) {
                if(this.compatible(i)) {
                    return ()->i.getRawAxis(this.value);
                }
                return ()->0.0;
            }
            public AnalogSupplier getCallback(int p) {
                if(this.compatible(p)) {
                    return ()->DriverStation.getStickAxis(p, this.value);
                }
                return ()->0.0;
            }
        }
        public static enum Digital implements DigitalMap {
            LB(5), RB(6), LS(9), RS(10),
            A(1), B(2), X(3), Y(4),
            BACK(7), START(8),
            BUTTONS(10),
            DT(11), DR(12), DB(13), DL(14),  // Dpad buttons (only valid with PovButton objects)
            POVS(1),
            TOTAL(14);
        
            public final int value;
            private Digital(int value) { this.value = value; }

            public boolean compatible(GenericHID i) {
                return i.getButtonCount() + (i.getPOVCount()*4) == TOTAL.value;
            }
            public boolean compatible(int p) {
                return DriverStation.getStickButtonCount(p) + (DriverStation.getStickPOVCount(p)*4) == TOTAL.value;
            }
            public boolean isPovBind(GenericHID i) {
                if(this.compatible(i)) {
                    return this.value > i.getButtonCount();
                }
                return false;
            }
            public boolean isPovBind(int p) {
                if(this.compatible(p)) {
                    return this.value > DriverStation.getStickPOVCount(p);
                }
                return false;
            }
            public int getPovBind(GenericHID i) {
                if(this.compatible(i) && this.isPovBind(i)) {
                    return (this.value - i.getButtonCount())/4;
                }
                return -1;
            }
            public int getPovBind(int p) {
                if(this.compatible(p) && this.isPovBind(p)) {
                    return (this.value - DriverStation.getStickButtonCount(p))/4;
                }
                return -1;
            }
            public PovButton getButton(InputDevice i) {
                if(this.compatible(i)) {
                    return i.getCallback(this.value);
                }
                return PovButton.dummy;
            }
            public PovButton getButton(GenericHID i) {
                if(this.compatible(i)) {
                    return new PovButton(i, this.value);
                }
                return PovButton.dummy;
            }
            public PovButton getButton(int p) {
                if(this.compatible(p)) {
                    return new PovButton(p, this.value);
                }
                return PovButton.dummy;
            }
            public boolean getValue(GenericHID i) {
                if(this.isPovBind(i)) {
                    return (i.getPOV((this.value-i.getButtonCount()-1) / 4) / 90.0 + 1) == this.value-i.getButtonCount();
                } else if(this.compatible(i)) {
                    return i.getRawButton(this.value);
                }
                return false;
            }
            public boolean getValue(int p) {
                if(this.isPovBind(p)) {
                    return (DriverStation.getStickPOV(p, (this.value-DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.value-DriverStation.getStickButtonCount(p);
                } else if(this.compatible(p)) {
                    return DriverStation.getStickButton(p, this.value);
                }
                return false;
            }
            public DigitalSupplier getCallback(GenericHID i) {
                if(this.isPovBind(i)) {
                    return ()->(i.getPOV((this.value-i.getButtonCount()-1) / 4) / 90.0 + 1) == this.value-i.getButtonCount();
                } else if(this.compatible(i)) {
                    return ()->i.getRawButton(this.value);
                }
                return ()->false;
            }
            public DigitalSupplier getCallback(int p) {
                if(this.isPovBind(p)) {
                    return ()->(DriverStation.getStickPOV(p, (this.value-DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.value-DriverStation.getStickButtonCount(p);
                } else if(this.compatible(p)) {
                    return ()->DriverStation.getStickButton(p, this.value);
                }
                return ()->false;
            }
        }
    }
    public static class PlayStation {
        public static enum Analog implements AnalogMap {
            LX(0), LY(1), RX(2), RY(5), LT(3), RT(4),
            TOTAL(6);

            public final int value;
            private Analog(int value) { this.value = value; }

            public boolean compatible(GenericHID i) { return i.getAxisCount() == TOTAL.value; }
            public boolean compatible(int p) { return DriverStation.getStickAxisCount(p) == TOTAL.value; }
            public double getValue(GenericHID i) {
                if(this.compatible(i)) {
                    return i.getRawAxis(this.value);
                }
                return 0.0;
            }
            public double getValue(int p) {
                if(this.compatible(p)) {
                    return DriverStation.getStickAxis(p, this.value);
                }
                return 0.0;
            }
            public AnalogSupplier getCallback(InputDevice i) {
                if(this.compatible(i)) {
                    return ()->i.getRawAxis(this.value);
                }
                return ()->0.0;
            }
            public AnalogSupplier getCallback(int p) {
                if(this.compatible(p)) {
                    return ()->DriverStation.getStickAxis(p, this.value);
                }
                return ()->0.0;
            }
        }
        public static enum Digital {
            SQR(1), X(2), O(3), TRI(4),     // square, cross, circle, triangle
            LB(5), RB(6), L2(7), R2(8),     // right-bumper, left-bumper,   left-trigger, right-trigger (button mode)
            SHARE(9), OPT(10), PS(13),      // share, options, ps button
            TOUCH(14), LS(11), RS(12),      // touchpad, left-stick, right-stick
            BUTTONS(14), POVS(0), TOTAL(14);

            public final int value;
            private Digital(int value) { this.value = value; }

            public boolean compatible(GenericHID i) {
                return i.getButtonCount() + (i.getPOVCount()*4) == TOTAL.value;
            }
            public boolean compatible(int p) {
                return DriverStation.getStickButtonCount(p) + (DriverStation.getStickPOVCount(p)*4) == TOTAL.value;
            }
            public boolean isPovBind(GenericHID i) {
                if(this.compatible(i)) {
                    return this.value > i.getButtonCount();
                }
                return false;
            }
            public boolean isPovBind(int p) {
                if(this.compatible(p)) {
                    return this.value > DriverStation.getStickPOVCount(p);
                }
                return false;
            }
            public int getPovBind(GenericHID i) {
                if(this.compatible(i) && this.isPovBind(i)) {
                    return (this.value - i.getButtonCount())/4;
                }
                return -1;
            }
            public int getPovBind(int p) {
                if(this.compatible(p) && this.isPovBind(p)) {
                    return (this.value - DriverStation.getStickButtonCount(p))/4;
                }
                return -1;
            }
            public PovButton getButton(InputDevice i) {
                if(this.compatible(i)) {
                    return i.getCallback(this.value);
                }
                return PovButton.dummy;
            }
            public PovButton getButton(GenericHID i) {
                if(this.compatible(i)) {
                    return new PovButton(i, this.value);
                }
                return PovButton.dummy;
            }
            public PovButton getButton(int p) {
                if(this.compatible(p)) {
                    return new PovButton(p, this.value);
                }
                return PovButton.dummy;
            }
            public boolean getValue(GenericHID i) {
                if(this.isPovBind(i)) {
                    return (i.getPOV((this.value-i.getButtonCount()-1) / 4) / 90.0 + 1) == this.value-i.getButtonCount();
                } else if(this.compatible(i)) {
                    return i.getRawButton(this.value);
                }
                return false;
            }
            public boolean getValue(int p) {
                if(this.isPovBind(p)) {
                    return (DriverStation.getStickPOV(p, (this.value-DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.value-DriverStation.getStickButtonCount(p);
                } else if(this.compatible(p)) {
                    return DriverStation.getStickButton(p, this.value);
                }
                return false;
            }
            public DigitalSupplier getCallback(GenericHID i) {
                if(this.isPovBind(i)) {
                    return ()->(i.getPOV((this.value-i.getButtonCount()-1) / 4) / 90.0 + 1) == this.value-i.getButtonCount();
                } else if(this.compatible(i)) {
                    return ()->i.getRawButton(this.value);
                }
                return ()->false;
            }
            public DigitalSupplier getCallback(int p) {
                if(this.isPovBind(p)) {
                    return ()->(DriverStation.getStickPOV(p, (this.value-DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.value-DriverStation.getStickButtonCount(p);
                } else if(this.compatible(p)) {
                    return ()->DriverStation.getStickButton(p, this.value);
                }
                return ()->false;
            }
        }
    }
    public static class Attack3 {
        public static enum Analog  implements AnalogMap {
            X(0), Y(1), S(2),   // ~ X-Axis, Y-Axis, Slider thing on the bottom
            TOTAL(3);

            public final int value;
            private Analog(int value) { this.value = value; }

            public boolean compatible(GenericHID i) { return i.getAxisCount() == TOTAL.value; }
            public boolean compatible(int p) { return DriverStation.getStickAxisCount(p) == TOTAL.value; }
            public double getValue(GenericHID i) {
                if(this.compatible(i)) {
                    return i.getRawAxis(this.value);
                }
                return 0.0;
            }
            public double getValue(int p) {
                if(this.compatible(p)) {
                    return DriverStation.getStickAxis(p, this.value);
                }
                return 0.0;
            }
            public AnalogSupplier getCallback(InputDevice i) {
                if(this.compatible(i)) {
                    return ()->i.getRawAxis(this.value);
                }
                return ()->0.0;
            }
            public AnalogSupplier getCallback(int p) {
                if(this.compatible(p)) {
                    return ()->DriverStation.getStickAxis(p, this.value);
                }
                return ()->0.0;
            }
        }
        public static enum Digital {
            TRI(1), TB(2), TT(3), TL(4), TR(5),             // ~ trigger, top-bottom, top-top, top-left, top-right
            B1(6), B2(7), B3(8), B4(9), B5(10), B6(11),     // ~ buttons on the base of the joystick (labeled)
            BUTTONS(11), POVS(0), TOTAL(11);

            public final int value;
            private Digital(int value) { this.value = value; }

            public boolean compatible(GenericHID i) {
                return i.getButtonCount() + (i.getPOVCount()*4) == TOTAL.value;
            }
            public boolean compatible(int p) {
                return DriverStation.getStickButtonCount(p) + (DriverStation.getStickPOVCount(p)*4) == TOTAL.value;
            }
            public boolean isPovBind(GenericHID i) {
                if(this.compatible(i)) {
                    return this.value > i.getButtonCount();
                }
                return false;
            }
            public boolean isPovBind(int p) {
                if(this.compatible(p)) {
                    return this.value > DriverStation.getStickPOVCount(p);
                }
                return false;
            }
            public int getPovBind(GenericHID i) {
                if(this.compatible(i) && this.isPovBind(i)) {
                    return (this.value - i.getButtonCount())/4;
                }
                return -1;
            }
            public int getPovBind(int p) {
                if(this.compatible(p) && this.isPovBind(p)) {
                    return (this.value - DriverStation.getStickButtonCount(p))/4;
                }
                return -1;
            }
            public PovButton getButton(InputDevice i) {
                if(this.compatible(i)) {
                    return i.getCallback(this.value);
                }
                return PovButton.dummy;
            }
            public PovButton getButton(GenericHID i) {
                if(this.compatible(i)) {
                    return new PovButton(i, this.value);
                }
                return PovButton.dummy;
            }
            public PovButton getButton(int p) {
                if(this.compatible(p)) {
                    return new PovButton(p, this.value);
                }
                return PovButton.dummy;
            }
            public boolean getValue(GenericHID i) {
                if(this.isPovBind(i)) {
                    return (i.getPOV((this.value-i.getButtonCount()-1) / 4) / 90.0 + 1) == this.value-i.getButtonCount();
                } else if(this.compatible(i)) {
                    return i.getRawButton(this.value);
                }
                return false;
            }
            public boolean getValue(int p) {
                if(this.isPovBind(p)) {
                    return (DriverStation.getStickPOV(p, (this.value-DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.value-DriverStation.getStickButtonCount(p);
                } else if(this.compatible(p)) {
                    return DriverStation.getStickButton(p, this.value);
                }
                return false;
            }
            public DigitalSupplier getCallback(GenericHID i) {
                if(this.isPovBind(i)) {
                    return ()->(i.getPOV((this.value-i.getButtonCount()-1) / 4) / 90.0 + 1) == this.value-i.getButtonCount();
                } else if(this.compatible(i)) {
                    return ()->i.getRawButton(this.value);
                }
                return ()->false;
            }
            public DigitalSupplier getCallback(int p) {
                if(this.isPovBind(p)) {
                    return ()->(DriverStation.getStickPOV(p, (this.value-DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.value-DriverStation.getStickButtonCount(p);
                } else if(this.compatible(p)) {
                    return ()->DriverStation.getStickButton(p, this.value);
                }
                return ()->false;
            }
        }
    }
    public static class Extreme3d {
        public static enum Analog  implements AnalogMap {
            X(0), Y(1), Z(2), S(3),     // x-axis, y-axis, swivell-axis, slider-axis
            TOTAL(4);

            public final int value;
            private Analog(int value) { this.value = value; }

            public boolean compatible(GenericHID i) { return i.getAxisCount() == TOTAL.value; }
            public boolean compatible(int p) { return DriverStation.getStickAxisCount(p) == TOTAL.value; }
            public double getValue(GenericHID i) {
                if(this.compatible(i)) {
                    return i.getRawAxis(this.value);
                }
                return 0.0;
            }
            public double getValue(int p) {
                if(this.compatible(p)) {
                    return DriverStation.getStickAxis(p, this.value);
                }
                return 0.0;
            }
            public AnalogSupplier getCallback(InputDevice i) {
                if(this.compatible(i)) {
                    return ()->i.getRawAxis(this.value);
                }
                return ()->0.0;
            }
            public AnalogSupplier getCallback(int p) {
                if(this.compatible(p)) {
                    return ()->DriverStation.getStickAxis(p, this.value);
                }
                return ()->0.0;
            }
        }
        public static enum Digital {
            TRI(1), SIDE(2), TLB(3), TRB(4), TLT(5), TRT(6),    // trigger, side, top-left-bottom, top-right-bottom, top-left-top, top-right-top
            B7(7), B8(8), B9(9), B10(10), B11(11), B12(12),     // as printed on the actual joystick
            BUTTONS(12), POVS(0), TOTAL(12);

            public final int value;
            private Digital(int value) { this.value = value; }

            public boolean compatible(GenericHID i) {
                return i.getButtonCount() + (i.getPOVCount()*4) == TOTAL.value;
            }
            public boolean compatible(int p) {
                return DriverStation.getStickButtonCount(p) + (DriverStation.getStickPOVCount(p)*4) == TOTAL.value;
            }
            public boolean isPovBind(GenericHID i) {
                if(this.compatible(i)) {
                    return this.value > i.getButtonCount();
                }
                return false;
            }
            public boolean isPovBind(int p) {
                if(this.compatible(p)) {
                    return this.value > DriverStation.getStickPOVCount(p);
                }
                return false;
            }
            public int getPovBind(GenericHID i) {
                if(this.compatible(i) && this.isPovBind(i)) {
                    return (this.value - i.getButtonCount())/4;
                }
                return -1;
            }
            public int getPovBind(int p) {
                if(this.compatible(p) && this.isPovBind(p)) {
                    return (this.value - DriverStation.getStickButtonCount(p))/4;
                }
                return -1;
            }
            public PovButton getButton(InputDevice i) {
                if(this.compatible(i)) {
                    return i.getCallback(this.value);
                }
                return PovButton.dummy;
            }
            public PovButton getButton(GenericHID i) {
                if(this.compatible(i)) {
                    return new PovButton(i, this.value);
                }
                return PovButton.dummy;
            }
            public PovButton getButton(int p) {
                if(this.compatible(p)) {
                    return new PovButton(p, this.value);
                }
                return PovButton.dummy;
            }
            public boolean getValue(GenericHID i) {
                if(this.isPovBind(i)) {
                    return (i.getPOV((this.value-i.getButtonCount()-1) / 4) / 90.0 + 1) == this.value-i.getButtonCount();
                } else if(this.compatible(i)) {
                    return i.getRawButton(this.value);
                }
                return false;
            }
            public boolean getValue(int p) {
                if(this.isPovBind(p)) {
                    return (DriverStation.getStickPOV(p, (this.value-DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.value-DriverStation.getStickButtonCount(p);
                } else if(this.compatible(p)) {
                    return DriverStation.getStickButton(p, this.value);
                }
                return false;
            }
            public DigitalSupplier getCallback(GenericHID i) {
                if(this.isPovBind(i)) {
                    return ()->(i.getPOV((this.value-i.getButtonCount()-1) / 4) / 90.0 + 1) == this.value-i.getButtonCount();
                } else if(this.compatible(i)) {
                    return ()->i.getRawButton(this.value);
                }
                return ()->false;
            }
            public DigitalSupplier getCallback(int p) {
                if(this.isPovBind(p)) {
                    return ()->(DriverStation.getStickPOV(p, (this.value-DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.value-DriverStation.getStickButtonCount(p);
                } else if(this.compatible(p)) {
                    return ()->DriverStation.getStickButton(p, this.value);
                }
                return ()->false;
            }
        }
    }

}