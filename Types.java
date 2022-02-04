package frc.robot.modules.common;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Types {

	public static enum DriveLayout {
        DIFFERENTIAL	(new DriveMode[]{DriveMode.TANK, DriveMode.ARCADE, DriveMode.RACE, DriveMode.CURVATURE}),
        MECANUM			(DriveMode.values()),
        KILLOUGH		(new DriveMode[]{DriveMode.ARCADE, DriveMode.RACE, DriveMode.CURVATURE, DriveMode.TOP}),
        SWERVE			(DriveMode.values());

		public final DriveMode[] supported;
		private DriveLayout(DriveMode[] s) { this.supported = s; }
		public boolean supports(DriveMode m) {
			for(DriveMode mode : this.supported) {
				if(mode == m) { return true; }
			}
			return false;
		}
    }

    public static enum Inversions {
        NEITHER (false, false),
        LEFT    (true, false),
        RIGHT   (false, true),
        BOTH    (true, true);

        public final boolean left, right;
        Inversions(boolean l, boolean r) {
            this.left = l;
            this.right = r;
        }
        public String toString() { return "Inversions@" + this.hashCode() + ": {Left:" + this.left + " Right:" + this.right + "}"; }
    }

    public static enum DriveMode {
        TANK		(0),
        ARCADE		(1),
        RACE		(2),
        CURVATURE	(3),
        TOP			(4);

        public final int index;
        private DriveMode(int i) {
            this.index = i;
        }
        public String toString() { return "DriveMode@" + this.hashCode() + ": " + this.name(); }
    }
	public static interface Drivable {

        public DriveLayout getLayout();
		public MotorController[] getMotors();

		public void tankDrive(double l, double r);
		public void arcadeDrive(double s, double rot);
		public void raceDrive(double f, double b, double rot);
		public void curvatureDrive(double s, double rot, boolean q);
		public void topDownDrive(double x,  double y, double rot);

        public void autoTurn(double v); // turn by supplying "speed" -> positive for one direction, negative for the other
        public void autoDrive(double l, double r);

		public void feed();
    
	}
    public static class DriveModes {    // handles incrementing/decrementing and custom drivemode arrays

        private DriveMode[] modes = DriveMode.values();
        private int index = -1;

        public DriveModes(DriveMode m) {
            this.index = m.index;
        }
        public DriveModes(DriveMode m, DriveMode[] options) {
            this.modes = options;
            for(int i = 0; i < this.modes.length; i++) {
                if(this.modes[i] == m) {
                    this.index = i;
                }
            }
            if(this.index == -1) {
                this.index = 0;
            }
        }

        public DriveMode get() { return this.modes[this.index]; }
        public void set(DriveMode m) {
            this.index = -1;
            for(int i = 0; i < this.modes.length; i++) {
                if(this.modes[i] == m) {
                    this.index = i;
                }
            }
            if(this.index == -1) {
                this.index = 0;
            }
        }
        public int getIndex() { return this.index; }
        public DriveMode[] getOptions() { return this.modes; }
        public int getSize() { return this.modes.length; }

        public void setOptions(DriveMode[] o) { this.modes = o; }
        public void resetOptions() { this.modes = DriveMode.values(); }

        public DriveMode increment() {
            if (this.index + 1 < this.modes.length) {
                this.index++;
            }
            return this.modes[this.index];
        }
        public DriveMode increment(int v) {
            if (this.index + v < this.modes.length) {
                this.index += v;
            }
            return this.modes[this.index];
        }
        public DriveMode decrement() {
            if (this.index > 0) {
                this.index--;
            }
            return this.modes[this.index];
        }
        public DriveMode decrement(int v) {
            if (this.index - v >= 0) {
                this.index -= v;
            }
            return this.modes[this.index];
        }

    }
	
    public static enum Deceleration {
        _96	(0.96),
        _97 (0.97),
        _98	(0.98),
        _99	(0.99);

        public final double value;
        private Deceleration(double v) { this.value = v; }
        public String toString() { return "Deceleration@" + this.hashCode() + ": " + this.value; }
    }

	public static interface MotorSupplier<M extends MotorController> { M create(int p); }

    public static final class DB2 {     // a drivebase map containing two sides, each with one motor port

        //public static final DriveLayout[] supported = {DriveLayout.DIFFERENTIAL};
		public final DriveLayout layout = DriveLayout.DIFFERENTIAL;
		public final MotorController 
            left, 
            right;
        public final Inversions 
            invert;
        
        public<M extends MotorController> DB2(int l, int r, MotorSupplier<M> t) {
            this.left = t.create(l);
            this.right = t.create(r);
            this.invert = Inversions.NEITHER;
        }
        public<M extends MotorController> DB2(int l, int r, MotorSupplier<M> t, Inversions i) {
            this.left = DriveBase.inlineInverter(t.create(l), i.left);
            this.right = DriveBase.inlineInverter(t.create(r), i.right);
            this.invert = i;
        }

        public String toString() {
            return "DB2@" + this.hashCode() + ": {Left port:" + this.left + " Right port:" + this.right + "}\n >> " + this.invert.toString();
        }
		
    }
    public static final class DB3 {

        //public static final DriveLayout[] supported = {DriveLayout.KILLOUGH};
		public final DriveLayout layout = DriveLayout.KILLOUGH;
        public final MotorController
			left,
			mid,
			right;
		
		public<M extends MotorController> DB3(int l, int m, int r, MotorSupplier<M> t) {
			this.left = t.create(l);
			this.mid = t.create(m);
			this.right = t.create(r);
		}

    }
    public static final class DB4 {     // a drivebase map containing two sides, each with two motor ports
        
		public static final DriveLayout[] supported = {DriveLayout.DIFFERENTIAL, DriveLayout.MECANUM};
		public final DriveLayout layout;
		public final MotorController 
            front_left,
            front_right,
            back_left,
            back_right;
        public final Inversions
            invert;

        public<M extends MotorController> DB4(int fl, int fr, int bl, int br, MotorSupplier<M> t) {
            this.front_left = t.create(fl);
            this.front_right = t.create(fr);
            this.back_left = t.create(bl);
            this.back_right = t.create(br);
            this.invert = Inversions.NEITHER;
			this.layout = DriveLayout.DIFFERENTIAL;
        }
        public<M extends MotorController> DB4(int fl, int fr, int bl, int br, MotorSupplier<M> t, Inversions i) {
            this.front_left = DriveBase.inlineInverter(t.create(fl), i.left);
            this.front_right = DriveBase.inlineInverter(t.create(fr), i.right);
            this.back_left = DriveBase.inlineInverter(t.create(bl), i.left);
            this.back_right = DriveBase.inlineInverter(t.create(br), i.right);
            this.invert = i;
			this.layout = DriveLayout.DIFFERENTIAL;
        }
		public<M extends MotorController> DB4(int fl, int fr, int bl, int br, MotorSupplier<M> t, DriveLayout l) {
			this.front_left = t.create(fl);
            this.front_right = t.create(fr);
            this.back_left = t.create(bl);
            this.back_right = t.create(br);
            this.invert = Inversions.NEITHER;
			for(DriveLayout lo : supported) {
				if(l == lo) {
					this.layout = l;
					return;
				}
			}
			this.layout = supported[0];
		}
		public<M extends MotorController> DB4(int fl, int fr, int bl, int br, MotorSupplier<M> t, Inversions i, DriveLayout l) {
			this.front_left = DriveBase.inlineInverter(t.create(fl), i.left);
            this.front_right = DriveBase.inlineInverter(t.create(fr), i.right);
            this.back_left = DriveBase.inlineInverter(t.create(bl), i.left);
            this.back_right = DriveBase.inlineInverter(t.create(br), i.right);
            this.invert = i;
			for(DriveLayout lo : supported) {
				if(l == lo) {
					this.layout = l;
					return;
				}
			}
			this.layout = supported[0];
		}

        public MotorController getLeftGroup() {
            this.front_left.setInverted(false);	// revert inversion in constructor
            this.back_left.setInverted(false);
            return DriveBase.inlineInverter(new MotorControllerGroup(this.front_left, this.back_left), this.invert.left);
        }
        public MotorController getRightGroup() {
            this.front_right.setInverted(false);	// revert inversion in constructor
            this.back_right.setInverted(false);
            return DriveBase.inlineInverter(new MotorControllerGroup(this.front_right, this.back_right), this.invert.right);
        }

        public String toString() {
            return "DB4@" + this.hashCode() + ": {FL:" + this.front_left + " FR:" + this.front_right + 
                " BL:" + this.back_left + " BR:" + this.back_right + "}\n >> " + this.invert.toString();
        }

    }

    public static final class DBS {     // a container for all drivebase settings

        public final boolean default_squaring;
        public final Deceleration s_deceleration;

        public DBS(Deceleration gradient, boolean squaring) {
            this.s_deceleration = gradient;
            this.default_squaring = squaring;
        }

	}

}