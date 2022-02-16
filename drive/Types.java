package frc.robot.modules.common.drive;

//import edu.wpi.first.wpilibj.drive.RobotDriveBase;
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
	// public static interface Drivable<D extends RobotDriveBase> {

    //     public DriveLayout getLayout();
	// 	public MotorController[] getMotors();
	// 	public D getDriveBase();

	// 	public void tankDrive(double l, double r);
	// 	public void arcadeDrive(double s, double rot);
	// 	public void raceDrive(double f, double b, double rot);
	// 	public void curvatureDrive(double s, double rot, boolean q);
	// 	public void topDownDrive(double x,  double y, double rot);

    //     public void autoTurn(double v); // turn by supplying "speed" -> positive for one direction, negative for the other
    //     public void autoDrive(double l, double r);

	// 	public void feed();
    
	// }

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

    public static class DrivePortMap_2 {

		//public static final DriveLayout[] supported = {DriveLayout.DIFFERENTIAL};
        public final DriveLayout layout = DriveLayout.DIFFERENTIAL;
        public final int 
            p_left,
            p_right;
        public final Inversions
            invert;
        
        public DrivePortMap_2(int l, int r) {
			this.p_left = l;
			this.p_right = r;
			this.invert = Inversions.NEITHER;
		}
		public DrivePortMap_2(int l, int r, Inversions i) {
			this.p_left = l;
			this.p_right = r;
			this.invert = i;
		}
    }
    public static final class DriveMap_2<M extends MotorController> extends DrivePortMap_2 {     // a drivebase map containing two sides, each with one motor port

		public final M 
            left, 
            right;
        
        public DriveMap_2(int l, int r, MotorSupplier<M> t) {
			super(l, r);
            this.left = t.create(l);
            this.right = t.create(r);
        }
        public DriveMap_2(int l, int r, MotorSupplier<M> t, Inversions i) {
			super(l, r, i);
            this.left = Helper.getInverted(t.create(l), i.left);
            this.right = Helper.getInverted(t.create(r), i.right);
        }

        public String toString() {
            return "DB2@" + this.hashCode() + ": {Left port:" + this.left + " Right port:" + this.right + "}\n >> " + this.invert.toString();
        }
		
    }

	public static class DrivePortMap_3 {

		//public static final DriveLayout[] supported = {DriveLayout.KILLOUGH};
		public final DriveLayout layout = DriveLayout.KILLOUGH;
		public final int
            p_left,
            p_mid,
            p_right;

		public DrivePortMap_3(int l, int m, int r) {
			this.p_left = l;
			this.p_mid = m;
			this.p_right = r;
		}
	}
    public static final class DriveMap_3<M extends MotorController> extends DrivePortMap_3 {

        public final M
			left,
			mid,
			right;
		
		public DriveMap_3(int l, int m, int r, MotorSupplier<M> t) {
			super(l, m, r);
			this.left = t.create(l);
			this.mid = t.create(m);
			this.right = t.create(r);
		}

    }

    public static class DrivePortMap_4 {

        public static final DriveLayout[] supported = {DriveLayout.DIFFERENTIAL, DriveLayout.MECANUM};
        public final DriveLayout layout;
        public final int
            p_front_left,
            p_front_right,
            p_back_left,
            p_back_right;
        public final Inversions
            invert;

		public DrivePortMap_4(int fl, int fr, int bl, int br) {
			this(fl, fr, bl, br, Inversions.NEITHER, supported[0]);
		}
		public DrivePortMap_4(int fl, int fr, int bl, int br, Inversions i) {
			this(fl, fr, bl, br, i, supported[0]);
		}
		public DrivePortMap_4(int fl, int fr, int bl, int br, DriveLayout l) {
			this(fl, fr, bl, br, Inversions.NEITHER, l);
		}
        public DrivePortMap_4(int fl, int fr, int bl, int br, Inversions i, DriveLayout l) {
            this.p_front_left = fl;
            this.p_front_right = fr;
            this.p_back_left = bl;
            this.p_back_right = br;
            this.invert = i;
            for(DriveLayout lo : supported) {
				if(l == lo) {
					this.layout = l;
					return;
				}
			}
			this.layout = supported[0];
        }

    }
    public static final class DriveMap_4<M extends MotorController> extends DrivePortMap_4 {     // a drivebase map containing two sides, each with two motor ports
        
		public final M 
            front_left,
            front_right,
            back_left,
            back_right;

        public DriveMap_4(int fl, int fr, int bl, int br, MotorSupplier<M> t) {
			this(fl, fr, bl, br, t, Inversions.NEITHER, supported[0]);
        }
        public DriveMap_4(int fl, int fr, int bl, int br, MotorSupplier<M> t, Inversions i) {
			this(fl, fr, bl, br, t, i, supported[0]);
        }
		public DriveMap_4(int fl, int fr, int bl, int br, MotorSupplier<M> t, DriveLayout l) {
			this(fl, fr, bl, br, t, Inversions.NEITHER, l);
		}
		public DriveMap_4(int fl, int fr, int bl, int br, MotorSupplier<M> t, Inversions i, DriveLayout l) {
			super(fl, fr, bl, br, i, l);
			this.front_left = Helper.getInverted(t.create(fl), i.left);
            this.front_right = Helper.getInverted(t.create(fr), i.right);
            this.back_left = Helper.getInverted(t.create(bl), i.left);
            this.back_right = Helper.getInverted(t.create(br), i.right);
		}

        public MotorController getLeftGroup() {
            this.front_left.setInverted(false);	// revert inversion in constructor
            this.back_left.setInverted(false);
            return Helper.getInverted(new MotorControllerGroup(this.front_left, this.back_left), this.invert.left);
        }
        public MotorController getRightGroup() {
            this.front_right.setInverted(false);	// revert inversion in constructor
            this.back_right.setInverted(false);
            return Helper.getInverted(new MotorControllerGroup(this.front_right, this.back_right), this.invert.right);
        }

        public String toString() {
            return "DB4@" + this.hashCode() + ": {FL:" + this.front_left + " FR:" + this.front_right + 
                " BL:" + this.back_left + " BR:" + this.back_right + "}\n >> " + this.invert.toString();
        }

    }


}