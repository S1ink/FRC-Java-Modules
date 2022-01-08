package frc.robot.modules;

public class Types {

	public static enum DriveLayout {
        DIFFERENTIAL,
        MECANUM,
        KILLOUGH/*,
        SWERVE*/;
    }

    public static enum Inversions {
        NEITHER (false, false),
        LEFT    (true, false),
        BOTH    (true, true),
        RIGHT   (false, true);

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
        TRIGGER		(3),
        CURVATURE	(4);

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
            for(int i = 0; i < this.modes.length; i++) {
                if(this.modes[i] == m) {
                    this.index = i;
                }
            }
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
        private Deceleration(double v) {
            this.value = v;
        }

        public String toString() { return "Deceleration@" + this.hashCode() + ": " + this.value; }
    }

    public static final class DB2 {     // a drivebase map containing two sides, each with one motor port
        public final int 
            left, 
            right;
        public final Inversions 
            invert;
        
        public DB2(int l, int r) {
            this.left = l;
            this.right = r;
            this.invert = Inversions.NEITHER;
        }
        public DB2(int l, int r, Inversions i) {
            this.left = l;
            this.right = r;
            this.invert = i;
        }

        public String toString() {
            return "DB2@" + this.hashCode() + ": {Left port:" + this.left + " Right port:" + this.right + "}\n >> " + this.invert.toString();
        }
    }
    public static final class DB4 {     // a drivebase map containing two sides, each with two motor ports
        public final int 
            front_left,
            front_right,
            back_left,
            back_right;
        public final Inversions
            invert;

        public DB4(int fl, int fr, int bl, int br) {
            this.front_left = fl;
            this.front_right = fr;
            this.back_left = bl;
            this.back_right = br;
            this.invert = Inversions.NEITHER;
        }
        public DB4(int fl, int fr, int bl, int br, Inversions i) {
            this.front_left = fl;
            this.front_right = fr;
            this.back_left = bl;
            this.back_right = br;
            this.invert = i;
        }

        public String toString() {
            return "DB4@" + this.hashCode() + ": {FL:" + this.front_left + " FR:" + this.front_right + 
                " BL:" + this.back_left + " BR:" + this.back_right + "}\n >> " + this.invert.toString();
        }
    }

    public static final class DBS {     // a container for all drivebase settings
        public final DriveMode default_mode;
        public final DriveMode[] mode_options;
        public final boolean default_squaring;
        public final Deceleration s_deceleration;

        public DBS(DriveMode dmode, Deceleration gradient, boolean squaring) {
            this.default_mode = dmode;
            this.s_deceleration = gradient;
            this.default_squaring = squaring;
            this.mode_options = null;
        }

        public DBS(DriveMode dmode, Deceleration gradient, boolean squaring, DriveMode[] options) {
            this.default_mode = dmode;
            this.mode_options = options;
            this.s_deceleration = gradient;
            this.default_squaring = squaring;
        }
    }
}
