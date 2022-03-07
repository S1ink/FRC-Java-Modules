package frc.robot.modules.common.drive;

//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.drive.*;
// make swerve drive if that is ever relevant
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.modules.common.Input;
import frc.robot.modules.common.Input.*;
import frc.robot.modules.common.drive.Types.*;


public class DriveBase extends SubsystemBase {

	private static class DifferentialBase implements Drivable {

		private final MotorController[] motors = new MotorController[2];
		private final DifferentialDrive drive;
		private boolean squaring = false;

		public<M extends MotorController> DifferentialBase(DriveMap_2<M> map) {
			this.motors[0] = map.left;
			this.motors[1] = map.right;
			this.drive = new DifferentialDrive(this.motors[0], this.motors[1]);
		}
		public <M extends MotorController>DifferentialBase(DriveMap_4<M> map) {
			this.motors[0] = map.getLeftGroup();
			this.motors[1] = map.getRightGroup();
			this.drive = new DifferentialDrive(this.motors[0], this.motors[1]);
		}

		@Override public DriveLayout getLayout() { return DriveLayout.DIFFERENTIAL; }
		@Override public MotorController[] getMotors() { return this.motors; }

		@Override public void tankDrive(double l, double r) { drive.tankDrive(l, r, this.squaring); }
		@Override public void arcadeDrive(double s, double rot) { drive.arcadeDrive(s, rot, this.squaring); }
		@Override public void raceDrive(double f, double b, double rot) { drive.arcadeDrive(f-b, rot, this.squaring); }
		@Override public void curvatureDrive(double s, double rot, boolean q) { drive.curvatureDrive(s, rot, q); }
		@Override public void topDownDrive(double x,  double y, double rot) { System.out.println("DifferentialBase: topDownDrive() is not supported"); }

		@Override public void autoTurn(double v) {
			this.motors[0].set(v);
			this.motors[1].set(-v);
			this.drive.feed();
		}
		@Override public void autoDrive(double l, double r) {
			this.motors[0].set(l);
			this.motors[1].set(r);
			this.drive.feed();
		}
		@Override public void autoDriveVoltage(double lv, double rv) {
			this.motors[0].setVoltage(lv);
			this.motors[1].setVoltage(rv);
			this.drive.feed();
		}

		@Override public void feed() { this.drive.feed(); }
		@Override public void setScaling(double s) { this.drive.setMaxOutput(s); }
		@Override public void setDeadband(double d) { this.drive.setDeadband(d); }
		@Override public void setSquaring(boolean s) { this.squaring = s; }


	}
	private class MecanumBase implements Drivable {

		private final MotorController[] motors = new MotorController[4];
		private final MecanumDrive drive;

		public<M extends MotorController> MecanumBase(DriveMap_4<M> map) {
			this.motors[0] = map.front_left;
			this.motors[1] = map.back_left;
			this.motors[2] = map.front_right;
			this.motors[3] = map.back_right;
			this.drive = new MecanumDrive(this.motors[0], this.motors[1], this.motors[2], this.motors[3]);
		}

		@Override public DriveLayout getLayout() { return DriveLayout.MECANUM; }
		@Override public MotorController[] getMotors() { return this.motors; }

		@Override public void tankDrive(double l, double r) { System.out.println("MecanumBase: tankDrive() is not implemented yet"); }
		@Override public void arcadeDrive(double s, double rot) { this.drive.driveCartesian(s, 0, rot); }
		@Override public void raceDrive(double f, double b, double rot) { this.drive.driveCartesian(f-b, 0, rot); }
		@Override public void curvatureDrive(double s, double rot, boolean q) { System.out.println("MecanumBase: curvatureDrive() is not implemented yet"); }
		@Override public void topDownDrive(double x,  double y, double rot) { this.drive.driveCartesian(y, x, rot); }

		@Override public void autoTurn(double v) {
			this.motors[0].set(v);
			this.motors[1].set(v);
			this.motors[2].set(-v);
			this.motors[3].set(-v);
			this.drive.feed();
		}
		@Override public void autoDrive(double l, double r) {
			this.motors[0].set(l);
			this.motors[1].set(l);
			this.motors[2].set(r);
			this.motors[3].set(r);
			this.drive.feed();
		}
		@Override public void autoDriveVoltage(double lv, double rv) {
			this.motors[0].setVoltage(lv);
			this.motors[1].setVoltage(lv);
			this.motors[2].setVoltage(rv);
			this.motors[3].setVoltage(rv);
			this.drive.feed();
		}

		@Override public void feed() { this.drive.feed(); }
		@Override public void setScaling(double s) { this.drive.setMaxOutput(s); }
		@Override public void setDeadband(double d) { this.drive.setDeadband(d); }


	}
	private class KilloughBase implements Drivable {

		private final MotorController[] motors = new MotorController[3];
		private final KilloughDrive drive;

		public<M extends MotorController> KilloughBase(DriveMap_3<M> map) {
			this.motors[0] = map.left;
			this.motors[1] = map.right;
			this.motors[2] = map.mid;
			this.drive = new KilloughDrive(this.motors[0], this.motors[1], this.motors[2]);
		}

		@Override public DriveLayout getLayout() { return DriveLayout.KILLOUGH; }
		@Override public MotorController[] getMotors() { return this.motors; }

		@Override public void tankDrive(double l, double r) { System.out.println("KilloughBase: tankDrive() is not supported"); }
		@Override public void arcadeDrive(double s, double rot) { this.drive.driveCartesian(s, 0, rot); }
		@Override public void raceDrive(double f, double b, double rot) { this.drive.driveCartesian(f-b, 0, rot); }
		@Override public void curvatureDrive(double s, double rot, boolean q) { System.out.println("KilloughBase: curvatureDrive() is not implemented yet"); }
		@Override public void topDownDrive(double x,  double y, double rot) { this.drive.driveCartesian(y, x, rot); }

		@Override public void autoTurn(double v) {
			Helper.applyAll(this.motors, v);
			this.drive.feed();
		}
		@Override public void autoDrive(double l, double r) {

			this.drive.feed();
		}
		@Override public void autoDriveVoltage(double lv, double rv) {

			this.drive.feed();
		}

		@Override public void feed() { this.drive.feed(); }
		@Override public void setScaling(double s) { this.drive.setMaxOutput(s); }
		@Override public void setDeadband(double d) { this.drive.setDeadband(d); }


	}


	private final Drivable drive;
	private final Idle idle_command;
	private Decelerate decelerate_command;
	private TankDrive tank_command = new TankDrive(this, ()->0.0, ()->0.0);
	private ArcadeDrive arcade_command = new ArcadeDrive(this, ()->0.0, ()->0.0);
	private RaceDrive race_command = new RaceDrive(this, ()->0.0, ()->0.0, ()->0.0);
	private CurvatureDrive curvature_command = new CurvatureDrive(this, ()->0.0, ()->0.0, ()->false);
	private TopDownDrive topdown_command = new TopDownDrive(this, ()->0.0, ()->0.0, ()->0.0);
	private ModeDrive modedrive_command;

	public<M extends MotorController> DriveBase(DriveMap_2<M> map) {
		this.drive = new DifferentialBase(map);
		this.idle_command = new Idle(this);
		this.modedrive_command = new ModeDrive(this, ()->0.0, ()->0.0, ()->false, ()->false);
		super.setDefaultCommand(this.idle_command);
	}
	public<M extends MotorController> DriveBase(DriveMap_3<M> map) {
		this.drive = new KilloughBase(map);
		this.idle_command = new Idle(this);
		this.modedrive_command = new ModeDrive(this, ()->0.0, ()->0.0, ()->false, ()->false);
		super.setDefaultCommand(this.idle_command);
	}
	public<M extends MotorController> DriveBase(DriveMap_4<M> map) {
		this.idle_command = new Idle(this);
		super.setDefaultCommand(this.idle_command);
		switch(map.layout) {
			case MECANUM: 
				this.drive = new MecanumBase(map);
				break;
			case DIFFERENTIAL: 
			default:
				this.drive = new DifferentialBase(map);
				break;
		}
		this.modedrive_command = new ModeDrive(this, ()->0.0, ()->0.0, ()->false, ()->false);
	}

	public void setSpeedScaling(double s) {
		this.drive.setScaling(s);
	}
	public void setSpeedDeadband(double d) {
		this.drive.setDeadband(d);
	}
	public void setSpeedSquaring(boolean s) {
		this.drive.setSquaring(s);
	}

	protected Drivable getDrive() {
		return this.drive;
	}

	/**Extend this class to gain access to direct control methods*/
	public static abstract class DriveCommandBase extends CommandBase {

		protected final DriveBase drivebase;

		protected DriveCommandBase(DriveBase db) {
			this.drivebase = db;
			super.addRequirements(db);
		}

		protected void tankDrive(double l, double r) { this.drivebase.drive.tankDrive(l, r); }
		protected void arcadeDrive(double s, double rot) { this.drivebase.drive.arcadeDrive(s, rot); }
		protected void raceDrive(double f, double b, double rot) { this.drivebase.drive.raceDrive(f, b, rot); }
		protected void curvatureDrive(double s, double rot, boolean q) { this.drivebase.drive.curvatureDrive(s, rot, q); }
		protected void topDownDrive(double x,  double y, double rot) { this.drivebase.drive.topDownDrive(x, y, rot); }

		protected void autoTurn(double v) { this.drivebase.drive.autoTurn(v); }
		protected void autoDrive(double l, double r) { this.drivebase.drive.autoDrive(l, r); }
		protected void autoDriveVoltage(double lv, double rv) { this.drivebase.drive.autoDriveVoltage(lv, rv); }

		protected void fromLast(double p) { 
			Helper.applyPercentage(this.drivebase.drive.getMotors(), p); 
			this.drivebase.drive.feed();
		}

		protected Drivable getDrive() {
			return this.drivebase.drive;
		}


	}
	public static abstract class RateLimitedAutoDrive extends DriveCommandBase {

		protected final SlewRateLimiter t_limit, d_limit;

		protected RateLimitedAutoDrive(DriveBase db) {
			super(db);
			this.t_limit = new SlewRateLimiter(Double.MAX_VALUE);
			this.d_limit = new SlewRateLimiter(Double.MAX_VALUE);
		}
		protected RateLimitedAutoDrive(DriveBase db, double rlimit) {
			super(db);
			this.t_limit = new SlewRateLimiter(rlimit);
			this.d_limit = new SlewRateLimiter(rlimit);
		}

		@Override protected void autoTurn(double v) {
			super.drivebase.drive.autoTurn(this.t_limit.calculate(v));
		}
		@Override protected void autoDrive(double l, double r) {
			double p = this.d_limit.calculate(Math.max(Math.abs(l), Math.abs(r)));
			p /= Math.max(Math.abs(l), Math.abs(r));		// find proportion between input and output or max input
			l *= p;		// scale each input by ^ so that the ratio remains constant between left and right
			r *= p;		// ^^
			super.drivebase.drive.autoDrive(l, r);
		}


	}

	public TankDrive tankDrive() { return this.tank_command; }
	public TankDrive tankDrive(Input.AnalogSupplier l, Input.AnalogSupplier r) { 
		this.tank_command = new TankDrive(this, l, r);
		return this.tank_command; 
	}
	public ArcadeDrive arcadeDrive() { return this.arcade_command; }
	public ArcadeDrive arcadeDrive(Input.AnalogSupplier s, Input.AnalogSupplier rot) { 
		this.arcade_command =  new ArcadeDrive(this, s, rot); 
		return this.arcade_command;
	}
	public RaceDrive raceDrive() { return this.race_command; }
	public RaceDrive raceDrive(Input.AnalogSupplier f, Input.AnalogSupplier b, Input.AnalogSupplier rot) { 
		this.race_command = new RaceDrive(this, f, b, rot); 
		return this.race_command;
	}
	public CurvatureDrive curvatureDrive() { return this.curvature_command; }
	public CurvatureDrive curvatureDrive(Input.AnalogSupplier s, Input.AnalogSupplier rot, Input.DigitalSupplier qs) { 
		this.curvature_command = new CurvatureDrive(this, s, rot, qs); 
		return this.curvature_command;
	}
	public TopDownDrive topDownDrive() { return this.topdown_command; }
	public TopDownDrive topDownDrive(Input.AnalogSupplier x, Input.AnalogSupplier y, Input.AnalogSupplier rot) { 
		this.topdown_command = new TopDownDrive(this, x, y, rot); 
		return this.topdown_command;
	}
	public ModeDrive modeDrive() { return this.modedrive_command; } 
	public ModeDrive modeDrive(AnalogSupplier x, AnalogSupplier y, DigitalSupplier inc, DigitalSupplier dec) {
		this.modedrive_command = new ModeDrive(this, x, y, inc, dec);
		return this.modedrive_command;
	}
	public ModeDrive modeDrive(
		AnalogSupplier lx, AnalogSupplier ly,
		AnalogSupplier rx, AnalogSupplier ry,
		DigitalSupplier inc, DigitalSupplier dec
	) {
		this.modedrive_command = new ModeDrive(this, lx, ly, rx, ry, inc, dec);
		return this.modedrive_command;
	}
	public ModeDrive modeDrive(
		AnalogSupplier lx, AnalogSupplier ly, AnalogSupplier lt,
		AnalogSupplier rx, AnalogSupplier ry, AnalogSupplier rt,
		DigitalSupplier inc, DigitalSupplier dec
	) {
		this.modedrive_command = new ModeDrive(this, lx, ly, lt, rx, ry, rt, inc, dec);
		return this.modedrive_command;
	}
	public Decelerate decelerate() { return this.decelerate_command; }
	public Decelerate decelerate(Deceleration a) {
		this.decelerate_command = new Decelerate(this, a);
		return this.decelerate_command;
	}
	

	public static class TankDrive extends DriveCommandBase {
		
		private final Input.AnalogSupplier left, right;

		public TankDrive(DriveBase db, Input.AnalogSupplier l, Input.AnalogSupplier r) {
			super(db);
			this.left = l;
			this.right = r;
		}

		@Override public void initialize() {
			if(!this.drivebase.drive.getLayout().supports(DriveMode.TANK)) {
				System.out.println("TankDrive(Command): Drivebase does not support drive mode!");
				this.cancel();
			}
		}
		@Override public void execute() {
			this.drivebase.drive.tankDrive(this.left.get() * -1, this.right.get() * -1);
		}
		@Override public boolean isFinished() { return false; }


	}
	public static class ArcadeDrive extends DriveCommandBase {
		
		private final Input.AnalogSupplier speed, rotation;

		public ArcadeDrive(DriveBase db, Input.AnalogSupplier s, Input.AnalogSupplier rot) {
			super(db);
			this.speed = s;
			this.rotation = rot;
		}

		@Override public void initialize() {
			if(!this.drivebase.drive.getLayout().supports(DriveMode.ARCADE)) {
				System.out.println("ArcadeDrive(Command): Drivebase does not support drive mode!");
				this.cancel();
			}
		}
		@Override public void execute() {
			this.drivebase.drive.arcadeDrive(this.speed.get(), this.rotation.get());
		}
		@Override public boolean isFinished() { return false; }


	}
	public static class RaceDrive extends DriveCommandBase {
		
		private final Input.AnalogSupplier forward, backward, rotation;

		public RaceDrive(DriveBase db, Input.AnalogSupplier f, Input.AnalogSupplier b, Input.AnalogSupplier rot) {
			super(db);
			this.forward = f;
			this.backward = b;
			this.rotation = rot;
		}

		@Override public void initialize() {
			if(!this.drivebase.drive.getLayout().supports(DriveMode.RACE)) {
				System.out.println("RaceDrive(Command): Drivebase does not support drive mode!");
				this.cancel();
			}
		}
		@Override public void execute() {
			this.drivebase.drive.raceDrive(this.forward.get(), this.backward.get(), this.rotation.get());
		}
		@Override public boolean isFinished() { return false; }


	}
	public static class CurvatureDrive extends DriveCommandBase {
		
		private final Input.AnalogSupplier speed, rotation;
		private final Input.DigitalSupplier qstop;

		public CurvatureDrive(DriveBase db, Input.AnalogSupplier s, Input.AnalogSupplier rot, Input.DigitalSupplier qs) {
			super(db);
			this.speed = s;
			this.rotation = rot;
			this.qstop = qs;
		}

		@Override public void initialize() {
			if(!this.drivebase.drive.getLayout().supports(DriveMode.CURVATURE)) {
				System.out.println("CurvatureDrive(Command): Drivebase does not support drive mode!");
				this.cancel();
			}
		}
		@Override public void execute() {
			this.drivebase.drive.curvatureDrive(this.speed.get(), this.rotation.get(), this.qstop.get());
		}
		@Override public boolean isFinished() { return false; }


	}
	public static class TopDownDrive extends DriveCommandBase {
		
		private final Input.AnalogSupplier xspeed, yspeed, rotation;

		public TopDownDrive(DriveBase db, Input.AnalogSupplier x, Input.AnalogSupplier y, Input.AnalogSupplier rot) {
			super(db);
			this.xspeed = x;
			this.yspeed = y;
			this.rotation = rot;
		}

		@Override public void initialize() {
			if(!this.drivebase.drive.getLayout().supports(DriveMode.TOP)) {
				System.out.println("TopDownDrive(Command): Drivebase does not support drive mode!");
				this.cancel();
			}
		}
		@Override public void execute() {
			this.drivebase.drive.topDownDrive(this.xspeed.get(), this.yspeed.get(), this.rotation.get());
		}
		@Override public boolean isFinished() { return false; }


	}

	public static class ModeDrive extends DriveCommandBase {

		private final AnalogSupplier 
			left_x,
			left_y,
			left_t,
			right_x,
			right_y,
			right_t;
		public final DigitalSupplier
			increment,
			decrement;
		private final DriveModes mode;

		public ModeDrive(	// single stick
			DriveBase db,
			AnalogSupplier x, AnalogSupplier y,
			DigitalSupplier inc, DigitalSupplier dec
		) {
			super(db);
			this.left_x = ()->0.0;
			this.left_y = ()->0.0;
			this.left_t = ()->0.0;
			this.right_x = x;
			this.right_y = y;
			this.right_t = ()->0.0;
			this.increment = inc;
			this.decrement = dec;
			this.mode = new DriveModes(
				DriveModes.filter(	// filter options so something that isn't supported never gets called
					super.getDrive().getLayout().supported, 
					new DriveMode[]{DriveMode.ARCADE, DriveMode.CURVATURE}
				)
			);
		}
		public ModeDrive(	// dual stick
			DriveBase db,
			AnalogSupplier lx, AnalogSupplier ly, AnalogSupplier rx, AnalogSupplier ry,
			DigitalSupplier inc, DigitalSupplier dec
		) {
			super(db);
			this.left_x = lx;
			this.left_y = ly;
			this.left_t = ()->0.0;
			this.right_x = rx;
			this.right_y = ry;
			this.right_t = ()->0.0;
			this.increment = inc;
			this.decrement = dec;
			this.mode = new DriveModes(
				DriveModes.filter(	// filter options so something that isn't supported never gets called
					super.getDrive().getLayout().supported, 
					new DriveMode[]{DriveMode.TANK, DriveMode.ARCADE, DriveMode.CURVATURE, DriveMode.TOP}
				)
			);
		}
		public ModeDrive(	// 2 sticks, 2 triggers
			DriveBase db,
			AnalogSupplier lx, AnalogSupplier ly, AnalogSupplier lt,
			AnalogSupplier rx, AnalogSupplier ry, AnalogSupplier rt,
			DigitalSupplier inc, DigitalSupplier dec
		) {
			super(db);
			this.left_x = lx;
			this.left_y = ly;
			this.left_t = lt;
			this.right_x = rx;
			this.right_y = ry;
			this.right_t = rt;
			this.increment = inc;
			this.decrement = dec;
			this.mode = new DriveModes(
				super.getDrive().getLayout().supported
			);
		}

		@Override public void initialize() {
			System.out.println("ModeDrive: Running...");
		}
		@Override public void execute() {
			if(this.increment.get()) {
				this.mode.increment();
				System.out.println("Mode incremented: " + this.mode.get().name());
			}
			if(this.decrement.get()) {
				this.mode.decrement();
				System.out.println("Mode decremented: " + this.mode.get().name());
			}
			switch(this.mode.get()) {
				case TANK:
					super.tankDrive(this.left_y.get(), this.right_y.get());
					break;
				case ARCADE:
					super.arcadeDrive(this.right_y.get(), this.right_x.get()*-1);	// for some reason the steering is inverted
					break;
				case RACE:
					super.raceDrive(this.right_t.get(), this.left_t.get(), this.right_x.get()*-1);
					break;
				case CURVATURE:
					super.curvatureDrive(this.right_y.get(), this.right_x.get()*-1, this.right_t.get() >= 0.5);
					break;
				case TOP:
					super.topDownDrive(this.left_x.get(), this.left_y.get(), this.right_x.get());
					break;
				default:
					super.arcadeDrive(this.right_y.get(), this.right_x.get());	// default because it should always be supported (single stick prereq)
			}
		}
		@Override public boolean isFinished() { return false; }


	}

	private static class Decelerate extends DriveCommandBase {
		
		private final Deceleration constant;

		public Decelerate(DriveBase db, Deceleration c) {
			super(db);
			this.constant = c;
		}

		@Override public void initialize() { System.out.println("Decelerating..."); }
		@Override public void execute() { 
			Helper.applyDeceleration(this.constant, this.drivebase.drive.getMotors());
			this.drivebase.drive.feed();
		}
		@Override public void end(boolean i) {
			Helper.applyStop(this.drivebase.drive.getMotors());
			this.drivebase.drive.feed();
		}
		@Override public boolean isFinished() {
			for(int i = 0; i < this.drivebase.drive.getMotors().length; i++) {
				if(Math.abs(this.drivebase.drive.getMotors()[i].get()) > 0.1) { return false; }
			}
			return true;
		}


	}
	private static class Idle extends DriveCommandBase {
		
		public Idle(DriveBase db) {
			super(db);
		}

		@Override public void initialize() { System.out.println("DriveBase Idling..."); }
		@Override public void execute() { 
			Helper.applyStop(this.drivebase.drive.getMotors());
			this.drivebase.drive.feed(); 
		}
		@Override public void end(boolean i) { System.out.println("Idling Stopped."); }
		@Override public boolean isFinished() { return false; }


	}


}