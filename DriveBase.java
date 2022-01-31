package frc.robot.modules.common;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

//import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.KilloughDrive;
// make swerve drive if that is ever relevant

//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveBase extends SubsystemBase {
	
	public static MotorController inlineInverter(MotorController m, boolean invert) {
		m.setInverted(invert);
		return m;
	}
	public static void applyDeceleration(Types.Deceleration constant, MotorController m, MotorController... ms) {
		m.set(m.get()*constant.value);
		for(int i = 0; i < ms.length; i++) {
			ms[i].set(ms[i].get()*constant.value);
		}
	}
	public static void applyDeceleration(Types.Deceleration constant, MotorController[] motors) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].set(motors[i].get()*constant.value);
		}
	}
	public static void applyStop(MotorController m, MotorController... ms) {
		m.set(0);
		for(int i = 0; i < ms.length; i++) {
			ms[i].set(0);
		}
	}
	public static void applyStop(MotorController[] motors) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].set(0);
		}
	}

	private static class DifferentialBase implements Types.Drivable {

		private final MotorController[] motors = new MotorController[2];
		private final DifferentialDrive drive;
		private final Types.DBS settings;

		public DifferentialBase(Types.DB2 map, Types.DBS s) {
			this.motors[0] = map.left;
			this.motors[1] = map.right;
			this.drive = new DifferentialDrive(this.motors[0], this.motors[1]);
			this.settings = s;
		}
		public DifferentialBase(Types.DB4 map, Types.DBS s) {
			this.motors[0] = map.getLeftGroup();
			this.motors[1] = map.getRightGroup();
			this.drive = new DifferentialDrive(this.motors[0], this.motors[1]);
			this.settings = s;
		}

		@Override public Types.DriveLayout getLayout() { return Types.DriveLayout.DIFFERENTIAL; }
		@Override public MotorController[] getMotors() { return this.motors; }

		@Override public void tankDrive(double l, double r) { drive.tankDrive(l, r, this.settings.default_squaring); }
		@Override public void arcadeDrive(double s, double rot) { drive.arcadeDrive(s, rot, this.settings.default_squaring); }
		@Override public void raceDrive(double f, double b, double rot) { drive.arcadeDrive(f-b, rot, this.settings.default_squaring); }
		@Override public void curvatureDrive(double s, double rot, boolean q) { drive.curvatureDrive(s, rot, q); }
		@Override public void topDownDrive(double x,  double y, double rot) { System.out.println("DifferentialBase: topDownDrive() is not supported"); }

		@Override public void feed() { this.drive.feed(); }


	}
	private class MecanumBase implements Types.Drivable {

		private final MotorController[] motors = new MotorController[4];
		private final MecanumDrive drive;
		private final Types.DBS settings;

		public MecanumBase(Types.DB4 map, Types.DBS s) {
			this.motors[0] = map.front_left;
			this.motors[1] = map.back_left;
			this.motors[2] = map.front_right;
			this.motors[3] = map.back_right;
			this.drive = new MecanumDrive(this.motors[0], this.motors[1], this.motors[2], this.motors[3]);
			this.settings = s;
		}

		@Override public Types.DriveLayout getLayout() { return Types.DriveLayout.MECANUM; }
		@Override public MotorController[] getMotors() { return this.motors; }

		@Override public void tankDrive(double l, double r) { System.out.println("MecanumBase: tankDrive() is not implemented yet"); }
		@Override public void arcadeDrive(double s, double rot) { this.drive.driveCartesian(s, 0, rot); }
		@Override public void raceDrive(double f, double b, double rot) { this.drive.driveCartesian(f-b, 0, rot); }
		@Override public void curvatureDrive(double s, double rot, boolean q) { System.out.println("MecanumBase: curvatureDrive() is not implemented yet"); }
		@Override public void topDownDrive(double x,  double y, double rot) { this.drive.driveCartesian(y, x, rot); }

		@Override public void feed() { this.drive.feed(); }


	}
	private class KilloughBase implements Types.Drivable {

		private final MotorController[] motors = new MotorController[3];
		private final KilloughDrive drive;
		private final Types.DBS settings;

		public<M extends MotorController> KilloughBase(Types.DB3 map, Types.DBS s) {
			this.motors[0] = map.left;
			this.motors[1] = map.right;
			this.motors[2] = map.mid;
			this.drive = new KilloughDrive(this.motors[0], this.motors[1], this.motors[2]);
			this.settings = s;
		}

		@Override public Types.DriveLayout getLayout() { return Types.DriveLayout.KILLOUGH; }
		@Override public MotorController[] getMotors() { return this.motors; }

		@Override public void tankDrive(double l, double r) { System.out.println("KilloughBase: tankDrive() is not supported"); }
		@Override public void arcadeDrive(double s, double rot) { this.drive.driveCartesian(s, 0, rot); }
		@Override public void raceDrive(double f, double b, double rot) { this.drive.driveCartesian(f-b, 0, rot); }
		@Override public void curvatureDrive(double s, double rot, boolean q) { System.out.println("KilloughBase: curvatureDrive() is not implemented yet"); }
		@Override public void topDownDrive(double x,  double y, double rot) { this.drive.driveCartesian(y, x, rot); }

		@Override public void feed() { this.drive.feed(); }


	}


	private final Types.Drivable drive;
	private final Decelerate decelerate_command;
	private final Idle idle_command;
	private TankDrive tank_command = new TankDrive(this, ()->0.0, ()->0.0);
	private ArcadeDrive arcade_command = new ArcadeDrive(this, ()->0.0, ()->0.0);
	private RaceDrive race_command = new RaceDrive(this, ()->0.0, ()->0.0, ()->0.0);
	private CurvatureDrive curvature_command = new CurvatureDrive(this, ()->0.0, ()->0.0, ()->false);
	private TopDownDrive topdown_command = new TopDownDrive(this, ()->0.0, ()->0.0, ()->0.0);

	public DriveBase(Types.DB2 map, Types.DBS s) {
		this.drive = new DifferentialBase(map, s);
		this.decelerate_command = new Decelerate(this, s.s_deceleration);
		this.idle_command = new Idle(this);
		super.setDefaultCommand(this.idle_command);
	}
	public DriveBase(Types.DB3 map, Types.DBS s) {
		this.drive = new KilloughBase(map, s);
		this.decelerate_command = new Decelerate(this, s.s_deceleration);
		this.idle_command = new Idle(this);
		super.setDefaultCommand(this.idle_command);
	}
	public DriveBase(Types.DB4 map, Types.DBS s) {
		this.decelerate_command = new Decelerate(this, s.s_deceleration);
		this.idle_command = new Idle(this);
		super.setDefaultCommand(this.idle_command);
		switch(map.layout) {
			case MECANUM: 
				this.drive = new MecanumBase(map, s);
				break;
			case DIFFERENTIAL: 
			default:
				this.drive = new DifferentialBase(map, s);
				break;
		}
	}

	// Extend this class to gain access to direct control methods
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

	}

	public Decelerate getDecelerateCommand() { return this.decelerate_command; }

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
	

	public static class TankDrive extends CommandBase {
		private final DriveBase drivebase;
		private final Input.AnalogSupplier left, right;
		public TankDrive(DriveBase db, Input.AnalogSupplier l, Input.AnalogSupplier r) {
			this.drivebase = db;
			this.left = l;
			this.right = r;
			super.addRequirements(db);
		}
		@Override public void initialize() {
			if(!this.drivebase.drive.getLayout().supports(Types.DriveMode.TANK)) {
				System.out.println("TankDrive(Command): Drivebase does not support drive mode!");
				this.cancel();
			}
		}
		@Override public void execute() {
			this.drivebase.drive.tankDrive(this.left.get(), this.right.get());
		}
		@Override public boolean isFinished() {
			return false;
		}
	}
	public static class ArcadeDrive extends CommandBase {
		private final DriveBase drivebase;
		private final Input.AnalogSupplier speed, rotation;
		public ArcadeDrive(DriveBase db, Input.AnalogSupplier s, Input.AnalogSupplier rot) {
			this.drivebase = db;
			this.speed = s;
			this.rotation = rot;
			super.addRequirements(db);
		}
		@Override public void initialize() {
			if(!this.drivebase.drive.getLayout().supports(Types.DriveMode.ARCADE)) {
				System.out.println("ArcadeDrive(Command): Drivebase does not support drive mode!");
				this.cancel();
			}
		}
		@Override public void execute() {
			this.drivebase.drive.arcadeDrive(this.speed.get(), this.rotation.get());
		}
		@Override public boolean isFinished() {
			return false;
		}
	}
	public static class RaceDrive extends CommandBase {
		private final DriveBase drivebase;
		private final Input.AnalogSupplier forward, backward, rotation;
		public RaceDrive(DriveBase db, Input.AnalogSupplier f, Input.AnalogSupplier b, Input.AnalogSupplier rot) {
			this.drivebase = db;
			this.forward = f;
			this.backward = b;
			this.rotation = rot;
			super.addRequirements(db);
		}
		@Override public void initialize() {
			if(!this.drivebase.drive.getLayout().supports(Types.DriveMode.RACE)) {
				System.out.println("RaceDrive(Command): Drivebase does not support drive mode!");
				this.cancel();
			}
		}
		@Override public void execute() {
			this.drivebase.drive.raceDrive(this.forward.get(), this.backward.get(), this.rotation.get());
		}
		@Override public boolean isFinished() {
			return false;
		}
	}
	public static class CurvatureDrive extends CommandBase {
		private final DriveBase drivebase;
		private final Input.AnalogSupplier speed, rotation;
		private final Input.DigitalSupplier qstop;
		public CurvatureDrive(DriveBase db, Input.AnalogSupplier s, Input.AnalogSupplier rot, Input.DigitalSupplier qs) {
			this.drivebase = db;
			this.speed = s;
			this.rotation = rot;
			this.qstop = qs;
			super.addRequirements(db);
		}
		@Override public void initialize() {
			if(!this.drivebase.drive.getLayout().supports(Types.DriveMode.CURVATURE)) {
				System.out.println("CurvatureDrive(Command): Drivebase does not support drive mode!");
				this.cancel();
			}
		}
		@Override public void execute() {
			this.drivebase.drive.curvatureDrive(this.speed.get(), this.rotation.get(), this.qstop.get());
		}
		@Override public boolean isFinished() {
			return false;
		}
	}
	public static class TopDownDrive extends CommandBase {
		private final DriveBase drivebase;
		private final Input.AnalogSupplier xspeed, yspeed, rotation;
		public TopDownDrive(DriveBase db, Input.AnalogSupplier x, Input.AnalogSupplier y, Input.AnalogSupplier rot) {
			this.drivebase = db;
			this.xspeed = x;
			this.yspeed = y;
			this.rotation = rot;
			super.addRequirements(db);
		}
		@Override public void initialize() {
			if(!this.drivebase.drive.getLayout().supports(Types.DriveMode.TOP)) {
				System.out.println("TopDownDrive(Command): Drivebase does not support drive mode!");
				this.cancel();
			}
		}
		@Override public void execute() {
			this.drivebase.drive.topDownDrive(this.xspeed.get(), this.yspeed.get(), this.rotation.get());
		}
		@Override public boolean isFinished() {
			return false;
		}
	}

	public static class Decelerate extends CommandBase {
		private final DriveBase drivebase;
		private final Types.Deceleration constant;
		public Decelerate(DriveBase db, Types.Deceleration c) {
			this.drivebase = db;
			this.constant = c;
			super.addRequirements(db);
		}
		@Override public void initialize() { System.out.println("Decelerating..."); }
		@Override public void execute() { 
			DriveBase.applyDeceleration(this.constant, this.drivebase.drive.getMotors());
			this.drivebase.drive.feed();
		}
		@Override public void end(boolean i) {
			DriveBase.applyStop(this.drivebase.drive.getMotors());
			this.drivebase.drive.feed();
		}
		@Override public boolean isFinished() {
			for(int i = 0; i < this.drivebase.drive.getMotors().length; i++) {
				if(Math.abs(this.drivebase.drive.getMotors()[i].get()) > 0.1) { return false; }
			}
			return true;
		}

	}
	private class Idle extends CommandBase {
		private final DriveBase drivebase;
		public Idle(DriveBase db) {
			this.drivebase = db;
			super.addRequirements(db);
		}
		//@Override public void initialize() { applyStop(this.drivebase.drive.getMotors()); }
		@Override public void execute() { 
			applyStop(this.drivebase.drive.getMotors());
			this.drivebase.drive.feed(); }
		@Override public boolean isFinished() { return false; }

	}


}