package frc.robot.modules;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.KilloughDrive;
// make swerve drive if that is ever relevant

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
	
	private static MotorController inlineInverter(MotorController m, boolean invert) {
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

		public<M extends MotorController> DifferentialBase(Types.DB2<M> map, Types.DBS s) {
			this.motors[0] = inlineInverter(map.type.create(map.left),  map.invert.left);
			this.motors[1] = inlineInverter(map.type.create(map.right), map.invert.right);
			this.drive = new DifferentialDrive(this.motors[0], this.motors[1]);
			this.settings = s;
		}
		public<M extends MotorController> DifferentialBase(Types.DB4<M> map, Types.DBS s) {
			this.motors[0] = inlineInverter(new MotorControllerGroup( map.type.create(map.front_left), map.type.create(map.back_left)), map.invert.left);
			this.motors[1] = inlineInverter(new MotorControllerGroup( map.type.create(map.front_right), map.type.create(map.back_right)), map.invert.right);
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

		public<M extends MotorController> MecanumBase(Types.DB4<M> map, Types.DBS s) {
			this.motors[0] = inlineInverter(map.type.create(map.front_left), map.invert.left);
			this.motors[1] = inlineInverter(map.type.create(map.back_left), map.invert.left);
			this.motors[2] = inlineInverter(map.type.create(map.front_right), map.invert.right);
			this.motors[3] = inlineInverter(map.type.create(map.back_right), map.invert.right);
			this.drive = new MecanumDrive(this.motors[0], this.motors[1], this.motors[2], this.motors[3]);
			this.settings = s;
		}

		@Override public Types.DriveLayout getLayout() { return Types.DriveLayout.MECANUM; }
		@Override public MotorController[] getMotors() { return this.motors; }

		@Override public void tankDrive(double l, double r) {}
		@Override public void arcadeDrive(double s, double rot) { this.drive.driveCartesian(s, 0, rot); }
		@Override public void raceDrive(double f, double b, double rot) { this.drive.driveCartesian(f-b, 0, rot); }
		@Override public void curvatureDrive(double s, double rot, boolean q) {}
		@Override public void topDownDrive(double x,  double y, double rot) { this.drive.driveCartesian(y, x, rot); }

		@Override public void feed() { this.drive.feed(); }


	}
	private class KilloughBase implements Types.Drivable {

		private final MotorController[] motors = new MotorController[3];
		private final KilloughDrive drive;
		private final Types.DBS settings;

		public<M extends MotorController> KilloughBase(Types.DB3<M> map, Types.DBS s) {
			this.motors[0] = map.type.create(map.left);
			this.motors[1] = map.type.create(map.right);
			this.motors[2] = map.type.create(map.mid);
			this.drive = new KilloughDrive(this.motors[0], this.motors[1], this.motors[2]);
			this.settings = s;
		}

		@Override public Types.DriveLayout getLayout() { return Types.DriveLayout.KILLOUGH; }
		@Override public MotorController[] getMotors() { return this.motors; }

		@Override public void tankDrive(double l, double r) { System.out.println("KilloughBase: tankDrive() is not supported"); }
		@Override public void arcadeDrive(double s, double rot) { this.drive.driveCartesian(s, 0, rot); }
		@Override public void raceDrive(double f, double b, double rot) { this.drive.driveCartesian(f-b, 0, rot); }
		@Override public void curvatureDrive(double s, double rot, boolean q) {}
		@Override public void topDownDrive(double x,  double y, double rot) { this.drive.driveCartesian(y, x, rot); }

		@Override public void feed() { this.drive.feed(); }


	}


	private final Types.Drivable drive;
	private final Decelerate decelerate_command;

	public<M extends MotorController> DriveBase(Types.DB2<M> map, Types.DBS s) {
		this.drive = new DifferentialBase(map, s);
		this.decelerate_command = new Decelerate(this, s.s_deceleration);
		//this.setDefaultCommand(new Idle(this));
	}
	public<M extends MotorController> DriveBase(Types.DB3<M> map, Types.DBS s) {
		this.drive = new KilloughBase(map, s);
		this.decelerate_command = new Decelerate(this, s.s_deceleration);
		//this.setDefaultCommand(new Idle(this));
	}
	public<M extends MotorController> DriveBase(Types.DB4<M> map, Types.DBS s) {
		this.decelerate_command = new Decelerate(this, s.s_deceleration);
		//this.setDefaultCommand(new Idle(this));
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

	public void tankDrive(double l, double r) { this.drive.tankDrive(l, r); }
	public void arcadeDrive(double s, double rot) { this.drive.arcadeDrive(s, rot); }
	public void raceDrive(double f, double b, double rot) { this.drive.raceDrive(f, b, rot); }
	public void curvatureDrive(double s, double rot, boolean q) { this.drive.curvatureDrive(s, rot, q); }
	public void topDownDrive(double x,  double y, double rot) { this.drive.topDownDrive(x, y, rot); }

	public Decelerate getDecelerateCommand() { return this.decelerate_command; }
	

	public static class Decelerate extends CommandBase {
		private final DriveBase drivebase;
		private final Types.Deceleration constant;
		public Decelerate(DriveBase db, Types.Deceleration c) {
			this.drivebase = db;
			this.constant = c;
			addRequirements(db);
		}
		@Override public void execute() { 
			DriveBase.applyDeceleration(this.constant, this.drivebase.drive.getMotors());
			this.drivebase.drive.feed();
		}
		@Override public void end(boolean i) {
			DriveBase.applyStop(this.drivebase.drive.getMotors());
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
			addRequirements(db);
		}
		@Override public void initialize() { DriveBase.applyStop(this.drivebase.drive.getMotors()); }
		@Override public void execute() { this.drivebase.drive.feed(); }
		@Override public boolean isFinished() { return false; }

	}


}