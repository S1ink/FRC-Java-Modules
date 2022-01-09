package frc.robot.modules;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.KilloughDrive;
// make swerve drive if that is relevant

public class DriveBase {
	
	public static class DifferentialBase extends DifferentialDrive implements Types.Drivable {

		private static MotorController inlineInverter(MotorController m, boolean invert) {
			m.setInverted(invert);
			return m;
		}

		public<M extends MotorController> DifferentialBase(Types.DB2<M> map, Types.DBS s) {
			super(inlineInverter(map.type.create(map.left),  map.invert.left), inlineInverter(map.type.create(map.right), map.invert.right));
		}

		@Override public void tankDrive(double l, double r) { super.tankDrive(l, r); }
		@Override public void arcadeDrive(double s, double rot) { super.arcadeDrive(s, rot); }
		@Override public void raceDrive(double f, double b, double rot) { super.arcadeDrive(f-b, rot); }
		@Override public void curvatureDrive(double s, double rot, boolean q) { super.curvatureDrive(s, rot, q); }
		@Override public void topDownDrive(double x,  double y) { System.out.println("DifferentialBase: topDownDrive() is not supported"); }
	}
	// private class MecanumBase extends MecanumDrive implements Types.Drivable {

	// }
	// private class KilloughBase extends KilloughDrive implements Types.Drivable {

	// }

	private final Types.Drivable drive;

	public<M extends MotorController> DriveBase(Types.DB2<M> map, Types.DBS s) {
		drive = new DifferentialBase(map, s);
	}
	// public DriveBase(Types.DB3 map) {
	// 	drive = new KilloughBase();
	// }
	// public DriveBase(Types.DB4 map) {
	// 	switch(map.layout) {
	// 		case MECANUM: 
	// 			this.drive = new MecanumBase();
	// 			break;
	// 		case DIFFERENTIAL: 
	// 		default:
	// 			this.drive = new DifferentialBase();
	// 			break;


	// 	}
	// }
	
}
