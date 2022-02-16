package frc.robot.modules.common.drive;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;


public class Helper {
	
	// public static MotorController inlineInverter(MotorController m, boolean invert) {
	// 	m.setInverted(invert);
	// 	return m;
	// }
	public static<M extends MotorController> M getInverted(M m, boolean invert) {
		m.setInverted(invert);
		return m;
	}
	public static void applyDeceleration(Types.Deceleration constant, MotorController m, MotorController... ms) {
		m.set(m.get() * constant.value);
		for(int i = 0; i < ms.length; i++) {
			ms[i].set(ms[i].get() * constant.value);
		}
	}
	public static void applyDeceleration(Types.Deceleration constant, MotorController[] motors) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].set(motors[i].get() * constant.value);
		}
	}
	public static void applyPercentage(MotorController[] motors, double p) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].set(motors[i].get() * p);
		}
	}
	public static void applyAll(MotorController[] motors, double s) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].set(s);
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


}