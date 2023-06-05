package frc.robot.team3407;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Util {

	/** Passthough for putting a sendable on SmartDashboard */
	public static <T extends Sendable> T send(T t) {
		SmartDashboard.putData(t);
		return t;
	}
	/** Passthough for putting a sendable on SmartDashboard */
	public static <T extends Sendable> T send(T t, String key) {
		SmartDashboard.putData(key, t);
		return t;
	}

	public static double zeroRange(double v, double epsilon) {
		return (v < epsilon && v > -epsilon) ? 0.0 : v;
	}
	public static double clamp(double v, double min, double max) {
		return Math.min(max, Math.max(min, v));
	}
	public static double clampEq(double v, double range) {
		return clamp(v, -Math.abs(range), Math.abs(range));
	}

}
