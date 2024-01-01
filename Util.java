package frc.robot.team3407;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.team3407.SenderNT.RecursiveSendable;


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

	/** Overload for {@link SenderNT} instances in place of the SmartDashboard singleton */
	public static <T extends Sendable> T send(T t, SenderNT sender) {
		sender.putData(t);
		return t;
	}
	/** Overload for {@link SenderNT} instances in place of the SmartDashboard singleton */
	public static <T extends Sendable> T send(T t, SenderNT sender, String key) {
		sender.putData(key, t);
		return t;
	}
	/** Overload for {@link SenderNT} instances with the use of {@link RecursiveSendable} sendables */
	public static <T extends RecursiveSendable> T send(T t, SenderNT sender, String key) {
		sender.putData(key, t);
		return t;
	}

	/** Test if a value is within a certain epsilon away from zero. */
	public static boolean isZero(double v, double epsilon) {
		return (v < epsilon && v > -epsilon);
	}
	/** Return 0.0 if a value is within a certain epsilon away from zero, else return the value. */
	public static double zeroRange(double v, double epsilon) {
		return isZero(v, epsilon) ? 0.0 : v;
	}
	/** Clamp a value between two bounds. */
	public static double clamp(double v, double min, double max) {
		return Math.min(max, Math.max(min, v));
	}
	/** Clamp a value between the positive and negatives of a range. */
	public static double clampEq(double v, double range) {
		return clamp(v, -Math.abs(range), Math.abs(range));
	}
	/** Return the sign of a number, returning 1.0 if the number is equal to 0. */
	public static double sgnnz(double v) {	// 'sgn', No Zero
		return v >= 0.0 ? 1.0 : -1.0;
	}


	/** Convert a set of 2d poses to an array of telemetry values. */
	public static double[] toComponents2d(Pose2d... poses) {
		final double[] values = new double[poses.length * 3];
		for(int i = 0; i < poses.length; i++) {
			int offset = i * 3;
			values[offset + 0] = poses[i].getX();
			values[offset + 1] = poses[i].getY();
			values[offset + 2] = poses[i].getRotation().getRadians();
		}
		return values;
	}
	/** Convert a set of 3d poses to an array of telemetry values. */
	public static double[] toComponents3d(Pose3d... poses) {
		final double[] values = new double[poses.length * 7];
		for(int i = 0; i < poses.length; i++) {
			int offset = i * 7;
			Quaternion q = poses[i].getRotation().getQuaternion();
			values[offset + 0] = poses[i].getX();
			values[offset + 1] = poses[i].getY();
			values[offset + 2] = poses[i].getZ();
			values[offset + 3] = q.getW();
			values[offset + 4] = q.getX();
			values[offset + 5] = q.getY();
			values[offset + 6] = q.getZ();
		}
		return values;
	}

}
