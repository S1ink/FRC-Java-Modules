package frc.robot.team3407.commandbased;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Trigger becomes active after the condition is true, then never becomes unactive
 */
public class EternalTrigger extends Trigger {

	private boolean has_triggered = false;

	public EternalTrigger(BooleanSupplier t) { this(new Trigger(t)); }
	public EternalTrigger(Trigger t) {
		super(()->false);
		// t.onTrue(()->this.has_triggered = true);
	}


}