package frc.robot.modules.common;

import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * Trigger becomes active when base trigger is initially active and becomes inactive when the base becomes active again
 */
public class ToggleTrigger extends Trigger {

	private boolean is_triggered = false;

	public ToggleTrigger(Trigger t) {
		t.whenActive(()->this.is_triggered = !this.is_triggered);
	}
	
	@Override public boolean get() {
		return this.is_triggered;
	}


}