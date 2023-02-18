package frc.robot.team3407.commandbased;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.team3407.commandbased.Commands.*;


public final class Triggers {
    
    // private static class Toggler implements BooleanSupplier {

    // }

    // public static Trigger createToggle(BooleanSupplier b) {

    // }



    // /**
	//  * Trigger becomes active after the condition is true, then never becomes unactive
	//  */
	// public class EternalTrigger extends Trigger {

	// 	private boolean has_triggered = false;

	// 	public EternalTrigger(BooleanSupplier t) { this(new Trigger(t)); }
	// 	public EternalTrigger(Trigger t) {
	// 		super(()->false);
	// 		t.onTrue(new LambdaCommand(()->{ this.has_triggered = true; }));
	// 	}

	// }

	// /**
	//  * Bascially just wraps a boolean
	//  */
	// public class StaticTrigger extends Trigger {

	// 	private boolean state;

	// 	public StaticTrigger(boolean init) {
	// 		this.state = init;
	// 	}

	// 	public void enable() { this.state = true; }
	// 	public void disable() { this.state = false; }
	// 	public void setState(boolean val) { this.state = val; }

	// 	// @Override public boolean get() {
	// 	// 	return this.state;
	// 	// }

	// }

	// /**
	//  * Trigger becomes active when base trigger is initially active and becomes inactive when the base becomes active again
	//  */
	// public class ToggleTrigger extends Trigger {

	// 	private boolean is_triggered = false;

	// 	public ToggleTrigger(Trigger t) {
	// 		t.whenActive(()->this.is_triggered = !this.is_triggered);
	// 	}
	// 	public ToggleTrigger(BooleanSupplier t) { this(new Trigger(t)); }
		
	// 	// @Override public boolean get() {
	// 	// 	return this.is_triggered;
	// 	// }

	// }

}
