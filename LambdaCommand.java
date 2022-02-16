package frc.robot.modules.common;

import edu.wpi.first.wpilibj2.command.CommandBase;


// Runnable can already be passed into most commandscheduler methods, but this acts as a wrapper for use in creating command groups
public class LambdaCommand extends CommandBase {

	private final Runnable run;
	public LambdaCommand(Runnable r) {
		this.run = r;
	}
	@Override public void initialize() { run.run(); }
	@Override public boolean isFinished() { return true; }
	@Override public boolean runsWhenDisabled() { return true; }


}