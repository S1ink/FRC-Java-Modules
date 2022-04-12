package frc.robot.team3407.commandbased;

import java.lang.Runnable;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class RunForCommand extends CommandBase {

	private final Runnable func;
	private final int loops;
	private int count = 0;

	public RunForCommand(Runnable f, int loops) {
		this.func = f;
		this.loops = loops;
	}

	@Override public void initialize() {
		this.count = 0;
	}
	@Override public void execute() {
		this.func.run();
		this.count++;
	}
	@Override public boolean isFinished() {
		return this.count <= this.loops;
	}


}