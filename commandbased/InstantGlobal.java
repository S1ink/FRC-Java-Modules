package frc.robot.team3407.commandbased;

import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * A base command that instantly finishes and can run in disabled mode
 */
public class InstantGlobal extends CommandBase {

	@Override public boolean isFinished() { return true; }
	@Override public boolean runsWhenDisabled() { return true; }


}