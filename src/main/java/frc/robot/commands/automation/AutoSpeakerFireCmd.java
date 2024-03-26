package frc.robot.commands.automation;

import frc.robot.commands.feeder.FeederFeedCmd;
import frc.robot.commands.feeder.FeederStopCmd;
import frc.robot.commands.pivot.PivotHomePresetCmd;
import frc.robot.commands.rollers.RollersFireCmd;
import frc.robot.commands.rollers.RollersStopCmd;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.FeederSys;
import frc.robot.subsystems.PivotSys;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoSpeakerFireCmd extends SequentialCommandGroup {

	public AutoSpeakerFireCmd(FeederSys feeder, RollersSys rollers, PivotSys pivot, SwerveSys swerveSys) {
		super(
			new RollersFireCmd(rollers),
			new AutoSetPivotToSpeakerCmd(swerveSys, pivot).raceWith(
				new WaitCommand(.75).andThen(
				new FeederFeedCmd(feeder)).andThen(
				new WaitCommand(0.8))),
			new PivotHomePresetCmd(pivot),
			new RollersStopCmd(rollers),
			new FeederStopCmd(feeder)
		);
	}
}