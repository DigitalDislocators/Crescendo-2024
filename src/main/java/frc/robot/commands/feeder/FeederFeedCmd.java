package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSys;

public class FeederFeedCmd extends Command {

    private FeederSys feederSys;
    
    public FeederFeedCmd(FeederSys feederSys) {
        this.feederSys = feederSys;

        addRequirements(feederSys);
    }

    public void initialize() {
        feederSys.setPower(1.0);
    }

    public void execute() {

    }

    public void end(boolean isInerrupted) {

    }

    public boolean isFinished() {
        return true;
    }
}