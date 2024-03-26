package frc.robot.commands.spacebar;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpacebarSys;

public class SpacebarOutCmd extends Command {

    private SpacebarSys spacebarSys;
    
    public SpacebarOutCmd(SpacebarSys spacebarSys) {
        this.spacebarSys = spacebarSys;

        addRequirements(spacebarSys);
    }

    public void initialize() {
        spacebarSys.setSpacebarLeftPosition(0.875);
        spacebarSys.setSpacebarRightPosition(0.12 + 0.015);
    }

    public void execute() {

    }

    public void end(boolean isInerrupted) {

    }

    public boolean isFinished() {
        return true;
    }
}