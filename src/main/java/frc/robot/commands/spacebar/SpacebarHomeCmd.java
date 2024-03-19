package frc.robot.commands.spacebar;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpacebarSys;

public class SpacebarHomeCmd extends Command {

    private SpacebarSys spacebarSys;
    
    public SpacebarHomeCmd(SpacebarSys spacebarSys) {
        this.spacebarSys = spacebarSys;

        addRequirements(spacebarSys);
    }

    public void initialize() {
        spacebarSys.setPosition(0.0);
    }

    public void execute() {

    }

    public void end(boolean isInerrupted) {

    }

    public boolean isFinished() {
        return true;
    }
}