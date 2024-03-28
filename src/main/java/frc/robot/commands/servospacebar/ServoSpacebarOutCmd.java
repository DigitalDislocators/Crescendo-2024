package frc.robot.commands.servospacebar;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoSpacebarSys;

public class ServoSpacebarOutCmd extends Command {

    private ServoSpacebarSys spacebarSys;
    
    public ServoSpacebarOutCmd(ServoSpacebarSys spacebarSys) {
        this.spacebarSys = spacebarSys;

        addRequirements(spacebarSys);
    }

    public void initialize() {
        spacebarSys.setSpacebarLeftPosition(0.875 + .1);
        spacebarSys.setSpacebarRightPosition(0.12 + 0.015 - .1);
    }

    public void execute() {

    }

    public void end(boolean isInerrupted) {

    }

    public boolean isFinished() {
        return true;
    }
}