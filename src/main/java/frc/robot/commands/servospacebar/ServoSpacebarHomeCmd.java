package frc.robot.commands.servospacebar;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoSpacebarSys;

public class ServoSpacebarHomeCmd extends Command {

    private ServoSpacebarSys spacebarSys;
    
    public ServoSpacebarHomeCmd(ServoSpacebarSys spacebarSys) {
        this.spacebarSys = spacebarSys;

        addRequirements(spacebarSys);
    }

    public void initialize() {
        spacebarSys.setSpacebarLeftPosition(0.125);
        spacebarSys.setSpacebarRightPosition(0.875 + 0.015);

    }

    public void execute() {

    }

    public void end(boolean isInerrupted) {

    }

    public boolean isFinished() {
        return true;
    }
}