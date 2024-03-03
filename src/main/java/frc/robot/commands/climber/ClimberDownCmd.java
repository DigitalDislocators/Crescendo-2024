package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSys;

public class ClimberDownCmd extends Command {

    private ClimberSys climberSys;
    
    public ClimberDownCmd(ClimberSys climberSys) {
        this.climberSys = climberSys;

        addRequirements(climberSys);
    }

    public void initialize() {
        climberSys.setClimberPower(-1.0);
    }

    public void execute() {

    }

    public void end(boolean isInterrupted) {
        
    }

    public boolean isFinished() {
        return true;
    }
}