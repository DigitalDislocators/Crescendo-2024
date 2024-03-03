package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSys;

public class ClimberStopCmd extends Command {

    private ClimberSys climberSys;
    
    public ClimberStopCmd(ClimberSys climberSys) {
        this.climberSys = climberSys;

        addRequirements(climberSys);
    }

    public void initialize() {
        climberSys.setClimberPower(0.0);
    }

    public void execute() {
    
    }

    public void end(boolean isInerrupted) {
        
    }

    public boolean isFinished() {
        return true;
    }
}