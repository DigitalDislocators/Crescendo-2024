package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitCmd extends Command {

    private final Timer timer;

    private final double seconds;

    public WaitCmd(double seconds) {
        this.seconds = seconds;

        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {

    }
    
    @Override
    public void end(boolean interrupted) {
        
    }
    
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}