package frc.robot.commands.automation;

import java.util.List;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.auto.FollowPathCmd;
import frc.robot.subsystems.SwerveSys;

public class AutoDriveToAmpCmd extends Command {

    private final SwerveSys swerveSys;

    private FollowPathCmd followPathCmd;

    public AutoDriveToAmpCmd(SwerveSys swerveSys) {
        this.swerveSys = swerveSys;

        addRequirements(swerveSys);
    }

    @Override
    public void initialize() {
        List<Translation2d> waypoints = PathPlannerPath.bezierFromPoses(
            swerveSys.getPose(),
            AutoConstants.driveToAmpWaypoint,
            AutoConstants.driveToAmpTargetPoint
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(
                AutoConstants.driveToAmpMaxVelMetersPerSec,
                AutoConstants.driveToAmpMaxAccelMetersPerSecSq,
                AutoConstants.autoAimTurnSpeedRadPerSec,
                AutoConstants.autoAumTurnAccelRadPerSecSq),
            new GoalEndState(0.0, AutoConstants.driveToAmpTargetPoint.getRotation(), true));
        
        followPathCmd = new FollowPathCmd(path, swerveSys);
        followPathCmd.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        followPathCmd.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}