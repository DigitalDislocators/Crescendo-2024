package frc.robot.commands.auto;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSys;

public class FollowPathCmd extends FollowPathHolonomic {

    private final SwerveSys swerveSys;
    
    /**
     * Creates a new FollowPathCmd.
     * <p>
     * FollowPathCmd follows a path using {@link FollowPathHolonomic} using the
     * maximum velocity given in the {@link AutoConstants} class.
     * 
     * @param path The path to follow.
     * @param swerveSys The SwerveSys to follow the path.
     */
    public FollowPathCmd(PathPlannerPath path, SwerveSys swerveSys) {
        super(
            path,
            swerveSys::getPose,
            swerveSys::getChassisSpeeds,
            swerveSys::setChassisSpeeds,
            new PIDConstants(AutoConstants.drivekP, AutoConstants.drivekD),
            new PIDConstants(AutoConstants.rotkP, AutoConstants.rotkD),
            DriveConstants.maxModuleSpeedMetersPerSec,
            Math.hypot(DriveConstants.trackWidth / 2.0, DriveConstants.wheelBase / 2.0),
            new ReplanningConfig(),
            () -> DriverStation.getAlliance().get() == Alliance.Red);

        this.swerveSys = swerveSys;
    }

    /**
     * Creates a new FollowPathCmd.
     * <p>
     * FollowPathCmd follows a path using {@link FollowPathHolonomic} using the
     * maximum velocity given in the {@link AutoConstants} class.
     * 
     * @param pathName The name of the path given in PathPlanner.
     * @param swerveSys The SwerveSys to follow the path.
     */
    public FollowPathCmd(String pathName, SwerveSys swerveSys) {
        this(PathPlannerPath.fromPathFile(pathName), swerveSys);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerveSys.stop();
    }
}