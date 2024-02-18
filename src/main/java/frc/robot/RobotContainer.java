package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
//import frc.robot.commands.IntakeCmd;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.subsystems.RollerSys;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.ServoSys;
import frc.robot.subsystems.ClimberSys;
import frc.robot.subsystems.SwerveSys;
import frc.robot.commands.PivotManualCmd;
import frc.robot.commands.ManualClimbCmd;
import frc.robot.commands.ManualRollersCmd;

public class RobotContainer {
    

    // Initialize subsystems.
    private final SwerveSys swerveSys = new SwerveSys();
    private final PivotSys pivotSys = new PivotSys();
    private final RollerSys rollerSys = new RollerSys();
    private final ClimberSys climberSys = new ClimberSys();

    // Initialize joysticks.

  private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverGamepadPort);
  private final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorGamepadPort);

  
    // Initialize auto selector.
    //SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public RobotContainer() {
        // SmartDashboard.putData("auto selector", autoSelector);

        // intakeSys.setDefaultCommand(getAutonomousCommand());

        // Add programs to auto selector.
        // autoSelector.setDefaultOption("Do Nothing", null);

        configDriverBindings();
        configOperatorsBindings();

        swerveSys.setDefaultCommand(new ArcadeDriveCmd(
            () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
            true,
            true,
            swerveSys));
    }

    private void configOperatorsBindings() {
       rollerSys.setDefaultCommand(new ManualRollersCmd(
        () -> (operatorController.getRightTriggerAxis() * 0.5) - (operatorController.getLeftTriggerAxis() * 0.3), rollerSys));

       pivotSys.setDefaultCommand(new PivotManualCmd( 
        () -> MathUtil.applyDeadband(operatorController.getLeftY(), ControllerConstants.joystickDeadband), pivotSys));

       climberSys.setDefaultCommand(new ManualClimbCmd(
        () -> MathUtil.applyDeadband((operatorController.getRightY() * -1), ControllerConstants.joystickDeadband), climberSys));
      
      
            //ServoSys.set
    }

    public void configDriverBindings() {
       
    }

    public Command getAutonomousCommand() {
        return null;//return autoSelector.getSelected();
    }

    // For uniformity, any information sent to Shuffleboard/SmartDashboard should go here.
    /*public void updateInterface() {
        SmartDashboard.putNumber("heading degrees", swerveSys.getHeading().getDegrees());
        SmartDashboard.putNumber("speed m/s", swerveSys.getAverageDriveVelocityMetersPerSec());

        SmartDashboard.putNumber("FL angle degrees", swerveSys.getModuleStates()[0].angle.getDegrees());
        SmartDashboard.putNumber("FR angle degrees", swerveSys.getModuleStates()[1].angle.getDegrees());
        SmartDashboard.putNumber("BL angle degrees", swerveSys.getModuleStates()[2].angle.getDegrees());
        SmartDashboard.putNumber("BR angle degrees", swerveSys.getModuleStates()[3].angle.getDegrees());

        SmartDashboard.putNumber("FL raw CANCoder degrees", swerveSys.getCanCoderAngles()[0].getDegrees());
        SmartDashboard.putNumber("FR raw CANCoder degrees", swerveSys.getCanCoderAngles()[1].getDegrees());
        SmartDashboard.putNumber("BL raw CANCoder degrees", swerveSys.getCanCoderAngles()[2].getDegrees());
        SmartDashboard.putNumber("BR raw CANCoder degrees", swerveSys.getCanCoderAngles()[3].getDegrees());

        SmartDashboard.putNumber("FL offset CANCoder degrees", swerveSys.getCanCoderAngles()[0].getDegrees() - DriveConstants.frontLeftModOffset.getDegrees());
        SmartDashboard.putNumber("FR offset CANCoder degrees", swerveSys.getCanCoderAngles()[1].getDegrees() - DriveConstants.frontRightModOffset.getDegrees());
        SmartDashboard.putNumber("BL offset CANCoder degrees", swerveSys.getCanCoderAngles()[2].getDegrees() - DriveConstants.backLeftModOffset.getDegrees());
        SmartDashboard.putNumber("BR offset CANCoder degrees", swerveSys.getCanCoderAngles()[3].getDegrees() - DriveConstants.backRightModOffset.getDegrees());
    }*/
}
