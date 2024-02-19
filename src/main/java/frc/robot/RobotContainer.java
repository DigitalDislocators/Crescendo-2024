package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.commands.feeder.FeederFeedCmd;
import frc.robot.commands.feeder.FeederInCmd;
import frc.robot.commands.feeder.FeederStopCmd;
import frc.robot.commands.pivot.PivotTrapPresetCmd;
import frc.robot.commands.pivot.PivotHomePresetCmd;
import frc.robot.commands.pivot.PivotPodiumPresetCmd;
import frc.robot.commands.pivot.PivotSourcePresetCmd;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.ClimberSys;
import frc.robot.subsystems.FeederSys;
import frc.robot.subsystems.SwerveSys;
import frc.robot.commands.pivot.PivotManualCmd;
import frc.robot.commands.automation.AutoAllHomeCmd;
import frc.robot.commands.automation.AutoIntakeCmd;
import frc.robot.commands.automation.AutoShootCmd;
import frc.robot.commands.climber.ClimberDownCmd;
import frc.robot.commands.climber.ClimberUpCmd;
import frc.robot.commands.rollers.RollersManualCmd;

public class RobotContainer {
    
    // Initialize subsystems.
    private final SwerveSys swerveSys = new SwerveSys();
    private final PivotSys pivotSys = new PivotSys();
    private final RollersSys rollerSys = new RollersSys();
    private final FeederSys feederSys = new FeederSys();
    private final ClimberSys climberSys = new ClimberSys();

    //Initialize joysticks.
    private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverGamepadPort);
    private final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorGamepadPort);

    //Initialize auto selector.
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
        rollerSys.setDefaultCommand(new RollersManualCmd(
            () -> (operatorController.getRightTriggerAxis() * RollerConstants.manualShootPower) - 
                  (operatorController.getLeftTriggerAxis() * RollerConstants.manualIntakePower),
            rollerSys));

        pivotSys.setDefaultCommand(new PivotManualCmd( 
            () -> MathUtil.applyDeadband((operatorController.getLeftY()), ControllerConstants.joystickDeadband),
            pivotSys));

        operatorController.a().onTrue(new PivotTrapPresetCmd(pivotSys));

        operatorController.b().onTrue(new PivotPodiumPresetCmd(pivotSys));

        operatorController.x().onTrue(new PivotHomePresetCmd(pivotSys));

        operatorController.y().onTrue(new PivotSourcePresetCmd(pivotSys));

        operatorController.rightBumper().onTrue(new FeederFeedCmd(feederSys)).onFalse(new FeederStopCmd(feederSys));
      
        operatorController.leftBumper().onTrue(new FeederInCmd(feederSys)).onFalse(new FeederStopCmd(feederSys));

        operatorController.povUp().whileTrue(new ClimberUpCmd(climberSys)).onFalse(new ClimberDownCmd(climberSys));

        operatorController.povRight().onTrue(new AutoShootCmd(feederSys, rollerSys));
    }

    public void configDriverBindings() {
        driverController.start().onTrue(Commands.runOnce(() -> swerveSys.resetHeading()));

        driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold)
            .whileTrue(Commands.runOnce(() -> swerveSys.lock()));
        
        driverController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.triggerPressedThreshhold)
            .onTrue(new AutoIntakeCmd(pivotSys, feederSys, rollerSys)).onFalse(new AutoAllHomeCmd(pivotSys, feederSys, rollerSys));
    }

    public Command getAutonomousCommand() {
        return null;
        //return autoSelector.getSelected();
    } 

    // For uniformity, any information sent to Shuffleboard/SmartDashboard should go here.
    public void updateInterface() {
        SmartDashboard.putNumber("pivot degrees", pivotSys.getCurrentPositionDegrees());
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
    }
}
