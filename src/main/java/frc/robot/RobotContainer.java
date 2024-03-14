package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.commands.drivetrain.LockCmd;
import frc.robot.commands.drivetrain.TurnToHeadingCmd;
import frc.robot.commands.drivetrain.AimToSpeakerCmd;
import frc.robot.commands.feeder.FeederFeedCmd;
import frc.robot.commands.feeder.FeederInCmd;
import frc.robot.commands.feeder.FeederStopCmd;
import frc.robot.commands.lights.LightsDefaultCmd;
import frc.robot.commands.lights.PartyModeCmd;
import frc.robot.commands.pivot.PivotHomePresetCmd;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.ClimberSys;
import frc.robot.subsystems.FeederSys;
import frc.robot.subsystems.LightsSys;
// import frc.robot.subsystems.LimelightSys;
import frc.robot.subsystems.SwerveSys;
import frc.robot.commands.pivot.PivotManualCmd;
import frc.robot.commands.auto.programs.AllianceNoteFivePiece;
import frc.robot.commands.auto.programs.AllianceNoteFourPiece;
import frc.robot.commands.auto.programs.ExampleAuto;
import frc.robot.commands.auto.programs.MidlineNoteThreePiece;
import frc.robot.commands.auto.programs.PiHiThreePiece;
import frc.robot.commands.auto.programs.TestFivePiece;
import frc.robot.commands.automation.AutoAllHomeCmd;
import frc.robot.commands.automation.AutoGroundIntakeCmd;
import frc.robot.commands.automation.AutoAmpFireCmd;
import frc.robot.commands.automation.AutoSourceIntakeCmd;
import frc.robot.commands.automation.AutoSpeakerFireCmd;
import frc.robot.commands.automation.AutoSubwooferFireCmd;
import frc.robot.commands.climber.ClimberDownCmd;
import frc.robot.commands.climber.ClimberUpCmd;
import frc.robot.commands.rollers.RollersFireCmd;
import frc.robot.commands.rollers.RollersIntakeCmd;
import frc.robot.commands.rollers.RollersStopCmd;

public class RobotContainer {
    
    // Initialize subsystems.
    private final SwerveSys swerveSys = new SwerveSys();
    private final PivotSys pivotSys = new PivotSys();
    private final RollersSys rollerSys = new RollersSys();
    private final FeederSys feederSys = new FeederSys();
    private final ClimberSys climberSys = new ClimberSys();
    private final LightsSys lightsSys = new LightsSys();

    //Initialize joysticks.
    private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverGamepadPort);
    private final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorGamepadPort);

    //Initialize auto selector.
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public RobotContainer() {
        RobotController.setBrownoutVoltage(DriveConstants.brownoutVoltage);

        SmartDashboard.putData("auto selector", autoSelector);

        // Add programs to auto selector.
        autoSelector.setDefaultOption("Do Nothing", null);
        autoSelector.addOption("Example Auto", new ExampleAuto(swerveSys));
        autoSelector.addOption("AllianceNoteFourPiece", new AllianceNoteFourPiece(swerveSys, feederSys, rollerSys, pivotSys));
        autoSelector.addOption("AllianceNoteFivePiece", new AllianceNoteFivePiece(swerveSys, feederSys, rollerSys, pivotSys));
        autoSelector.addOption("MidlineNoteThreePiece", new MidlineNoteThreePiece(swerveSys, feederSys, rollerSys, pivotSys));
        autoSelector.addOption("PiHiThreePiece", new PiHiThreePiece(swerveSys, feederSys, rollerSys, pivotSys));
        autoSelector.addOption("TestFivePiece", new TestFivePiece(swerveSys, feederSys, rollerSys, pivotSys));

        configDriverBindings();
        configOperatorBindings();

        lightsSys.setDefaultCommand(new LightsDefaultCmd(lightsSys));
    }

    private void configOperatorBindings() {
        // rollerSys.setDefaultCommand(new RollersManualCmd(
        //     () -> (operatorController.getRightTriggerAxis() * RollerConstants.manualFirePower) - 
        //           (operatorController.getLeftTriggerAxis() * RollerConstants.manualIntakePower),
        //     rollerSys));

        operatorController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold).onFalse(new RollersStopCmd(rollerSys));

        operatorController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.triggerPressedThreshhold).onFalse(new RollersStopCmd(rollerSys));

        pivotSys.setDefaultCommand(new PivotManualCmd( 
            () -> MathUtil.applyDeadband((operatorController.getLeftY()), ControllerConstants.joystickDeadband),
            pivotSys));

        operatorController.a().onTrue(new AutoAmpFireCmd(feederSys, rollerSys, pivotSys));

        // operatorController.b().onTrue(new AutoPodiumFireCmd(feederSys, rollerSys, pivotSys));
        // operatorController.b().and(driverController.a()).onTrue(new AutoSpeakerFireCmd(feederSys, rollerSys, pivotSys, swerveSys));

        operatorController.x().onTrue(new PivotHomePresetCmd(pivotSys));

        operatorController.y().onTrue(new AutoSubwooferFireCmd(feederSys, rollerSys, pivotSys));

        operatorController.rightBumper().onTrue(new FeederFeedCmd(feederSys)).onFalse(new FeederStopCmd(feederSys));
      
        operatorController.leftBumper().onTrue(new FeederInCmd(feederSys)).onFalse(new FeederStopCmd(feederSys));

        operatorController.povUp().whileTrue(new ClimberUpCmd(climberSys)).whileFalse(new ClimberDownCmd(climberSys));

        operatorController.b()
            .onTrue(new AutoSpeakerFireCmd(feederSys, rollerSys, pivotSys, swerveSys))
            .onFalse(new RollersStopCmd(rollerSys))
            .onFalse(new PivotHomePresetCmd(pivotSys))
            .onFalse(new FeederStopCmd(feederSys));
            
        operatorController.povRight().onTrue(new AutoSourceIntakeCmd(pivotSys, feederSys, rollerSys)).onFalse(new AutoAllHomeCmd(pivotSys, feederSys, rollerSys));

        operatorController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.triggerPressedThreshhold)
            .onTrue(new RollersFireCmd(rollerSys))
            .onFalse(new RollersStopCmd(rollerSys));

        operatorController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold)
            .onTrue(new RollersIntakeCmd(rollerSys))
            .onFalse(new RollersStopCmd(rollerSys));

        operatorController.start().toggleOnTrue(new PartyModeCmd(lightsSys));
    }

    public void configDriverBindings() {
        swerveSys.setDefaultCommand(new ArcadeDriveCmd(
            () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
            true,
            true,
            swerveSys));

        driverController.start().onTrue(Commands.runOnce(() -> swerveSys.resetHeading()));

        driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold)
            .whileTrue(new LockCmd(swerveSys));
        
        driverController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.triggerPressedThreshhold)
            .onTrue(new AutoGroundIntakeCmd(pivotSys, feederSys, rollerSys)).onFalse(new AutoAllHomeCmd(pivotSys, feederSys, rollerSys));

        driverController.rightBumper().onTrue(new AutoSourceIntakeCmd(pivotSys, feederSys, rollerSys)).onFalse(new AutoAllHomeCmd(pivotSys, feederSys, rollerSys));
        
        driverController.a().whileTrue(new AimToSpeakerCmd(swerveSys));
        driverController.b().whileTrue(new TurnToHeadingCmd(Rotation2d.fromDegrees(90), swerveSys));
        driverController.x().whileTrue(new TurnToHeadingCmd(Rotation2d.fromDegrees(-60), swerveSys));
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    } 

    // For uniformity, any information sent to Shuffleboard/SmartDashboard should go here.
    public void updateInterface() {
        SmartDashboard.putNumber("heading degrees", swerveSys.getHeading().getDegrees());
        SmartDashboard.putNumber("speed m/s", swerveSys.getAverageDriveVelocityMetersPerSec());

        SmartDashboard.putNumber("pose x meters", swerveSys.getPose().getX());
        SmartDashboard.putNumber("pose y meters", swerveSys.getPose().getY());

        SmartDashboard.putNumber("blue pose x meters", swerveSys.getBlueSidePose().getX());

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

        SmartDashboard.putNumber("pivot degrees", pivotSys.getCurrentPositionDeg());

        SmartDashboard.putNumber("drive voltage", swerveSys.getAverageDriveVoltage());

        SmartDashboard.putNumber("roller rpm", rollerSys.getRPM());

    }
}
