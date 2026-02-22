// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;

import java.io.File;
import java.lang.annotation.Target;

import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.motors.SwerveMotor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  public double throttleTrans;
  public double throttleAngle;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandJoystick      m_driver1 = new CommandJoystick(0);
  final         CommandJoystick      m_driver2 = new CommandJoystick(1);
  final         CommandJoystick      m_buttonBox = new CommandJoystick(2);
  final         CommandJoystick      m_switchBox = new CommandJoystick(3);

  // The robot's subsystems and commands are defined here...
  final SwerveSubsystem       m_drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  final TargetingSubsystem    m_targeting = new TargetingSubsystem(m_drivebase);
  //final ShooterSubsystem      m_shooter = new ShooterSubsystem();

  // Create SmartDashboard chooser for autonomous and teleop routines
  private final SendableChooser<Command> m_chooserTeleop = new SendableChooser<>();
  private final SendableChooser<Command>   m_ChooserAuto = new SendableChooser<>();                                                                                
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                                                                () -> m_driver1.getRawAxis(1) * -1 * throttleTrans ,
                                                                () -> m_driver1.getRawAxis(0) * -1 * throttleTrans)
                                                            .withControllerRotationAxis(() -> m_driver1.getRawAxis(2) * -0.7 * throttleAngle)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true)
                                                            .aimWhile(m_driver1.button(4))
                                                            .aim(m_targeting.getGoalInMapFrame().orElse(null));

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> m_driver1.getRawAxis(2) * -1,
                                                                                             () -> m_driver1.getRawAxis(3) * -1)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                                                                        () -> -m_driver1.getRawAxis(1),
                                                                        () -> -m_driver1.getRawAxis(0))
                                                                    .withControllerRotationAxis(() -> m_driver1.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                m_driver1.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                m_driver1.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    throttleTrans = 1.0;
    throttleAngle = 1.0;
    m_drivebase.getSwerveController().setMaximumChassisAngularVelocity(3.1416);

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Drive Commands
    Command driveFieldOrientedDirectAngle      = m_drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = m_drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = m_drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = m_drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = m_drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = m_drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    // Named Commands
   // NamedCommands.registerCommand("Shoot", new Shoot(m_shooter, () -> m_driver1.getRawAxis(2)));
    //     NamedCommands.registerCommand("LowerToProcessor", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionProcessor, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionProcessor));
//     NamedCommands.registerCommand("RaiseToLowAlgae", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionA2, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionReef));
//     NamedCommands.registerCommand("RaiseToHighAlgae", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionA3, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionReef));
//     NamedCommands.registerCommand("LowerToCoralStation", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionDown, m_coral, Constants.CoralConstants.positionStation, m_algae, Constants.AlgaeArmConstants.positionUp));
//     NamedCommands.registerCommand("raise to L4", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionL4, m_coral, Constants.CoralConstants.positionReef, m_algae, Constants.AlgaeArmConstants.positionUp));
//     NamedCommands.registerCommand("raise to L3", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionL3, m_coral, Constants.CoralConstants.positionReef, m_algae, Constants.AlgaeArmConstants.positionUp));
//     NamedCommands.registerCommand("raise to L2", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionL2, m_coral, Constants.CoralConstants.positionReef, m_algae, Constants.AlgaeArmConstants.positionUp));
//     NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(m_coral, Constants.CoralConstants.powerIntake));
//     NamedCommands.registerCommand("LaunchCoral", new LaunchCoral(m_coral, Constants.CoralConstants.powerLaunch));
//    // NamedCommands.registerCommand("StationPosition", new IntakeCoral(m_coral, Constants.CoralConstants.positionStation));
//     NamedCommands.registerCommand("raise to station", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionDown, m_coral, Constants.CoralConstants.positionStation, m_algae, Constants.AlgaeArmConstants.positionUp));
// //lots of issues with this one
//     NamedCommands.registerCommand("LineUpAlgae", new AutoScoreCoral(m_targeting, 0.0, 0.03, 10, m_drivebase, m_elevator, m_coral, m_algae));
//     NamedCommands.registerCommand("Algae In", new IntakeAlgae(m_algae, Constants.AlgaeArmConstants.powerIntake));


//     NamedCommands.registerCommand("ScoreCoralRight", new AutoScoreCoral(m_targeting, 0.155, 0.26, 10, m_drivebase, m_elevator, m_coral, m_algae));
//     NamedCommands.registerCommand("ScoreCoralLeft", new AutoScoreCoral(m_targeting, -0.155, 0.26, 10, m_drivebase, m_elevator, m_coral, m_algae));
//     NamedCommands.registerCommand("Raise to Net", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionNet, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionNet));
//     NamedCommands.registerCommand("Algae Out", new LaunchAlgae(m_algae, Constants.AlgaeArmConstants.powerLaunch));
//     NamedCommands.registerCommand("raise to A2", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionA2, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionReef));
//     NamedCommands.registerCommand("raise to Processor", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionProcessor, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionProcessor)); 
//     NamedCommands.registerCommand("raise to A3", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionA3, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionReef));


    // Setup SmartDashboard chooser options
    m_chooserTeleop.setDefaultOption("driveFieldOrientedDirectAngle", driveFieldOrientedDirectAngle);
    m_chooserTeleop.addOption("driveFieldOrientedAnglularVelocity", driveFieldOrientedAnglularVelocity);
    m_chooserTeleop.addOption("driveRobotOrientedAngularVelocity", driveRobotOrientedAngularVelocity);
    SmartDashboard.putData("Teleop Mode", m_chooserTeleop);

    m_ChooserAuto.setDefaultOption("New Auto", m_drivebase.getAutonomousCommand("New Auto"));
    // m_ChooserAuto.addOption("3 Back Left L4", m_drivebase.getAutonomousCommand("BACK LEFT 3 L4"));
    // m_ChooserAuto.addOption("RIGHT BACK 3 L4", m_drivebase.getAutonomousCommand("RIGHT BACK 3 L4"));
    // m_ChooserAuto.addOption("3 Front Right L4", m_drivebase.getAutonomousCommand("RIGHT FRONT 3 L4"));
    // m_ChooserAuto.addOption("3 Front Left L4", m_drivebase.getAutonomousCommand("LEFT FRONT 3 L4"));
    // m_ChooserAuto.addOption("Middle L4 and net", m_drivebase.getAutonomousCommand("MIDDLE algae and L4"));
    //  m_ChooserAuto.addOption("3 Front Left L4 REVISED", m_drivebase.getAutonomousCommand("LEFT FRONT 3 L4 REVISED"));


    SmartDashboard.putData("Auto Mode", m_ChooserAuto);


    if (RobotBase.isSimulation())
    {
      m_drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      m_drivebase.setDefaultCommand(getTeleopDriveCommand());
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      m_driver2.button(1).onTrue(Commands.runOnce(() -> m_drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      //driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      //driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
      //                                               () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
      

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      m_driver1.button(3).whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      // m_driverSwitch.button(4).whileTrue(m_drivebase.driveToDistanceCommand(1.0, 0.2));
      m_driver1.button(8).onTrue((Commands.runOnce(m_drivebase::zeroGyroWithAlliance)));
      m_driver1.button(7).whileTrue(m_drivebase.centerModulesCommand());
      m_driver1.button(5).onTrue(Commands.none());
      m_driver1.button(6).onTrue(Commands.none());
    } else
    {
      // BUTTON CONTROLS
      m_driver1.button(2).onTrue((Commands.runOnce(m_drivebase::zeroGyroWithAlliance)));
       m_driver1.button(11).whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      m_driver1.button(3).onTrue(Commands.runOnce(m_drivebase::addFakeVisionReading));
      m_driver1.button(9).whileTrue(NamedCommands.getCommand("Shoot"));
      // m_driverSwitch.button(10).onTrue(m_drivebase.driveToDistanceCommand(2.0, 1.0));
    }
  }
      


 /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return m_ChooserAuto.getSelected();
  }

  public Command getTeleopDriveCommand() {
    return m_chooserTeleop.getSelected();
  }

  public double calcThrottle()
  {
    return Constants.minThrottle + (Constants.maxThrottle - Constants.minThrottle);
  }

  public void setMotorBrake(boolean brake)
  {
    m_drivebase.setMotorBrake(brake);
  }
}
