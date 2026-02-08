// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double maxThrottle = 1.0;
  public static final double minThrottle = 0.4;

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(12);// Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  
  public static final Pose2d startPoseBlue = new Pose2d(new Translation2d(Meter.of(1),
                                                                          Meter.of(4)),
                                                              Rotation2d.fromDegrees(0));
  public static final Pose2d startPoseRed = new Pose2d(new Translation2d(Meter.of(16),
                                                                         Meter.of(4)),
                                                             Rotation2d.fromDegrees(180));
  
  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class HangerConstants
  {

  }

  public static final class ShooterConstants
  {

  }

  public static final class VisionConstants
  {
    public static final String Camera1Name = "center";
    public static final Rotation3d Camera1Rotation = new Rotation3d(0, Units.degreesToRadians(15), 0);
    public static final Translation3d Camera1Translation = new Translation3d(Units.inchesToMeters(13.5),
                                                                            Units.inchesToMeters(0.0),
                                                                            Units.inchesToMeters(8.0));

    public static final String Camera2Name = "left";
    public static final Rotation3d Camera2Rotation = new Rotation3d(0, Units.degreesToRadians(15), 0);
    public static final Translation3d Camera2Translation = new Translation3d(Units.inchesToMeters(13.5),
                                                                            Units.inchesToMeters(0.0),
                                                                            Units.inchesToMeters(8.0));
   
    public static final String Camera3Name = "right";
    public static final Rotation3d Camera3Rotation = new Rotation3d(0, Units.degreesToRadians(15), 0);
    public static final Translation3d Camera3Translation = new Translation3d(Units.inchesToMeters(13.5),
                                                                            Units.inchesToMeters(0.0),
                                                                            Units.inchesToMeters(8.0));

    public static final Vector<N3>  CameraStdTrans = VecBuilder.fill(2, 2, 4);
    public static final Vector<N3>  CameraStdRot = VecBuilder.fill(0.5, 0.5, 1.0);
  }

}
