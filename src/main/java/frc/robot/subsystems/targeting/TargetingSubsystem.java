package frc.robot.subsystems.targeting;

import java.lang.StackWalker.Option;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TargetingSubsystem extends SubsystemBase {

    private SwerveSubsystem m_drivebase;
    private PhotonCamera m_camera1;
    private Transform3d m_camera1Pose;
    private final PhotonPoseEstimator m_photonEstimator;

    private int m_selectedFidicial;

    private Optional<Transform2d> m_transformToGoal;  //Transform to goal in Robot Frame
    private Optional<Transform3d> m_transForNearestTarget;  // Transform to nearest targets pose in Robot Frame
    private Optional<Pose2d> m_poseForNearestTarget; // Pose to align with nearest target (rotation is reversed) 
   
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Constructor
    public TargetingSubsystem(SwerveSubsystem drivebase) {
        m_drivebase = drivebase;
        m_camera1 = new PhotonCamera(Constants.VisionConstants.Camera1Name);
        m_camera1Pose = new Transform3d(Constants.VisionConstants.Camera1Translation, Constants.VisionConstants.Camera1Rotation);
        m_photonEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, m_camera1Pose);

        m_selectedFidicial = -1;
    }

    public int getFiducial() {
        return m_selectedFidicial;
    }

    public void setFiducial(int fiducial) {
        m_selectedFidicial = fiducial;
    }

    // Returns Pose of Goal
    public  Optional<Pose2d> getGoalInMapFrame()
    {
        boolean blueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue;

        Pose2d result = blueAlliance ? Constants.goalBlue : Constants.goalRed;
        
        return Optional.of(result);
    }

    // Returns Pose of nearest Target in Robot Frame
    public  Optional<Transform2d> getTransformToGoalInRobotFrame()
    {
        return m_transformToGoal;
    }

    // Returns Pose of nearest Target in Robot Frame
    public  Optional<Transform3d> getTransformToNearestTargetInRobotFrame()
    {
        return m_transForNearestTarget;
    }

    // Returns Pose that aligns with nearest target in Robot Frame
    public  Optional<Pose2d> getPoseForNearestTargetInRobotFrame()
    {
        return m_poseForNearestTarget;
    }

    // Returns Pose of nearest target in Map Frame
    public Optional<Pose2d> getPoseForNearestTargetInMapFrame(Pose2d poseOfRobot)
    {
        if(m_poseForNearestTarget.isPresent())
        {
            Transform2d transform = new Transform2d(poseOfRobot.getTranslation(), poseOfRobot.getRotation());
            return Optional.of(m_poseForNearestTarget.get().transformBy(transform));
        }
        else
        {
            return Optional.empty();
        }
    }



    @Override
    public void periodic() {

        // Get transform to goal
        Pose2d goal = getGoalInMapFrame().get();
        Pose2d robot = m_drivebase.getPose();
        m_transformToGoal = Optional.of(goal.minus(robot));

        // Clear targets
        m_transForNearestTarget = Optional.empty();
        m_poseForNearestTarget = Optional.empty();

        // Read in relevant data from the Camera
        double area = 0.0;
        var results = m_camera1.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    // Find largest AprilTag
                    if((m_selectedFidicial > 0) && (target.getFiducialId() == m_selectedFidicial)) {
                        area = target.area;
                        m_transForNearestTarget = Optional.of(target.getBestCameraToTarget());
                    } else if (target.area > area) {
                        area = target.area;
                        m_transForNearestTarget = Optional.of(target.getBestCameraToTarget());
                }
                }
            }
        }
        
        // Send target to drivetrain
        if(m_transForNearestTarget.isPresent())
        {
            Rotation2d correction = new Rotation2d(Math.PI);
            Rotation2d rotationOfTarget = m_transForNearestTarget.get().getRotation().toRotation2d().plus(correction);
            Pose2d poseOfTarget = new Pose2d(m_transForNearestTarget.get().getX(),m_transForNearestTarget.get().getY(),rotationOfTarget);
            m_poseForNearestTarget = Optional.of(poseOfTarget);
        }
    }

    private void updateTelemetry() {
        SmartDashboard.putBoolean("targeting/nearest_target/visible", m_transForNearestTarget.isPresent());
        SmartDashboard.putNumber("targeting/nearest_target/x", m_transForNearestTarget.get().getX());
        SmartDashboard.putNumber("targeting/nearest_target/y", m_transForNearestTarget.get().getY());
        SmartDashboard.putNumber("targeting/nearest_target/z", m_transForNearestTarget.get().getZ());
        SmartDashboard.putNumber("targeting/nearest_target/roll", m_transForNearestTarget.get().getRotation().getX());
        SmartDashboard.putNumber("targeting/nearest_target/pitch", m_transForNearestTarget.get().getRotation().getY());
        SmartDashboard.putNumber("targeting/nearest_target/yaw", m_transForNearestTarget.get().getRotation().getZ());
    }

}