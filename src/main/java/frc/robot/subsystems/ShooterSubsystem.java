package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HangerConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex m_shooter;
    private final SparkFlex m_indexerUpper;
    private final SparkFlex m_indexerLower;

    private final SparkFlexConfig m_shooterConfig;
    private final SparkFlexConfig m_indexerUpperConfig;
    private final SparkFlexConfig m_indexerLowerConfig;

    private final RelativeEncoder m_encoderShooter;

    private double m_currentVelocity;
    private double m_currentCurrent;
    
    public ShooterSubsystem() {
        m_shooter = new SparkFlex(Constants.ShooterConstants.CANID_SHOOTER, MotorType.kBrushless);
        m_indexerUpper = new SparkFlex(Constants.ShooterConstants.CANID_INDEX_UPPER, MotorType.kBrushless);
        m_indexerLower = new SparkFlex(Constants.ShooterConstants.CANID_INDEX_LOWER, MotorType.kBrushless);

         m_shooterConfig = new SparkFlexConfig();
         m_indexerUpperConfig = new SparkFlexConfig();
         m_indexerLowerConfig = new SparkFlexConfig();

        m_encoderShooter = m_shooter.getEncoder();

        configureMotors();
    }

    private void configureMotors() {
        m_shooterConfig.idleMode(IdleMode.kCoast)
                    .inverted(true)
                    .smartCurrentLimit(Constants.ShooterConstants.MAXCURRENTLIMIT)
                    .openLoopRampRate(Constants.ShooterConstants.RAMPRATESHOOTER);
        
        m_indexerUpperConfig.idleMode(IdleMode.kBrake)
                    .inverted(true)
                    .smartCurrentLimit(Constants.ShooterConstants.MAXCURRENTLIMIT)
                    .openLoopRampRate(Constants.ShooterConstants.RAMPRATESHOOTER);

        m_indexerUpperConfig.idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(Constants.ShooterConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.ShooterConstants.RAMPRATESHOOTER);               

        m_shooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Constants.ShooterConstants.kP_shooter)
            .i(Constants.ShooterConstants.kI_shooter)
            .d(Constants.ShooterConstants.kD_shooter)
            .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
            .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

        m_shooter.configure(m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_indexerUpper.configure(m_indexerUpperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_indexerLower.configure(m_indexerLowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void seVelocity(double velocity) {
       m_shooter.set(velocity);
    }
    
    public void stopShooter() {
        m_shooter.set(0);
    }

    @Override
    public void periodic() {
        m_currentVelocity = m_encoderShooter.getVelocity();
        m_currentCurrent  = m_shooter.getOutputCurrent();

        // Update SmartDashboard
        updateTelemetry();
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("hanger/velocity", m_currentVelocity);
        SmartDashboard.putNumber("hanger/motor_current", m_currentCurrent);
    }

}
