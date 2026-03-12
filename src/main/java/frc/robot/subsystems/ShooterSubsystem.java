package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.concurrent.TransferQueue;

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
   private final SparkFlex m_shooterL, m_shooterR;
   private final SparkFlex m_indexerUpperL, m_indexerUpperR;
   private final SparkFlex m_indexerLowerL, m_indexerLowerR;

   private final SparkFlexConfig m_shooterLConfig, m_shooterRConfig;
   private final SparkFlexConfig m_indexerUpperLConfig, m_indexerUpperRConfig;
   private final SparkFlexConfig m_indexerLowerLConfig, m_indexerLowerRConfig;

   private final RelativeEncoder m_encoderShooterL, m_encoderShooterR;

   private double m_currentVelocityL, m_currentVelocityR;
   private double m_currentCurrentL, m_currentCurrentR;
    
   public ShooterSubsystem() {
       m_shooterL = new SparkFlex(Constants.ShooterConstants.CANID_SHOOTER_LEFT, MotorType.kBrushless);
       m_indexerUpperL = new SparkFlex(Constants.ShooterConstants.CANID_INDEX_UPPER_LEFT, MotorType.kBrushless);
       m_indexerLowerL = new SparkFlex(Constants.ShooterConstants.CANID_INDEX_LOWER_LEFT, MotorType.kBrushless);

       m_shooterR = new SparkFlex(Constants.ShooterConstants.CANID_SHOOTER_RIGHT, MotorType.kBrushless);
       m_indexerUpperR = new SparkFlex(Constants.ShooterConstants.CANID_INDEX_UPPER_RIGHT, MotorType.kBrushless);
       m_indexerLowerR = new SparkFlex(Constants.ShooterConstants.CANID_INDEX_LOWER_RIGHT, MotorType.kBrushless);

       m_shooterLConfig = new SparkFlexConfig();
       m_shooterRConfig = new SparkFlexConfig();
       m_indexerUpperLConfig = new SparkFlexConfig();
       m_indexerUpperRConfig = new SparkFlexConfig();
       m_indexerLowerLConfig = new SparkFlexConfig();
       m_indexerLowerRConfig = new SparkFlexConfig();

       m_encoderShooterL = m_shooterL.getEncoder();
       m_encoderShooterR = m_shooterR.getEncoder();


       configureMotors();
    }

   private void configureMotors() {
       m_shooterLConfig.idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(Constants.ShooterConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.ShooterConstants.RAMPRATESHOOTER);

        m_shooterRConfig.idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(Constants.ShooterConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.ShooterConstants.RAMPRATESHOOTER);

       m_indexerUpperLConfig.idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(Constants.ShooterConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.ShooterConstants.RAMPRATESHOOTER);

       m_indexerUpperRConfig.idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(Constants.ShooterConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.ShooterConstants.RAMPRATESHOOTER);

      m_indexerLowerLConfig.idleMode(IdleMode.kBrake)
           .inverted(true)
           .smartCurrentLimit(Constants.ShooterConstants.MAXCURRENTLIMIT)
           .openLoopRampRate(Constants.ShooterConstants.RAMPRATESHOOTER);               

       m_indexerLowerRConfig.idleMode(IdleMode.kBrake)
           .inverted(true)
           .smartCurrentLimit(Constants.ShooterConstants.MAXCURRENTLIMIT)
           .openLoopRampRate(Constants.ShooterConstants.RAMPRATESHOOTER);               

       m_shooterLConfig.closedLoop
           .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
           .p(Constants.ShooterConstants.kP_shooter)
           .i(Constants.ShooterConstants.kI_shooter)
           .d(Constants.ShooterConstants.kD_shooter)
           .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
           .kV(Constants.ShooterConstants.kV_shooter)
           .kS(Constants.ShooterConstants.kS_shooter);

      m_shooterRConfig.closedLoop
           .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
           .p(Constants.ShooterConstants.kP_shooter)
           .i(Constants.ShooterConstants.kI_shooter)
           .d(Constants.ShooterConstants.kD_shooter)
           .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
           .kV(Constants.ShooterConstants.kV_shooter)
           .kS(Constants.ShooterConstants.kS_shooter);

       m_shooterL.configure(m_shooterLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       m_indexerUpperL.configure(m_indexerUpperLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       m_indexerLowerL.configure(m_indexerLowerLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

       m_shooterR.configure(m_shooterRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       m_indexerUpperR.configure(m_indexerUpperRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       m_indexerLowerR.configure(m_indexerLowerRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }

   public void setShooter(double velocity) {
      m_shooterL.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity);
      m_shooterR.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity);
   }

   public double getVelocityLeft() {
     return m_encoderShooterL.getVelocity();
   }

      public double getVelocityRight() {
     return m_encoderShooterR.getVelocity();
   }




   public void setIndexerR(double power) {
       m_indexerLowerR.setVoltage(power);
       m_indexerUpperR.setVoltage(power);
   }   

      public void setIndexerL(double power) {
       m_indexerLowerL.setVoltage(power);
       m_indexerUpperL.setVoltage(power);
   }   


   public void stopShooter() {
       m_shooterL.set(0);
       m_shooterR.set(0);
       m_indexerLowerL.set(0);
       m_indexerLowerR.set(0);
       m_indexerUpperL.set(0);
       m_indexerUpperR.set(0);              
   }

   @Override
   public void periodic() {
       m_currentVelocityL = m_encoderShooterL.getVelocity();
       m_currentCurrentL  = m_shooterL.getOutputCurrent();

        // Update SmartDashboard
       updateTelemetry();
   }

   private void updateTelemetry() {
      SmartDashboard.putNumber("shooter/velocityL", m_currentVelocityL);
      SmartDashboard.putNumber("shooter/currentL", m_currentCurrentL);
      SmartDashboard.putNumber("shooter/velocityR", m_currentVelocityR);
      SmartDashboard.putNumber("shooter/currentR", m_currentCurrentR); 
   }

}
