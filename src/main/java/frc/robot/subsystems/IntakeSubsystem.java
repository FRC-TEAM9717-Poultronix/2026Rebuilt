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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HangerConstants;

public class IntakeSubsystem extends SubsystemBase {
 private final SparkFlex m_DriveMotor;
  private final SparkFlexConfig m_DriveMotorConfig;
   private final RelativeEncoder m_encoderDriveMotor;
   
private final SparkFlex m_Intake;
 private final SparkFlexConfig m_IntakeConfig;
  private final RelativeEncoder m_encoderIntake;

   private double m_currentVelocity;
   private double m_currentCurrent;

      public IntakeSubsystem() {
       m_DriveMotor = new SparkFlex(Constants.IntakeConstants.CANID_Drive_Motor, MotorType.kBrushless);
         m_DriveMotorConfig = new SparkFlexConfig();
         m_encoderDriveMotor = m_DriveMotor.getEncoder();


       m_Intake = new SparkFlex(Constants.IntakeConstants.CANID_Intake_Right, MotorType.kBrushless);
         m_IntakeConfig = new SparkFlexConfig();
         m_encoderIntake = m_Intake.getEncoder();

       configureMotors();
    }

private void configureMotors() {
       m_DriveMotorConfig.idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(Constants.IntakeConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.IntakeConstants.RAMPRATEINTAKE);

       m_IntakeConfig.idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(Constants.IntakeConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.IntakeConstants.RAMPRATEINTAKE);


       m_DriveMotor.configure(m_DriveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       m_Intake.configure(m_IntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

     }

         public void setIntakeVelocity(double velocity) {
      m_DriveMotor.set(velocity);
      m_Intake.set(velocity);

   }

   public void setIntake(double power) {
       m_DriveMotor.setVoltage(power);
         m_Intake.setVoltage(power);
   }   

   public void stopIntake() {
       m_DriveMotor.set(0);
       m_Intake.set(0);

   }

   @Override
   public void periodic() {
       m_currentVelocity = m_encoderDriveMotor.getVelocity();
       m_currentCurrent  = m_DriveMotor.getOutputCurrent();

     m_currentVelocity = m_encoderIntake.getVelocity();
       m_currentCurrent  = m_Intake.getOutputCurrent();


        // Update SmartDashboard
       updateTelemetry();
   }

    private void updateTelemetry() {
         SmartDashboard.putNumber("Indexer Velocity", m_currentVelocity);
         SmartDashboard.putNumber("Indexer Current", m_currentCurrent);
         SmartDashboard.putNumber("Intake Left Velocity", m_currentVelocity);
         SmartDashboard.putNumber("Intake Left Current", m_currentCurrent);
         SmartDashboard.putNumber("Intake Right Velocity", m_currentVelocity);
         SmartDashboard.putNumber("Intake Right Current", m_currentCurrent);
   
    }


}
    