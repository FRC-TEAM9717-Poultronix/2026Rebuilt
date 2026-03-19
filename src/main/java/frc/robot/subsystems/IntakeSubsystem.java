package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
 private final SparkFlex m_DriveMotorL;
  private final SparkFlexConfig m_DriveMotorLConfig;
   private final RelativeEncoder m_encoderDriveLMotor;
 
    private final SparkFlex m_DriveMotorR;
  private final SparkFlexConfig m_DriveMotorRConfig;
   private final RelativeEncoder m_encoderDriveRMotor;

private final SparkFlex m_Intake;
 private final SparkFlexConfig m_IntakeConfig;
  private final RelativeEncoder m_encoderIntake;

   private double m_currentVelocity;
   private double m_currentCurrent;
   private boolean m_isHomed;

    public IntakeSubsystem() {
       m_DriveMotorL = new SparkFlex(Constants.IntakeConstants.CANID_Drive_Motor_L, MotorType.kBrushless);
         m_DriveMotorLConfig = new SparkFlexConfig();
         m_encoderDriveLMotor = m_DriveMotorL.getEncoder();

       m_DriveMotorR = new SparkFlex(Constants.IntakeConstants.CANID_Drive_Motor_R, MotorType.kBrushless);
         m_DriveMotorRConfig = new SparkFlexConfig();
         m_encoderDriveRMotor = m_DriveMotorR.getEncoder();

       m_Intake = new SparkFlex(Constants.IntakeConstants.CANID_Intake, MotorType.kBrushless);
         m_IntakeConfig = new SparkFlexConfig();
         m_encoderIntake = m_Intake.getEncoder();

       configureMotors();

       m_isHomed = false;
    }

  private void configureMotors() {
       m_DriveMotorLConfig.idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(Constants.IntakeConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.IntakeConstants.RAMPRATEINTAKE);

       m_DriveMotorRConfig.idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(Constants.IntakeConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.IntakeConstants.RAMPRATEINTAKE);

            m_IntakeConfig.idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(Constants.IntakeConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.IntakeConstants.RAMPRATEINTAKE);

       m_DriveMotorL.configure(m_DriveMotorLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       m_DriveMotorR.configure(m_DriveMotorRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       m_Intake.configure(m_IntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setIntakePower(double power) {
      m_Intake.set(power);

   }

  public void setDrivePower(double power) {
      m_DriveMotorL.set(power);
      m_DriveMotorR.set(power);


   }

   public void stopDrive() {
       m_DriveMotorL.set(0);
       m_DriveMotorL.clearFaults();
       m_DriveMotorR.set(0);
       m_DriveMotorR.clearFaults();


   }

   public void stopIntake() {
       m_Intake.set(0);

   }

    public boolean disableSoftLimits() {
        m_isHomed = false;
        
        // SoftLimitConfig newLimit = new SoftLimitConfig();
        // newLimit.forwardSoftLimitEnabled(false)
        //         .reverseSoftLimitEnabled(false);
        
        // m_DriveMotorLConfig.apply(newLimit);
        // m_DriveMotorRConfig.apply(newLimit);
        // m_DriveMotorL.configure(m_DriveMotorLConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // m_DriveMotorR.configure(m_DriveMotorRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        return true;
    }

    public double getvelocityDriveL() {
        return m_encoderDriveLMotor.getVelocity();
    }

    public double getvelocityDriveR() {
        return m_encoderDriveRMotor.getVelocity();

    }

    public double getvelocityIntake() {
        return m_encoderIntake.getVelocity();
    }

    public boolean isStalled() {
       if (m_DriveMotorL.getWarnings().stall) {
           return true;
       }
        return m_DriveMotorR.getWarnings().stall;
    }

    public boolean setHome() {
        
        m_isHomed = true;
        
        return true;
    }

   @Override
   public void periodic() {
       m_currentVelocity = m_encoderDriveLMotor.getVelocity();
       m_currentCurrent  = m_DriveMotorL.getOutputCurrent();

       m_currentVelocity = m_encoderDriveRMotor.getVelocity();
       m_currentCurrent  = m_DriveMotorR.getOutputCurrent();

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
    