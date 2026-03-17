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
  private final SparkFlexConfig m_DriveMotorConfigL;
   private final RelativeEncoder m_encoderDriveMotorL;
 private final SparkFlex m_DriveMotorR;
  private final SparkFlexConfig m_DriveMotorConfigR;
   private final RelativeEncoder m_encoderDriveMotorR;
 
   
private final SparkFlex m_Intake;
 private final SparkFlexConfig m_IntakeConfig;
  private final RelativeEncoder m_encoderIntake;

   private double m_currentVelocity;
   private double m_currentCurrent;
   private boolean m_isHomed;

    public IntakeSubsystem() {
       m_DriveMotorL = new SparkFlex(Constants.IntakeConstants.CANID_Drive_MotorL, MotorType.kBrushless);
         m_DriveMotorConfigL = new SparkFlexConfig();
         m_encoderDriveMotorL = m_DriveMotorL.getEncoder();

       m_DriveMotorR = new SparkFlex(Constants.IntakeConstants.CANID_Drive_MotorR, MotorType.kBrushless);
         m_DriveMotorConfigR = new SparkFlexConfig();
         m_encoderDriveMotorR = m_DriveMotorR.getEncoder();

       m_Intake = new SparkFlex(Constants.IntakeConstants.CANID_Intake, MotorType.kBrushless);
         m_IntakeConfig = new SparkFlexConfig();
         m_encoderIntake = m_Intake.getEncoder();

       configureMotors();

       m_isHomed = false;
    }

  private void configureMotors() {
       m_DriveMotorConfigL.idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(Constants.IntakeConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.IntakeConstants.RAMPRATEINTAKE);

        m_DriveMotorConfigR.idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(Constants.IntakeConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.IntakeConstants.RAMPRATEINTAKE);
           
       m_IntakeConfig.idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(Constants.IntakeConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.IntakeConstants.RAMPRATEINTAKE);


       m_DriveMotorL.configure(m_DriveMotorConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       m_DriveMotorR.configure(m_DriveMotorConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        
        SoftLimitConfig newLimit = new SoftLimitConfig();
        newLimit.forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);
        
        m_DriveMotorConfigL.apply(newLimit);
        m_DriveMotorL.configure(m_DriveMotorConfigL, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_DriveMotorConfigR.apply(newLimit);
        m_DriveMotorR.configure(m_DriveMotorConfigR, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


        return true;
    }

    public double getvelocityDrive() {
        return m_encoderDriveMotorL.getVelocity();
    }

    public double getvelocityIntake() {
        return m_encoderIntake.getVelocity();

    }

    public boolean isStalled() {
       boolean stalled = m_DriveMotorL.getWarnings().stall || m_DriveMotorR.getWarnings().stall; 
       return stalled;
        //return m_DriveMotorL.getWarnings().stall;
    }


    public boolean setHome() {
        
        m_isHomed = true;
        
        return true;
    }

   @Override
   public void periodic() {
       m_currentVelocity = m_encoderDriveMotorL.getVelocity();
       m_currentCurrent  = m_DriveMotorL.getOutputCurrent();

       m_currentVelocity = m_encoderDriveMotorR.getVelocity();
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
    