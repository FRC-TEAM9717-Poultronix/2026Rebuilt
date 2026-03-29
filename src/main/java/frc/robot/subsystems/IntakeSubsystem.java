package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkBase.ControlType;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkFlex m_Intake;
    private final SparkFlex m_DriveL, m_DriveR;
    private final SparkFlexConfig m_IntakeConfig;
    private final SparkFlexConfig m_DriveLConfig, m_DriveRConfig;

    private final RelativeEncoder m_encoderDriveL, m_encoderDriveR;

    private double m_currentSetpointL;
    private double m_curentPositionL;
    private double m_currentCurrentL, m_currentCurrentR;
    private boolean m_isHomed;
    
   public IntakeSubsystem() {
       m_Intake = new SparkFlex(Constants.IntakeConstants.CANID_Intake, MotorType.kBrushless);
       m_DriveR = new SparkFlex(Constants.IntakeConstants.CANID_DriveR, MotorType.kBrushless);
       m_DriveL = new SparkFlex(Constants.IntakeConstants.CANID_DriveL, MotorType.kBrushless);

       m_IntakeConfig = new SparkFlexConfig();
       m_DriveRConfig = new SparkFlexConfig();
       m_DriveLConfig = new SparkFlexConfig();

       m_encoderDriveL = m_DriveL.getEncoder();
       m_encoderDriveR = m_DriveR.getEncoder();


       configureMotors();
        m_isHomed = false;
    }

   private void configureMotors() {
       m_DriveLConfig.idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(Constants.IntakeConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.IntakeConstants.RAMPRATEINTAKE);

        m_DriveRConfig.idleMode(IdleMode.kBrake)
            .inverted(true)
             .follow(m_DriveL, true)
            .smartCurrentLimit(Constants.IntakeConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.IntakeConstants.RAMPRATEINTAKE);

        m_IntakeConfig.idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(Constants.IntakeConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.IntakeConstants.RAMPRATEINTAKE);


       m_DriveLConfig.closedLoop
           .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
           .p(Constants.IntakeConstants.kP_Drive)
           .i(Constants.IntakeConstants.kI_Drive)
           .d(Constants.IntakeConstants.kD_Drive)
           .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
           .kV(Constants.IntakeConstants.kV_Drive)
           .kS(Constants.IntakeConstants.kS_Drive);

       m_DriveL.configure(m_DriveLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       m_DriveR.configure(m_DriveRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       m_Intake.configure(m_IntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void setIntakePower(double power) {
      m_Intake.set(power);
   }

  public void setDrivePower(double power) {
      m_DriveL.set(power);
   }

   public void setDrivePosition(double position) {
       if(m_isHomed) m_DriveL.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
   }

   public void stopIntake() {
       m_Intake.set(0);
   }
    public boolean isStalled() {
        Boolean result = m_encoderDriveL.getVelocity() < 1;
        
        return result;
    }
    public boolean setHome() {
        
        m_isHomed = true;
        
        return true;
    }

    public Command agitateCommand()
    {
        return runOnce(() -> setIntakePower(0.30))
            .andThen(
                Commands.sequence(
                    runOnce(() -> setDrivePosition(2.25)),
                    Commands.waitUntil(() -> Math.abs(m_curentPositionL - 2.5) < 0.1),
                    runOnce(() -> setDrivePosition(3.0)),
                    Commands.waitUntil(() -> Math.abs(m_curentPositionL - 3.0) < 0.1)
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                setIntakePower(0.0);
                setDrivePower(0.0);
            });
    }

   @Override
   public void periodic() {
       m_currentSetpointL = m_DriveL.getClosedLoopController().getSetpoint();
       m_curentPositionL = m_encoderDriveL.getPosition();
       m_currentCurrentR  = m_DriveR.getOutputCurrent();       

        // Update SmartDashboard
       updateTelemetry();
   }

   private void updateTelemetry() {
      SmartDashboard.putNumber("Intake/setpointL", m_currentSetpointL);
      SmartDashboard.putNumber("Intake/positionL", m_curentPositionL);
      SmartDashboard.putNumber("Intake/currentL", m_currentCurrentL);
      SmartDashboard.putNumber("shooter/currentR", m_currentCurrentR); 
   }
   

}
