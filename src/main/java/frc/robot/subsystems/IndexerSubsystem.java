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

public class IndexerSubsystem extends SubsystemBase {
    private final SparkMax m_IndexMotor = new SparkMax(Constants.IndexConstants.CANID_Index_Motor, MotorType.kBrushless);

   private final SparkMaxConfig m_IndexMotorConfig;

      private final RelativeEncoder m_encoderIndexMotor;

   private double m_currentVelocity;
   private double m_currentCurrent;

      public IndexerSubsystem() {
       m_IndexMotor = new SparkMax(Constants.IndexConstants.CANID_Index_Motor, MotorType.kBrushless);
         m_IndexMotorConfig = new SparkMaxConfig();
         m_encoderIndexMotor = m_IndexMotor.getEncoder();
       configureMotors();
    }

private void configureMotors() {
       m_IndexMotorConfig.idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(Constants.IndexConstants.MAXCURRENTLIMIT)
            .openLoopRampRate(Constants.IndexConstants.RAMPRATEINDEXER);


       m_IndexMotor.configure(m_IndexMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     }

         public void setIndexerVelocity(double velocity) {
      m_IndexMotor.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity);
   }

   public void setIndexer(double power) {
       m_IndexMotor.setVoltage(power);
   }   

   public void stopIndexer() {
       m_IndexMotor.set(0);
   }

   @Override
   public void periodic() {
       m_currentVelocity = m_encoderIndexMotor.getVelocity();
       m_currentCurrent  = m_IndexMotor.getOutputCurrent();

        // Update SmartDashboard
       updateTelemetry();
   }

    private void updateTelemetry() {
         SmartDashboard.putNumber("Indexer Velocity", m_currentVelocity);
         SmartDashboard.putNumber("Indexer Current", m_currentCurrent);
    }


}
