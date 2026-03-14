package frc.robot.commands.Extend;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Intake.Intake;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * A command to rezero the elevator
 */
public class Extend extends Command {
  enum State {start, start_out, homing, finished};

  private final IntakeSubsystem m_DriveMotor;
  private double m_startTime;
  private State m_state;

  // Constructor
  public Extend(IntakeSubsystem Drive)
  {
    m_DriveMotor = Drive;
  
    addRequirements(m_DriveMotor);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_state = State.start;
    m_startTime =  System.currentTimeMillis();
    m_DriveMotor.disableSoftLimits();
  }

  // Called every cycle while command is active
  @Override
  public void execute()
  {
    double currentTime = System.currentTimeMillis();
    
    switch (m_state) {
      case start:
        m_DriveMotor.setDrivePower(-0.1);
        m_state = State.start_out;
        break;
      case start_out:
      if(currentTime > m_startTime + 100)
        {
          m_state = State.homing;
        }
        break;
      case homing:
        if(m_DriveMotor.isStalled())
        {
          m_DriveMotor.setHome();
          m_state = State.finished;
        }
        break;
      default:
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return m_state == State.finished;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_DriveMotor.stopDrive();;
    System.out.println("Extend Ended!");
  }

}