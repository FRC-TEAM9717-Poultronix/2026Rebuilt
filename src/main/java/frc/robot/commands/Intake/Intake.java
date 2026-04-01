package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class Intake extends Command{
 private IntakeSubsystem m_intake;
 private final DoubleSupplier  m_velocity;

  public Intake (IntakeSubsystem Intake, DoubleSupplier velocity)
  {
    m_intake  = Intake; 
    m_velocity = velocity;

    addRequirements();
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   Double desiredVelocity = m_velocity.getAsDouble();
   m_intake.setIntakeVelocity(desiredVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("intake end");

    m_intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
