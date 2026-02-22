package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command{
//  private ShooterSubsystem ;
 // private final DoubleSupplier  m_velocity;

  //public Shoot(ShooterSubsystem shooter, DoubleSupplier velocity)
  {
   //    = shooter; 
 //     m_velocity = velocity;

      addRequirements();
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 //   Double desiredVelocity = m_velocity.getAsDouble();
 //   .seVelocity(desiredVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end");

  //  .stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
