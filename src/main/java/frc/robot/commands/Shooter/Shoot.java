package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Shoot extends Command{
   boolean m_leftLocked = false;
 boolean m_rightLocked = false;
 private ShooterSubsystem m_shooter;
 private final DoubleSupplier  m_velocity;
 private IndexerSubsystem m_Indexer;


  public Shoot(ShooterSubsystem shooter, IndexerSubsystem indexer,  DoubleSupplier velocity)
  {
    m_shooter  = shooter; 
    m_Indexer = indexer;
    m_velocity = velocity;

    addRequirements();
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  Double desiredVelocity = m_velocity.getAsDouble();
   m_shooter.setShooter(desiredVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   Double desiredVelocity = m_velocity.getAsDouble();
    m_shooter.setShooter(desiredVelocity);
  if(m_shooter.getVelocityLeft() > desiredVelocity - 100) {
      m_shooter.setIndexerL(0.7);
  } 

  if(m_shooter.getVelocityRight() > desiredVelocity - 100) { 
      m_shooter.setIndexerR(0.7);
        m_Indexer.setIndexer(6);
        m_leftLocked = true;
  
  } else if(m_shooter.getVelocityRight() < 1000 && !m_leftLocked) { 
      m_Indexer.setIndexer(-3);}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("shooter end");

    m_shooter.stopShooter();
    m_Indexer.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
