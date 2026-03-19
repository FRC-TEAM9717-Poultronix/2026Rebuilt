package frc.robot.commands.AutoCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;



public class AutoShoot extends Command{
 private ShooterSubsystem m_shooter;
 private IndexerSubsystem m_Indexer;
 private final DoubleSupplier  m_velocity;
 boolean m_leftLocked = false;
 boolean m_rightLocked = false;
  private double m_startTime;
  private double currentTime;


  public AutoShoot(ShooterSubsystem shooter, IndexerSubsystem indexer,  DoubleSupplier velocity)
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
    m_startTime =  System.currentTimeMillis();

   

   m_leftLocked = false;
   m_rightLocked = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   Double desiredVelocity = m_velocity.getAsDouble();
    m_shooter.setShooter(desiredVelocity);

     currentTime = System.currentTimeMillis();

  if(m_shooter.getVelocityLeft() > desiredVelocity - 500) {
      m_shooter.setIndexerL(0.7);
      m_leftLocked = true;
  } else if(!m_leftLocked) {
      m_shooter.setIndexerL(-0.25);
  }

  if(m_shooter.getVelocityRight() > desiredVelocity - 500) { 
      m_shooter.setIndexerR(0.7);  
      m_rightLocked = true;
  } else if(!m_rightLocked) { 
      m_shooter.setIndexerR(-0.25);
  }

  m_Indexer.setIndexer(6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("shooter end");

    m_shooter.stopShooter();
    m_Indexer.stopIndexer();
  }

  // Returns true when the command should end.

  //in auto, we need this to return true after a certain amount of time
  @Override
  public boolean isFinished() {
      if(currentTime > m_startTime + 5000)
        {
            return true;
        }
    return false;}
}
