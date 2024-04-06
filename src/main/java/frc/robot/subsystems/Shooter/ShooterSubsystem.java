package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;


public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax m_shooterOne;
  private CANSparkMax m_shooterTwo;
       
    public ShooterSubsystem() {
    }

    public Command ShooterStart() { 
        m_shooterOne.set(-1);
        m_shooterTwo.set(-1);
        return null;
    }


    public Command ShooterStop(){
       m_shooterOne.set(0);
        m_shooterTwo.set(0);
        return null;
    }
} 