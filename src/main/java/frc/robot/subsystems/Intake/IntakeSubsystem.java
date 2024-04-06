package frc.robot.subsystems.Intake;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {

     private Spark m_intake;
       
    public IntakeSubsystem() {
    }

    public Command IntakeIn() { 
        m_intake.set(-1);
        return null;
    }

    public Command IntakeOut(){
        m_intake.set(1);
        return null;
    }

    public Command IntakeStop(){
        m_intake.set(0);
        return null;
    }
}