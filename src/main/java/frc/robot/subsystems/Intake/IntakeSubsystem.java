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
        return this.runOnce(() -> m_intake.set(-1.0));
    }

    public Command IntakeOut(){
        return this.runOnce(() -> m_intake.set(1.0));
    }

    public Command IntakeStop(){
        return this.runOnce(() -> m_intake.set(0.0));
    }

}