package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase {
   
    private CANSparkMax m_armLeftMotor;
    private CANSparkMax m_armRightMotor;
       
    public ArmSubsystem() {
    }

    
    public Command ArmUp() { 
        return this.runOnce(() -> {
           m_armLeftMotor.set(0.2);
           m_armRightMotor.set(-0.2);
        }           
        );
    }

     public Command ArmDown() { 
        return this.runOnce(() -> {
           m_armLeftMotor.set(-0.2);
           m_armRightMotor.set(0.2);
        }           
        );
    }  
  
      public Command ArmStop(){
         return this.runOnce(() -> {
          m_armLeftMotor.set(0.0);
           m_armRightMotor.set(0.0);
        }           
        );
    }  
}