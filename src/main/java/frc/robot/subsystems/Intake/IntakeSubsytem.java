package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

public class IntakeSubsytem extends SubsystemBase {
  public static class Hardware {
    private CANSparkMax intakeMotor;

    public Hardware(CANSparkMax intakeMotor) {
      this.intakeMotor = intakeMotor;
    }
  }

  private CANSparkMax m_intakeMotor;

  private final Measure<Dimensionless> INTAKE_VELOCITY = null;

  /** Creates a new IntakeSubsystem. */
  public void IntakeSubsystem(Hardware intakeHardware, Measure<Dimensionless> intakeVelocity) {
    this.m_intakeMotor = intakeHardware.intakeMotor;
    //INTAKE_VELOCITY = intakeVelocity;
    this.m_intakeMotor = intakeHardware.intakeMotor;

    // Reset motor to defaults
    m_intakeMotor.restoreFactoryDefaults();

    // Set idle mode
    m_intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Initialize hardware devices for intake subsystem
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
 // public static Hardware initializeHardware() {
    //Hardware intakeHardware = new Hardware(
      //new Spark(Constants.IntakeHardware.INTAKE_MOTOR_ID, MotorKind.NEO_VORTEX)
    //);
    //return intakeHardware;
  //}

  // Tells the robot to intake
  private void intake() {
    //m_intakeMotor.set(+INTAKE_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
  }

  // Tells the robot to outtake
  private void outtake() {
    //m_intakeMotor.set(-INTAKE_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
  }

  // Stop the robot
  private void stop() {
    m_intakeMotor.stopMotor();;
  }

  @Override
  public void periodic() {
    //m_intakeMotor.periodic();
  }

  /**
   * Intake game piece from ground
   * @return Command to run the intake motor
   */
  public Command intakeCommand() {
    return startEnd(() -> intake(), () -> stop());
  }

  /**
   * Spit out game piece from intake
   * @return Command to run the intake motor in the reverse direction
   */
  public Command outtakeCommand() {
    return startEnd(() -> outtake(), () -> stop());
  }

  /**
   * Stops the intake
   * @return Command to stop the intake
   */
  public Command stopCommand() {
    return run(() -> stop());
  }
}