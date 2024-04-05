// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot instance;
  private Command m_autonomousCommand;
  private CANcoder absoluteEncoder;

  private Constants m_Constants = new Constants();
   //Arm
  private CANSparkMax m_armLeftMotor;
  private CANSparkMax m_armRightMotor;
  //INTake and shoooter
  private Spark m_intake;
  private CANSparkMax m_shooterOne;
  private CANSparkMax m_shooterTwo;
  private  SparkPIDController m_pidController;
  //Time
  private final Timer timer = new Timer();
  private double startTime;
  //Absolute Encoder
  private DutyCycleEncoder encoder;
  //variables
  private double intitialX = 0;
  private double targetPosition;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
//multicam
  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;

  XboxController conXbox = new XboxController(1);


  private RobotContainer m_robotContainer;
  //private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    absoluteEncoder = new CANcoder(11);
   CANcoderConfigurator cfg = absoluteEncoder.getConfigurator();
   cfg.apply(new CANcoderConfiguration());
   MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
   cfg.refresh(magnetSensorConfiguration);
   cfg.apply(magnetSensorConfiguration
                  .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
    
    m_armLeftMotor = new CANSparkMax(m_Constants.leftMotorID, MotorType.kBrushless);
    m_armRightMotor = new CANSparkMax(m_Constants.rightMotorID, MotorType.kBrushless);
    m_shooterOne = new CANSparkMax(m_Constants.shooterOneID,MotorType.kBrushless);
    m_shooterTwo = new CANSparkMax(m_Constants.shooterTwoID, MotorType.kBrushless);
    m_intake = new Spark(m_Constants.PWMSparkIntakeChannel);
    targetPosition = 0;
    encoder = new DutyCycleEncoder(new DigitalInput(0));
    m_armRightMotor.setInverted(true);
    //arms
    m_armLeftMotor.restoreFactoryDefaults();
    m_armRightMotor.restoreFactoryDefaults();
    m_armLeftMotor.setIdleMode(IdleMode.kBrake);
    m_armRightMotor.setIdleMode(IdleMode.kBrake);
   // m_armFollowMotor.follow(m_armLeadMotor);
    m_pidController = m_armRightMotor.getPIDController();
    m_pidController = m_armLeftMotor.getPIDController();
    m_armLeftMotor.setSmartCurrentLimit(40);
    m_armRightMotor.setSmartCurrentLimit(40);

    kP = 0.1; 
     kI = 1e-4;
     kD = 1; 
     kIz = 0; 
     kFF = 0; 
     kMaxOutput = 1; 
     kMinOutput = -1;
 
     // set PID coefficients
     m_pidController.setP(kP);
     m_pidController.setI(kI);
     m_pidController.setD(kD);
     m_pidController.setIZone(kIz);
     m_pidController.setFF(kFF);
     m_pidController.setOutputRange(kMinOutput, kMaxOutput);
 
     // display PID coefficients on SmartDashboard
     SmartDashboard.putNumber("P Gain", kP);
     SmartDashboard.putNumber("I Gain", kI);
     SmartDashboard.putNumber("D Gain", kD);
     SmartDashboard.putNumber("I Zone", kIz);
     SmartDashboard.putNumber("Feed Forward", kFF);
     SmartDashboard.putNumber("Max Output", kMaxOutput);
     SmartDashboard.putNumber("Min Output", kMinOutput);
     SmartDashboard.putNumber("Set Rotations", 0);
     //CameraServer.startAutomaticCapture();
     
    //video server
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    StatusSignal<Double> angle = absoluteEncoder.getAbsolutePosition().waitForUpdate(0.1);

    //System.out.println("Absolute Encoder Angle: " + angle.getValue());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    startTime = Timer.getFPGATimestamp();
    m_robotContainer.autonomousInit();

     //schedule the autonomous command (example)
    //if (m_autonomousCommand != null)
    //{
      //m_autonomousCommand.schedule();
      
      new WaitCommand(9).andThen(
        m_robotContainer.drive2).andThen(
          new WaitCommand(3)).andThen(
            m_robotContainer.driveStop).andThen(
              new WaitCommand(3)).andThen(
                new InstantCommand(() -> m_shooterOne.set(-0.3)), 
                new InstantCommand(() -> m_shooterTwo.set(-0.3))).schedule();
      
      //m_robotContainer.reset.schedule();
      //m_robotContainer.drive1.schedule();
      //new WaitCommand(5).schedule();
      //m_robotContainer.drive1.schedule();
      //new WaitUntilCommand(null)
    //}
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
    double timePeriodic = Timer.getFPGATimestamp();
    double time = timePeriodic - startTime;

    if(time < 2) {
      m_armLeftMotor.set(0.2);
      m_armRightMotor.set(-0.2);
    }
    if (time > 2 && time < 4) {
      m_shooterOne.set(-0.8);
      m_shooterTwo.set(-0.8);
      m_armLeftMotor.set(0);
      m_armRightMotor.set(0);
    }
    
    if (time > 3 && time < 4) {
      m_intake.set(-1);
    }
    if (time > 4) {
      m_shooterOne.set(0);
      m_shooterTwo.set(0);
      m_intake.set(0);
    }
    //m_robotContainer.autoTest();
    /* 
    if (time < 2) {
      m_shooterOne.set(-0.8);
      m_shooterTwo.set(-0.8);
    }
    
    if (time > 1 && time < 2) {
      m_intake.set(-1);
    }
    
    if (time > 2) {
      m_shooterOne.set(0);
      m_shooterTwo.set(0);
      m_intake.set(0);
    }
    */
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);

    //m_robotContainer.teleopInit();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {

    //m_robotContainer.teleopPeriodic();
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", y);
    SmartDashboard.putNumber("Abs", encoder.getAbsolutePosition());
    SmartDashboard.putNumber("targetPosition", targetPosition);
    double target = y/2;
    //60 degrees
    // Aiming
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    double distanceFromLimelightToArmInches = 40;
    double goal = 72;
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 55.0; //ADD MOUNTED ANGLE
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 12.0; 
    // distance from the target to the floor
    double goalHeightInches = 50.0; 
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    // angle to goal
    double angle = Math.atan(goal/(distanceFromLimelightToGoalInches + distanceFromLimelightToArmInches));

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    //m_pidController.setReference(target, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", y);
    SmartDashboard.putNumber("rotations", y);
    //SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
    // Use the joystick Y axis for forward movement, X axis for lateral
    // movement, and Z axis for rotation.
    
    //should drive two arm motors together?
     //if(m_stick2.getRightBumper()){
     // m_pidController.setReference(y, CANSparkMax.ControlType.kPosition);
    //}else{ m_armLeadMotor.set((m_stick2.getLeftY()/2));
    // m_armFollowMotor.set((-m_stick2.getLeftY()/2));}
    //if for intake in and out
    
    //lift values divide by 360
    // ground - 325.6
    // up - 244
    // amp - 226

    double position = .902 - encoder.getAbsolutePosition();
    
    
    m_armLeftMotor.set(conXbox.getLeftY()*.4);
    m_armRightMotor.set(-conXbox.getLeftY()*.4);
    
    //video switch
    /* 
    if (m_stick2.getRightStickButtonPressed()){
      System.out.println("Camera 2");
      server.setSource(camera2);
    }else if (m_stick2.getRightStickButtonReleased()) {
      System.out.println("Camera 1");
      server.setSource(camera1);
    }
     
    if (Math.abs(m_stick2.getLeftY()) > .1) {
      targetPosition -= m_stick2.getLeftY()/100;
    }

    if (targetPosition < 0.001) {
      targetPosition = 0.001;
    }
    if (targetPosition > .31) {
      targetPosition = .31;
    }
      
    if (position < 0.04 && targetPosition < 0.002) {
      m_armLeftMotor.set(0);
      m_armRightMotor.set(0);
    } else {
      m_armLeftMotor.set(-(targetPosition - position)/(.4));
      m_armRightMotor.set((targetPosition - position)/(.4));
    }
    if (m_stick2.getLeftBumper()) {
      targetPosition = 0.14;
    }

    //m_armLeftMotor.set(1);//REMOVE
    //m_armRightMotor.set(1);//REMOVE
    */
    if(conXbox.getAButton()){
      m_intake.set(1);
    }else if(conXbox.getBButton()){
      m_intake.set(-1);
    }else{
      m_intake.set(0);
    }
    //shooter in and out
    if (conXbox.getYButton()){
      m_shooterOne.set(-0.5);
      m_shooterTwo.set(-0.5);
    } else if (conXbox.getXButton()) {
      m_shooterOne.set(-1);
      m_shooterTwo.set(-1);
    } else {
      m_shooterOne.set(0.0);
        m_shooterTwo.set(0.0);

    }
    
    

  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}