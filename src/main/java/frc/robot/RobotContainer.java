// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
//import frc.robot.subsystems.Shooter.ShooterSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
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

  
  
    
 



//INTAKE intake = new INTAKE();
  /*
  private static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem(
    IntakeSubsystem.initializeHardware(),
    Constants.Intake.INTAKE_VELOCITY
  );
  */







  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);
  XboxController controlXbox = new XboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  

  public RobotContainer()
  {
    drivebase.setupPathPlanner();
    
    NamedCommands.registerCommand("IntakeIn", intakeSubsystem.IntakeIn());
    NamedCommands.registerCommand("IntakeOut", intakeSubsystem.IntakeOut());
    NamedCommands.registerCommand("IntakeStop", intakeSubsystem.IntakeStop());
    NamedCommands.registerCommand("ShooterStart", shooterSubsystem.ShooterStart());
    NamedCommands.registerCommand("ShooterStop", shooterSubsystem.ShooterStop());
    NamedCommands.registerCommand("ArmUp", armSubsystem.ArmUp());
    NamedCommands.registerCommand("ArmDown", armSubsystem.ArmDown());
    NamedCommands.registerCommand("ArmStop", armSubsystem.ArmStop());

    
    autoChooser.addOption("Blue Mid", AutoBuilder.buildAuto("Blue_Mid"));
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Register Named Commands
    
   
   
   
    





    
    // ...

    // Build an auto chooser. This will use Commands.none() as the default option.
    

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);








    // Configure the trigger bindings
    configureBindings();
    ;

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox::getYButtonPressed,
                                                                   driverXbox::getAButtonPressed,
                                                                   driverXbox::getXButtonPressed,
                                                                   driverXbox::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.75);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
    









    //IntakeSubsystem closedIntakeSubsystem = new IntakeSubsystem();

    

    //(Constants.NamedCommands.INTAKE_COMMAND_NAME, autoIntakeCommand().withTimeout(7));    
    
    
    
    
    
    
    
    
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    new JoystickButton(driverXbox, 2).whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));

    
    //new JoystickButton(driverXbox, 6).whileFalse(new InstantCommand(() -> drivebase.maximumSpeed = 1.5));
    /*
        if (driverXbox.getLeftBumper()) {
      drivebase.maximumSpeed = 0.3;
    } else if (driverXbox.getRightBumper()) {
      drivebase.maximumSpeed = 1.5;
    } else {
      drivebase.maximumSpeed = 1;
    }
    */
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));






    //new JoystickButton(driverXbox, 4).onTrue((new InstantCommand(INTAKE_VELOCITY)));










  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void robotInit() {
    
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

  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    double x = tx.getDouble(0.0);
    intitialX = x;
  }

  public Command drive0 = drivebase.driveToPose(new Pose2d(new Translation2d(3,0), new Rotation2d(0)));
  public Command reset = new InstantCommand(() -> drivebase.resetOdometry(new Pose2d(new Translation2d(0,0), new Rotation2d(0))));
  public Command driveStop = drivebase.driveCommand(() -> 0, () -> 0, () -> 0);
  public Command drive1 = drivebase.driveCommandTimed(() -> -0.6, () -> 0.6, () -> 0);
  public Command drive2 = drivebase.driveCommandTimed(() -> -0.5, () -> 0, () -> 0);
  public Command drive3 = drivebase.driveCommand(() -> -0.6, () -> 0, () -> 0);



  public void autoTest() {
    double timePeriodic = Timer.getFPGATimestamp();
    double time = timePeriodic - startTime;
    
    //drivebase.drive(new ChassisSpeeds(-5, 0, 0));
    drivebase.drive(new Translation2d(2,0), 0, true);
    
    //drivebase.driveToPose(new Pose2d(1,0, new Rotation2d()));
    /*
    if (time < 2) {
      drivebase.drive(new ChassisSpeeds(1, 0, 0));
    }
    
    if (time > 2.5 && time < 3.5) {
      m_armLeftMotor.set(-0.1);
      m_armRightMotor.set(0.1);
    }

    if (time > 4 && time < 5) {
      m_intake.set(-1);
      m_shooterOne.set(-0.3);
      m_shooterTwo.set(-0.3);
    }

    if (time > 5.5 && time < 6.5) {
      m_armLeftMotor.set(0.4);
      m_armRightMotor.set(-0.4);
    }

    if (time > 7 && time < 9) {
      m_intake.set(0);
      m_shooterOne.set(0);
      m_shooterTwo.set(0);
      drivebase.drive(new ChassisSpeeds(-1, 0, 0));
    }
    if (time > 9.5 && time < 13.5) {
      drivebase.drive(new ChassisSpeeds(0,2,0));
    }
    */
    /*
    if (time > 8) {
      m_intake.set(0);
      m_shooterOne.set(0);
      m_shooterTwo.set(0);
      drivebase.drive(new ChassisSpeeds(0, 0, 0));
    }
    */
  }

  public void autonomousPeriodic() {
    double timePeriodic = Timer.getFPGATimestamp();
    double time = timePeriodic - startTime;
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
    double target = y/2;

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
    //double targetPosition = Math.atan(goal/(distanceFromLimelightToGoalInches + distanceFromLimelightToArmInches));
    targetPosition = 0;
    double position = .902 - encoder.getAbsolutePosition();

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
    m_pidController.setReference(target, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", y);
    SmartDashboard.putNumber("rotations", y);
    //SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
    
    if (time < 2) {
      m_armLeftMotor.set(0.2);
      m_armRightMotor.set(-0.2); 
    }
    //Simple Auto
    /*if (time > 4 && time < 8 ) {
      m_shooterOne.set(-0.8);
      m_shooterTwo.set(-0.8);
      m_armLeftMotor.set(-(0.13 - position)/(.4));
      m_armRightMotor.set((0.13 - position)/(.4));
    }
    if (time > 7 && time < 8) {
      m_intake.set(-1);
    }
    
    if (time > 8 && time < 9) {
      m_shooterOne.set(0);
      m_shooterTwo.set(0);
      m_armLeftMotor.set(0);
      m_armRightMotor.set(0);
      m_intake.set(0);
    }*/
    if (time > 4 && time < 6) {
      //frontLeft.set(-.2);
      //rearLeft.set(-.2);
      //frontRight.set(-.2);
      //rearRight.set(-.2);
      m_intake.set(-1);
      m_armLeftMotor.set(-(0.01 - position)/(.4));
      m_armRightMotor.set((0.01 - position)/(.4));
    }
    
    if (time > 11) {//CHANGE
      m_shooterOne.set(0);
      m_shooterTwo.set(0);
      m_armLeftMotor.set(0);
      m_armRightMotor.set(0);
      m_intake.set(0);
      //m_robotDrive.driveCartesian(0,0,0);
    }
    
    //Arm math
    if (targetPosition < 0.001) {
      targetPosition = 0.001;
    }
    if (targetPosition > .28) {
      targetPosition = .28;
    }
    if (position < 0.01 && targetPosition < 0.002) {
      m_armLeftMotor.set(0);
      m_armRightMotor.set(0);
    }
    
    //Time Based Auto    
    /*
    if (time < 2 && (x < (-1/360) || x > (1/360))) {
      m_robotDrive.driveCartesian(0, 0, -x/(5/360));
    }
    if (time > 2 && time < 4) {
      m_shooterOne.set(-0.8);
      m_shooterTwo.set(-0.8);
      m_armLeftMotor.set((angle - encoder.getAbsolutePosition())/angle);
      m_armRightMotor.set((angle - encoder.getAbsolutePosition())/angle);
    }
    if (time > 3 && time < 4) {
      m_intake.set(-1);
    }
    if (time > 4 && time < 6) {
      m_shooterOne.set(0);
      m_shooterTwo.set(0);
      m_armLeftMotor.set(0);
      m_armRightMotor.set(0);
      m_intake.set(0);
      m_robotDrive.driveCartesian(0,0, (intitialX - x)/(5/360));
    }
    if (time > 6 && time < 7) {
      m_robotDrive.driveCartesian(0,0.8,0);
      m_intake.set(-1);
    }
    if (time > 7 && time < 8) {
      m_robotDrive.driveCartesian(0,-0.8,0);
      m_intake.set(0);
    }
    if (time > 7.5 && time < 7.8) { // reverse intate to pull note away from shooter
      m_intake.set(0.5);
    }
    if (time > 8 && time < 10  && (x < (-1/360) || x > (1/360))) {
      m_robotDrive.driveCartesian(0,0,(1/360 - x)/(5/360));
    }
    if (time > 10 && time < 12) {
      m_shooterOne.set(-0.8);
      m_shooterTwo.set(-0.8);
      m_armLeftMotor.set((angle - encoder.getAbsolutePosition())/angle);
      m_armRightMotor.set((angle - encoder.getAbsolutePosition())/angle);
    }
    if (time > 11 && time < 12) {
      m_intake.set(-1);
    }
    if (time > 12) {
      m_shooterOne.set(0);
      m_shooterTwo.set(0);
      m_armLeftMotor.set(0);
      m_armRightMotor.set(0);
      m_intake.set(0);
    }
    */
  }

  public void teleopInit() {
    targetPosition = 0.001;
  }

  public void teleopPeriodic () {
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
    
    
    m_armLeftMotor.set(controlXbox.getLeftY()*.4);
    m_armRightMotor.set(-controlXbox.getLeftY()*.4);

   



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
    if(controlXbox.getAButton()){
      m_intake.set(1);
    }else if(controlXbox.getBButton()){
      m_intake.set(-1);
    }else{
      m_intake.set(0);
    }
    //shooter in and out
    if(controlXbox.getYButton()){
      m_shooterOne.set(-0.5);
      m_shooterTwo.set(-0.5);
    } else if (controlXbox.getXButton()) {
      m_shooterOne.set(-1);
      m_shooterTwo.set(-1);
    }
    else{
      m_shooterOne.set(0.0);
        m_shooterTwo.set(0.0);

    }
  }

  public Command getAutoCommand(){
    return autoChooser.getSelected();
  }
}