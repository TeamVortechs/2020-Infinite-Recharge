/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private int ballCount;
  private String m_autoSelected;
  private Boolean activated;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private AHRS navx;
  private AnalogInput ballbeam1, ballbeam2, ballbeam3, ballbeam4, ballbeam5, ballbeam6, ballbeam7, ballbeam8, ballbeam9, ballbeam10;
  private XboxController controllerdriver, controlleroperator;
  private Spark backRight, frontRight, backLeft, frontLeft, intake, belt1, belt2, belt3, belt4, loader;
  private SpeedControllerGroup leftMotors, rightMotors;
  private DifferentialDrive drive;
  private Encoder leftEncoder, rightEncoder;
  private NetworkTable table;
  private PWMTalonSRX arm;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // NavX sensor
    navx = new AHRS(I2C.Port.kMXP);

    // Magazine sensors
    ballCount = 0;
    ballbeam1 = new AnalogInput(0);
    ballbeam2 = new AnalogInput(1);
    ballbeam3 = new AnalogInput(2);
    ballbeam4 = new AnalogInput(3);
    ballbeam5 = new AnalogInput(4);
    ballbeam6 = new AnalogInput(5);
    ballbeam7 = new AnalogInput(6);
    ballbeam8 = new AnalogInput(7);
    ballbeam9 = new AnalogInput(8);
    ballbeam10 = new AnalogInput(9);

    // Xbox Controllers
    controllerdriver = new XboxController(0);
    controlleroperator = new XboxController(1);

    // Drive motors
    backRight = new Spark(0);
    frontRight = new Spark(1);
    backLeft = new Spark(2);
    frontLeft = new Spark(3);

    leftMotors = new SpeedControllerGroup(backLeft, frontLeft);
    rightMotors = new SpeedControllerGroup(backRight, frontRight);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    leftEncoder = new Encoder(5, 6, false, Encoder.EncodingType.k2X);
    rightEncoder = new Encoder(3, 4, true, Encoder.EncodingType.k2X);
    leftEncoder.setDistancePerPulse(1.0/256);
    rightEncoder.setDistancePerPulse(1.0/256);

    table = NetworkTableInstance.getDefault().getTable("limelight");

    // Intake motors
    intake = new Spark(4);
    activated = false;

    // Belt motors in the magazine
     belt1 = new Spark(5);
     belt2 = new Spark(6);
     belt3 = new Spark(7);
     belt4 = new Spark(8);
     loader = new Spark(9);

    // Arm motor
    // arm = new PWMTalonSRX(0);
  }

  public void setDriveWheels(double left, double right)
  {
    // backLeft.set(left);
    // frontLeft.set(left);
    // backRight.set(-right);
    // frontRight.set(-right);
  }

  public void setIntake(double intakeSpeed)
  {
    intake.set(intakeSpeed);
  }

  public void setMagazineBelts(double magazineSpeed)
  {
    belt1.set(magazineSpeed);
    belt2.set(magazineSpeed);
    belt3.set(magazineSpeed);
    belt4.set(magazineSpeed);
    loader.set(magazineSpeed);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);


  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    // double driveSpeed = Math.pow(controllerdriver.getY(GenericHID.Hand.kLeft), 3);
    // double direction = controllerdriver.getX(GenericHID.Hand.kRight) * 0.66;
    double driveSpeed = controllerdriver.getY(GenericHID.Hand.kLeft);
    double direction = controllerdriver.getX(GenericHID.Hand.kRight);
    //int pov = controllerdriver.getPOV(0);

    drive.arcadeDrive(driveSpeed, direction, true);
    System.out.println("Left: " + leftEncoder.getDistance() + " Right: " + rightEncoder.getDistance());
    // setDriveWheels(driveSpeed - direction, driveSpeed + direction);

    NetworkTableEntry tx = table.getEntry("tx");
    System.out.println("Limelight: " + tx);

    if (activated = false)
    {
      if (controllerdriver.getYButton())
      {
        intake.set(1);
        activated = true;
      } 
    }
    else if (activated = true)
    {
      if (controllerdriver.getYButton())
      {
        intake.set(0);
        activated = false;
      }
    }

    if (ballCount == 0)
    {

    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  {

  }

}






