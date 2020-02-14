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
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{

  private static final String autoGo4Feet = "autoGo4Feet";
  private static final String autoOutAndBack = "autoOutAndBack";
  private static final String autoTurn90 = "autoTurn90";

  private static final double shootDistance = 30.0;
  private static final double shootSpeed = 0.5;

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private boolean align, approach, shoot;
  private AHRS navx;
  private AnalogInput ballbeam1, ballbeam2, ballbeam3, ballbeam4, ballbeam5, ballbeam6, ballbeam7, ballbeam8, ballbeam9, ballbeam10;
  private XboxController controllerdriver, controlleroperator;
  private Spark backRight, frontRight, backLeft, frontLeft, intake, belt1, belt2, belt3, belt4, loader, colorMotor;
  private SpeedControllerGroup leftMotors, rightMotors;
  private DifferentialDrive drive;
  private Encoder leftEncoder, rightEncoder;
  private NetworkTable table;
  private PWMTalonSRX arm, backRightT, frontRightT, backLeftT, frontLeftT;
  private Timer timer;
  private ADXRS450_Gyro gyro;
  private int state;
  private final I2C.Port i2cPort = I2C.Port.kOnboard; //this is the i2c port
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); //uses the i2c parameter
  private final ColorMatch m_colorMatcher = new ColorMatch(); //detects out of predetermained colors
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429); // these targets can be configured
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private boolean isCheckingColor, isSpinningToSpecific, isSpinningMult, hasSeenColor; //color logic
  private int totalSpins;
  private String requiredColor;
  private final double topSpeed = 0, maxSpeedDiff = 0.3, minSpeedDiff = 0.2;
  private boolean getTopSpeed = true, tracON = true;

  private pulsedLightLIDAR lidar;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    m_chooser.setDefaultOption("Turn 90", autoTurn90);
    m_chooser.addOption("Out and back", autoOutAndBack);
    m_chooser.addOption("Go 4 feet", autoGo4Feet);
    SmartDashboard.putData("Auto choices", m_chooser);

    // NavX sensor
    navx = new AHRS(I2C.Port.kMXP);

    // Magazine sensors
    // ballbeam1 = new AnalogInput(0);
    // ballbeam2 = new AnalogInput(1);
    // ballbeam3 = new AnalogInput(2);
    // ballbeam4 = new AnalogInput(3);
    // ballbeam5 = new AnalogInput(4);
    // ballbeam6 = new AnalogInput(5);
    // ballbeam7 = new AnalogInput(6);
    // ballbeam8 = new AnalogInput(7);
    // ballbeam9 = new AnalogInput(8);
    // ballbeam10 = new AnalogInput(9);

    // Xbox Controllers
    controllerdriver = new XboxController(0);
    controlleroperator = new XboxController(1);

    final boolean driveWheelsAreTalonsAndNotSparks = true; // If you change this to false it will try to run the wheels off sparks

    // Drive motors
    if(driveWheelsAreTalonsAndNotSparks){
      backRightT = new PWMTalonSRX(0);
      frontRightT = new PWMTalonSRX(1);
      backLeftT = new PWMTalonSRX(2);
      frontLeftT = new PWMTalonSRX(3);
      leftMotors = new SpeedControllerGroup(backLeftT, frontLeftT);
      rightMotors = new SpeedControllerGroup(backRightT, frontRightT);
    }else{
      backRight = new Spark(0);
      frontRight = new Spark(1);
      backLeft = new Spark(2);
      frontLeft = new Spark(3);
      leftMotors = new SpeedControllerGroup(backLeft, frontLeft);
      rightMotors = new SpeedControllerGroup(backRight, frontRight);
    }

    drive = new DifferentialDrive(leftMotors, rightMotors);

    leftEncoder = new Encoder(5, 6, true, Encoder.EncodingType.k2X);
    rightEncoder = new Encoder(3, 4, false, Encoder.EncodingType.k2X);
    leftEncoder.setDistancePerPulse(5.3/256);
    rightEncoder.setDistancePerPulse(5.3/256);

    table = NetworkTableInstance.getDefault().getTable("limelight");

    lidar = new pulsedLightLIDAR();
    lidar.start();

    align = false;
    approach = false;
    shoot = false;

    // Intake motors
    //intake = new Spark(4);

    // Belt motors in the magazine
    // belt1 = new Spark(5);
    // belt2 = new Spark(6);
    // belt3 = new Spark(7);
    // belt4 = new Spark(8);
    // loader = new Spark(9);

    // Arm motor
    // arm = new PWMTalonSRX(0);

    //Timer
    timer = new Timer();

    gyro = new ADXRS450_Gyro(SPI.Port.kMXP);

    isSpinningMult = true;
    isSpinningToSpecific = false;
    isCheckingColor = true;
    hasSeenColor = false;
    requiredColor = "Blue";
    totalSpins = 0;
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    colorMotor = new Spark(4); //defining motor with spark
  }

  public void setDriveWheels(double left, double right)
  {
    backLeft.set(-left);
    frontLeft.set(-left);
    backRight.set(right);
    frontRight.set(right);
  }

  public double directionToTarget()
  {
    NetworkTableEntry tx = table.getEntry("tx");
    double x = tx.getDouble(0.0);
    if(x > -3 && x < 3){ // Dead Zone
      return 0.0;
    }else if(x > -15 && x < -3){ // Move from left to center
      return -0.2;
    }else if(x < -15){
      return -0.4;
    }else if(x > 3 && x < 15){ // Move from right to center
      return 0.2;
    }else if(x > 15){
      return 0.4;
    }else{ // If it finds nothing it won't change direction
      return 0.0;
    }
  }

  public void approach()
  {
    // Do later bc no sensor :(
  }

  public void shoot()
  {

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
    if(isCheckingColor) 
    {
      colorCheck();
    }
  }

  public void colorCheck() 
  {
    Color detectedColor = m_colorSensor.getColor(); // the color that was detected from the sensor

    //checks if the color seen matches the colors
    String colorString, requiredColorActual; 
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red); //results pasted into shuffleboard & smart dash
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    if(isSpinningToSpecific) 
    {
      colorMotor.set(0.05);
      if(requiredColor == "Blue") {
        requiredColorActual = "Red";
      } else if (requiredColor == "Yellow") {
        requiredColorActual = "Green";
      } else if(requiredColor == "Red") {
        requiredColorActual = "Blue";
      } else if(requiredColor == "Green") {
        requiredColorActual = "Yellow";
      } else {
        requiredColorActual = "Unknown";
      } //translates the color we need to the color the sensor needs to stop on

      if(colorString == requiredColorActual) 
      {
        //stops checking colors after required color found
        isSpinningToSpecific = false;
        isCheckingColor = false;
        colorMotor.set(0);
      }
    } else if (isSpinningMult) 
    {
      colorMotor.set(0.05);
      //spins around the disk a total of 3.5 to 4 spins
      if(colorString == "Yellow" && !hasSeenColor) 
      {
        hasSeenColor = true;
        totalSpins++;
      } else {
        hasSeenColor = false;
      }
      
      if(totalSpins >= 7) {
        //stops checking colors after spins
        isSpinningMult = false;
        isCheckingColor = false;
        totalSpins = 0;
        colorMotor.set(0);
      }
    }
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
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    timer.reset();
    timer.start();
    navx.reset();

    state = 1;
  }

  public void turn90()
  {
    System.out.println(navx.getAngle());

    if (navx.getAngle() < 75)         //Until 75 degrees, the robot turns at half power 
      setDriveWheels(0.5, -0.5);
    else if (navx.getAngle() < 90)    // For the last 15 degrees, the robot turns at third power
      setDriveWheels(0.3, 0.3);
    else
      setDriveWheels(0, 0);
  }

  public void outAndBack()
  {
     switch (state) {
       case 1:
         // Go forward 36"
         setDriveWheels(0.5, 0.5);
         if (leftEncoder.getDistance() >= 36)
           state++;
         break;
 
       case 2:
         // Turn 180 degrees
         setDriveWheels(0.5, -0.5);
         if (navx.getAngle() >= 180) {
           leftEncoder.reset();
           rightEncoder.reset();
           state++;
         }
         break;
 
       case 3:
         // Go forward 36" again (return)
         setDriveWheels(0.5, 0.5);
         if (leftEncoder.getDistance() >= 36) {
           navx.reset();
           state++;
         }
         break;
 
       case 4:
         // Turns itself 180 degrees
         setDriveWheels(0.5, -0.5);
         if (navx.getAngle() >= 180)
           state++;
           break;
 
       case 5:
         // Stops the robot
         setDriveWheels(0, 0);
         break;
    }
  }

  public void go4Feet()
  {
    System.out.println("Left: " + leftEncoder.getDistance() + " Right: " + rightEncoder.getDistance());
    if (leftEncoder.getDistance() < 48)
      setDriveWheels(0.3, 0.3);
    else
      setDriveWheels(0, 0);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {
    switch (m_autoSelected) {
      case autoTurn90:
        turn90();
        break;
      case autoGo4Feet:
      default:
        go4Feet();
        break;
    }

  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    double driveSpeed = controllerdriver.getY(GenericHID.Hand.kLeft);
    double driveDirection = controllerdriver.getX(GenericHID.Hand.kRight);
    //int pov = controllerdriver.getPOV(0);

    // System.out.println("Left: " + leftEncoder.getDistance() + " Right: " + rightEncoder.getDistance());
    // setDriveWheels(speed - direction, speed + direction);

    if(tracON){
      private double currentSpeedAvg = ((leftEncoder.getRate() + rightEncoder.getRate()) / 2) / topSpeed;
      if(driveSpeed > (currentSpeedAvg + maxSpeedDiff)){
        driveSpeed = (currentSpeedAvg + maxSpeedDiff);
      }else if(driveSpeed < (currentSpeedAvg - minSpeedDiff)){
        driveSpeed = (currentSpeedAvg - minSpeedDiff);
      }
    }

    drive.arcadeDrive(driveSpeed, driveDirection);

    if(controllerdriver.getAButtonPressed()){
      align = !align;
    }
    if(controllerdriver.getXButtonPressed()){
      approach = !approach;
    }
    if(controllerdriver.getYButtonPressed()){
      shoot = !shoot;
    }
    if(align){
      double autoDirection = directionToTarget();
      setDriveWheels(-autoDirection, autoDirection);
    }
    if(approach){
      approach();
    }
    if(shoot){
      shoot();
    }

    double lidarDist = lidar.getDistanceIn();
    System.out.println("Cool lidar stuff: " + lidarDist);

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  {

    if(getTopSpeed){
      private double driveSpeed = controllerdriver.getY(GenericHID.Hand.kLeft);
      private double driveDirection = controllerdriver.getX(GenericHID.Hand.kRight);
      private double currentSpeedAvg = (leftEncoder.getRate() + rightEncoder.getRate()) / 2;

      drive.arcadeDrive(driveSpeed, driveDirection, true);

      if(currentSpeedAvg > topSpeed){
        topSpeed = currentSpeed;
      }
      print("Top Speed: " + topSpeed)
    }
  }
}






