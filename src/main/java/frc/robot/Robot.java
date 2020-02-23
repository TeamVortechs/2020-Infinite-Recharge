package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PWMVictorSPX;
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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Ultrasonic;
// import com.ctre.phoenix.music.Orchestra;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import java.util.ArrayList;

import javax.xml.transform.Source;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;

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
  private static final String autoBackAndAround = "autoBackAndAround";
  private static final String autoTurn90 = "autoTurn90";
  private static final String autoGoAround = "autoGoAround";

  private static final double shootDistance = 30.0;
  private double shootPower = 0.0; // Motor current shoot power (adjusted in shoot() function)
  private double shootRate = 530; // Target RPM
  private static final double ticksPerInch = 1075.65;
  private double lidarDist, area, offsetAngle;

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private boolean align, approach, shoot, beltToggle;
  private AHRS navx;
  private AnalogInput ballbeam1, ballbeam2, ballbeam3, ballbeam4, ballbeam5, ballbeam6, ballbeam7, ballbeam8, ballbeam9, ballbeam10;
  private XboxController controllerdriver, controlleroperator;
  private Spark shooterP, shooterD, backRightS, frontRightS, backLeftS, frontLeftS, intake, colorMotor;
  private Encoder shootEncoder;
  private NetworkTable table;
  private NetworkTableEntry ta;
  private TalonFX belt, backRightT, frontRightT, backLeftT, frontLeftT;
  private Timer timer;
  private int state;

  private final I2C.Port i2cPort = I2C.Port.kOnboard; //this is the i2c port
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); //uses the i2c parameter
  private final ColorMatch m_colorMatcher = new ColorMatch(); //detects out of predetermained colors
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429); // these targets can be configured
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private boolean beltOn, isCheckingColor, isSpinningToSpecific, isSpinningMult, hasSeenColor; //color logic
  private int totalSpins;
  private String requiredColor;
  private int ultrasonicLPort, ultrasonicMPort, ultrasonicRPort;
  private double ultrasonicLDistance, ultrasonicMDistance, ultrasonicRDistance;
  private AnalogInput m_ultrasonicL, m_ultrasonicM, m_ultrasonicR;
  private static final double kValueToInches = 0.125, intakeSpeed = 0.4;
  
  private double topSpeed = 20857, maxSpeedDiff = 0.2, minSpeedDiff = 0.2, beltSpeed = 0.9;

  private double leftEncoderZero, rightEncoderZero;

  private boolean trac = true, intakeToggle, forward6, back6, left30, right30;
  final boolean driveWheelsAreTalonsAndNotSparks = true; // If you change this to false it will try to run the wheels off something

  private pulsedLightLIDAR lidar;
  private DigitalSource lidarPort = new DigitalInput(4); // fix my port and all uses of lidar.

    /* The orchestra object that holds all the instruments */
  // private Orchestra _orchestra;
  //   /* Talon FXs to play music through.  
  //   More complex music MIDIs will contain several tracks, requiring multiple instruments.  */
  // private TalonFX [] _fxes =  { new TalonFX(1), new TalonFX(2), new TalonFX(3), new TalonFX(4) };

  //   /* An array of songs that are available to be played, can you guess the song/artists? */
  // String song = "crabRave.chrp";
  

  /* A list of TalonFX's that are to be used as instruments */

  // ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    // playMusic();
    m_chooser.setDefaultOption("Turn 90", autoTurn90);
    m_chooser.addOption("Out and back", autoOutAndBack);
    m_chooser.addOption("Back and around", autoBackAndAround);
    m_chooser.addOption("Go 4 feet", autoGo4Feet);
    m_chooser.addOption("Go Around", autoGoAround);
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

    // BUTTON LAYOUT FOR CONTROLLERS:
    //
    // Driver:
    //   A: Intake Toggle ::: TO-DO
    //   B: N/A
    //   X: tracON/tracOFF
    //   Y: N/A
    //   Left Joystick X-Axis: N/A
    //   Left Joystick Y-Axis: Forward and Backward Desired Speeds
    //   Right Joystick X-Axis: Direction
    //   Right Joystick Y-Axis: N/A
    //   D-Pad Up: Move forward 6 inches ::: TO-DO
    //   D-Pad Down: Move backward 6 inches ::: TO-DO
    //   D-Pad Left: Rotate -30 degrees ::: TO-DO
    //   D-Pad Right: Rotate +30 degrees ::: TO-DO
    //   Right Trigger: N/A
    //   Left Trigger: Limelight align and LIDAR approach ::: TO-DO
    //   Start Button: Break from any loop
    
    // Operator:
    //   A: Start Color Wheel Turning (Toggle) ::: TO-DO
    //   B: N/A
    //   X: N/A
    //   Y: N/A
    //   Left Joystick X-Axis: N/A
    //   Left Joystick Y-Axis: Left Winch Up/Down ::: TO-DO
    //   Right Joystick X-Axis: N/A
    //   Right Joystick Y-Axis: Right Winch Up/Down ::: TO-DO
    //   D-Pad Up: Elevator Up Toggle ::: TO-DO also wait for last 30 to be able to use
    //   D-Pad Down: Elevator Down Toggle ::: TO-DO
    //   D-Pad Left: Color Wheel Left One (1) Color ::: TO-DO
    //   D-Pad Right: Color Wheel Right One (1) Color ::: TO-DO
    //   Right Trigger: Shoot (Until Released) ::: TO-DO
    //   Left Trigger: N/A

    // Drive motors
    if(driveWheelsAreTalonsAndNotSparks){
      backRightT = new TalonFX(1);
      frontRightT = new TalonFX(2);
      backLeftT = new TalonFX(3);
      frontLeftT = new TalonFX(4);
      backLeftT.setInverted(true);
      frontLeftT.setInverted(true);
      backLeftT.set(ControlMode.PercentOutput, 0);
      frontLeftT.set(ControlMode.PercentOutput, 0);
      backRightT.set(ControlMode.PercentOutput, 0);
      frontRightT.set(ControlMode.PercentOutput, 0);
    // }else{
    //   backRightS = new Spark(0);
    //   frontRightS = new Spark(1);
    //   backLeftS = new Spark(2);
    //   frontLeftS = new Spark(3);
    //   backLeftS.set(0);
    //   frontLeftS.set(0);
    //   backRightS.set(0);
    //   frontRightS.set(0);
    }

    shootEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k2X); // ideal for 0.7 is +530
    // rightEncoder = new Encoder(3, 4, false, Encoder.EncodingType.k2X);
    // leftEncoder.setDistancePerPulse(5.3/256);
    // rightEncoder.setDistancePerPulse(5.3/256);

    table = NetworkTableInstance.getDefault().getTable("limelight");

    lidar = new pulsedLightLIDAR(lidarPort);
    lidar.getDistance();

    align = false;
    approach = false;
    shoot = false;

    // Intake motors
    intake = new Spark(0);

    // Belt motor
    belt = new TalonFX(6);

    // Shooter motor
    shooterD = new Spark(1); // Driver Side
    shooterP = new Spark(3); // Passenger Side

    // Arm motor
    // arm = new PWMTalonSRX(0);

    //Timer
    timer = new Timer();

    ultrasonicLPort = 0;
    ultrasonicMPort = 1;
    ultrasonicRPort = 2; //Ultrasonic(int pingChannel, int echoChannel)

    m_ultrasonicL = new AnalogInput(ultrasonicLPort);
    m_ultrasonicM = new AnalogInput(ultrasonicMPort);
    m_ultrasonicR = new AnalogInput(ultrasonicRPort);
    System.out.println("BEFORE left: " + getLeftDriveDistance() + " right: " + getRightDriveDistance());
    resetDistance();
    System.out.println("AFTER left: " + getLeftDriveDistance() + " right: " + getRightDriveDistance());

    // isSpinningMult = false;
    // isSpinningToSpecific = false;
    // isCheckingColor = false;
    // hasSeenColor = false;
    // requiredColor = "Blue";
    // totalSpins = 0;
    // m_colorMatcher.addColorMatch(kBlueTarget);
    // m_colorMatcher.addColorMatch(kGreenTarget);
    // m_colorMatcher.addColorMatch(kRedTarget);
    // m_colorMatcher.addColorMatch(kYellowTarget);
     // colorMotor = new Spark(10); 
    //defining motor with spark

    /* Initialize the TalonFX's to be used */
  }

  public void drive(double desiredSpeed, double direction, boolean tracON){ // Both desiredSpeed and direction should be sent as positive values as you would expect
    if(tracON){
      double currentSpeedAvg = getDriveSpeed() / topSpeed;
      if(desiredSpeed > (currentSpeedAvg + maxSpeedDiff)){
        desiredSpeed = (currentSpeedAvg + maxSpeedDiff);
      }else if(desiredSpeed < (currentSpeedAvg - minSpeedDiff)){
        desiredSpeed = (currentSpeedAvg - minSpeedDiff);
      }
    }

    double leftSpeedFinal = desiredSpeed - direction;
    double rightSpeedFinal = desiredSpeed + direction;

    if(driveWheelsAreTalonsAndNotSparks){
      backLeftT.set(ControlMode.PercentOutput, leftSpeedFinal);
      frontLeftT.set(ControlMode.PercentOutput, leftSpeedFinal);
      backRightT.set(ControlMode.PercentOutput, rightSpeedFinal);
      frontRightT.set(ControlMode.PercentOutput, rightSpeedFinal);
    }else{
      // backLeft.set(-leftSpeedFinal * 0.7);
      // frontLeft.set(-leftSpeedFinal * 0.7);
      backLeftS.set(-leftSpeedFinal);
      frontLeftS.set(-leftSpeedFinal);
      backRightS.set(rightSpeedFinal);
      frontRightS.set(rightSpeedFinal);
    }
  }

  // public void directDrive(double desiredSpeed, double direction){
  //   double leftSpeedFinal = desiredSpeed + direction;
  //   double rightSpeedFinal = desiredSpeed - direction;

  //   if(driveWheelsAreTalonsAndNotSparks){
  //     backLeftT.set(-leftSpeedFinal);
  //     frontLeftT.set(-leftSpeedFinal);
  //     backRightT.set(rightSpeedFinal);
  //     frontRightT.set(rightSpeedFinal);
  //   }else{
  //     backLeft.set(-leftSpeedFinal);
  //     frontLeft.set(-leftSpeedFinal);
  //     backRight.set(rightSpeedFinal);
  //     frontRight.set(rightSpeedFinal);
  //   }
  // }

  //resets the encoder values to 0
  public void resetDistance() 
  {
    // backLeftT.setSelectedSensorPosition(0, 0, 10);
    // backRightT.setSelectedSensorPosition(0, 0, 10);
    leftEncoderZero = backLeftT.getSelectedSensorPosition();
    rightEncoderZero = backRightT.getSelectedSensorPosition();
  }

  //takes average of the encoder values in inches
  public double getDriveDistance() 
  {
    return (getLeftDriveDistance() + getRightDriveDistance())/2;
  }

  //takes average of encoder rates
  public double getDriveSpeed() 
  {
    return (backLeftT.getSelectedSensorVelocity() + backRightT.getSelectedSensorVelocity())/2;
  }

  //takes the left encoder value and returns distance in inches
  public double getLeftDriveDistance() 
  {
    return (backLeftT.getSelectedSensorPosition() - leftEncoderZero) / ticksPerInch;
  }

  //takes the right encoder value and returns distance in inches
  public double getRightDriveDistance() 
  {
    return (backRightT.getSelectedSensorPosition() - rightEncoderZero) / ticksPerInch;
  }

  public void goStraight(double power)
  {
    drive(power, 0, false);
  }

  public double directionToTarget()
  {
    NetworkTableEntry tx = table.getEntry("tx");
    double x = tx.getDouble(0.0);
    System.out.println("x: " + x);
    if(x > -3 && x < 3){              // Dead Zone
      // controllerdriver.setRumble(RumbleType.kLeftRumble, 1);
      // controllerdriver.setRumble(RumbleType.kRightRumble, 1);
      // controlleroperator.setRumble(RumbleType.kLeftRumble, 1);
      // controlleroperator.setRumble(RumbleType.kRightRumble, 1);
      return 0.0;
    }else if(x > -15 && x < -3){      // Move from left to center
      return -0.3;
    }else if(x < -15){
      return -0.5;
    }else if(x > 3 && x < 15){        // Move from right to center
      return 0.3;
    }else if(x > 15){
      return 0.5;
    }else{                            // If it finds nothing it won't change direction
      System.out.println("it is nothing");
      return 0.0;
    }
  }

  public void align()
  {
    double autoDirection = directionToTarget();
    drive(0, autoDirection, false);
  }

  public void approach()
  {
    // Do later bc no sensor :(
  }

  public void shoot(double targetRate)
  {
    double rate = shootEncoder.getRate();
    double speedChange = (targetRate - rate) * 0.0001;
    shootPower += speedChange;
    shootPower = Math.max(0.3, Math.min(0.8, shootPower));
    if(targetRate == 0)
      shootPower = 0;
    if(rate < (targetRate + 15) && rate > (targetRate - 15)){
      belt.set(ControlMode.PercentOutput, 0.9);
      controllerdriver.setRumble(RumbleType.kLeftRumble, 1);
      controllerdriver.setRumble(RumbleType.kRightRumble, 1);
      controlleroperator.setRumble(RumbleType.kLeftRumble, 1);
      controlleroperator.setRumble(RumbleType.kRightRumble, 1);
    }else{
      belt.set(ControlMode.PercentOutput, 0.0);
      controllerdriver.setRumble(RumbleType.kLeftRumble, 0);
      controllerdriver.setRumble(RumbleType.kRightRumble, 0);
      controlleroperator.setRumble(RumbleType.kLeftRumble, 0);
      controlleroperator.setRumble(RumbleType.kRightRumble, 0);
    }
    shooterD.set(shootPower);
    shooterP.set(shootPower);
  }

  public void stopShooter(){
    controllerdriver.setRumble(RumbleType.kLeftRumble, 0);
    controllerdriver.setRumble(RumbleType.kRightRumble, 0);
    controlleroperator.setRumble(RumbleType.kLeftRumble, 0);
    controlleroperator.setRumble(RumbleType.kRightRumble, 0);
    shooterD.set(0);
    shooterP.set(0);
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
    // if(isCheckingColor) 
    // {
    //   colorCheck();
    // }
  }

  public void getDistances() 
  {
    lidarDist = lidar.getDistance();
    ta = table.getEntry("ta");
    area = ta.getDouble(0.0);
    ultrasonicLDistance = m_ultrasonicL.getValue() * kValueToInches;
    ultrasonicMDistance = m_ultrasonicM.getValue() * kValueToInches;
    ultrasonicRDistance = m_ultrasonicR.getValue() * kValueToInches;
    SmartDashboard.putNumber("Distance Left", ultrasonicLDistance);
    SmartDashboard.putNumber("Distance Middle", ultrasonicMDistance);
    SmartDashboard.putNumber("Distance Right", ultrasonicRDistance);
    // System.out.println("Distance Left: " + ultrasonicLDistance);
    // System.out.println("Distance Middle: " + ultrasonicMDistance);
    // System.out.println("Distance Right: " + ultrasonicRDistance);
   }

  public void colorCheck() 
  {
    Color detectedColor = m_colorSensor.getColor(); // the color that was detected from the sensor

    //checks if the color seen matches the colors
    String colorString, requiredColorActual; 
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      colorString = "Blue";
      System.out.println("Blue");
    } else if (match.color == kRedTarget) {
      colorString = "Red";
      System.out.println("Red");
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
      System.out.println("Green");
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      System.out.println("Yellow");
    } else {
      colorString = "Unknown";
      System.out.println("Unknown");
    }

    SmartDashboard.putNumber("Red", detectedColor.red); //results pasted into shuffleboard & smart dash
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    if(isSpinningToSpecific) 
    {
      // colorMotor.set(0.05);
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
      // colorMotor.set(0.05);
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
    state = 1;
    m_autoSelected = m_chooser.getSelected();
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    timer.reset();
    timer.start();
    navx.reset();

    // backLeftT.enableBrakeMode(true);
    // frontLeftT.enableBrakeMode(true);
    // backRightT.set(ControlMode.PercentOutput, rightSpeedFinal);
    // frontRightT.set(ControlMode.PercentOutput, rightSpeedFinal);

    resetDistance();
  }
  //used if no positioning required (variables can change if you want)
  public void autonomousPos1() 
  {
    switch (state) {
      case 1:
        align = true;//sweetspot
        break;
      case 2:
        shoot(shootRate);//shoot
        break;
      case 3:
        drive(0.5, -0.5, false);//turn toward wall
        if (getDriveDistance()  >= 90) {
          state++;
          resetDistance();
        }
        break;
      case 4:
        drive(0.5, 0.5, false);//drive toward wall
        if (getDriveDistance()  >= 10) {
          state++;
          resetDistance();
        }
      case 5:
        drive(0.5, -0.5, false);//turn toward other balls
        if (getDriveDistance()  >= 90) {
          state++;
          resetDistance();
        }
      case 6:
        drive(0.5, 0.5, false);//get outa there toward balls
        if (getDriveDistance()  >= 30 || lidarDist <= 100) {
          state++;
          resetDistance();
        }
      case 7:
        drive(0, 0, false);//stop
        break;
    }
  }
  //used if positioning required (variables can change if you want)
  public void autonomousPos2() 
  {
    switch (state) {
      case 1:
        drive(-0.5, -0.5, false);
        if (getDriveDistance()  <= -20) {//sweet spot y
          state++;
          resetDistance();
        }
        break;
      case 2:
      drive(0.5, -0.5, false);
        if (getDriveDistance()  >= 90) {//turn 90
          state++;
          resetDistance();
        }
        break;
      case 3:
        drive(0.5, 0.5, false);
        if (getDriveDistance()  >= 40 || lidarDist <= 100) {//sweet spot x
          state++;
          resetDistance();
        }
        break;
      case 4:
        drive(-0.5, 0.5, false);
        if (getDriveDistance()  <= -90 || (directionToTarget() == 0.0 && area != 0.0)) {//turn -90 or until seen the target
          state++;
          offsetAngle = getDriveDistance();
          resetDistance();
        }
        break;
      case 5:
        align = true;
        break;
      case 6:
        shoot(shootRate);
        break;
      case 7:
        drive(0.5, -0.5, false);
        if (getDriveDistance()  >= offsetAngle) {//return to angle
          state++;
          resetDistance();
        }
        break;
      case 8:
        drive(0.5, 0.5, false);
        if (getDriveDistance()  >= 10) {//drive to wall
          state++;
          resetDistance();
        }
        break;
      case 9:
        drive(0.5, -0.5, false);
        if (getDriveDistance()  >= 90) {//turn toward balls
          state++;
          resetDistance();
        }
      case 10:
        drive(0.5, 0.5, false);
        if (getDriveDistance()  >= 30 || lidarDist <= 100) {//drive toward balls
          state++;
          resetDistance();
        }
      case 11:
        drive(0, 0, false);//stop
        break;
    }
  }

  public void turn90()
  {
    System.out.println(navx.getAngle());

    if (navx.getAngle() < 75)         //Until 75 degrees, the robot turns at half power 
      drive(0.0, 0.5, false);
    else if (navx.getAngle() < 90)    // For the last 15 degrees, the robot turns at third power
      drive(0.0, 0.3, false);
    else
      drive(0, 0, false);
  }

  public void outAndBack()
  {
     switch (state) {
       case 1:
         // Go forward 36"
         goStraight(0.1);
         if (getDriveDistance()  >= 36)
           state++;
         break;
 
       case 2:
         // Turn 180 degrees
         drive(0.0, -0.1, false);
         if (navx.getAngle() >= 170) {
          resetDistance();
           state++;
         }
         break;
 
       case 3:
         // Go forward 36" again (return)
         goStraight(0.1);
         if (getDriveDistance()  >= 36) {
           navx.reset();
           state++;
         }
         break;
 
       case 4:
         // Turns itself 180 degrees
         drive(0.0, -0.1, false);
         if (navx.getAngle() >= 170)
           state++;
           break;
 
       case 5:
         // Stops the robot
         drive(0, 0, false);
         break;
    }
  }

  public void backAndAround() {
    switch (state) {
      case 1:
        drive(0.5, 0.0, false);
        if (getDriveDistance()  >= 36) {
          state++;
        }
        break;

      case 2:
        drive(-0.5, 0.0, false);
        if (navx.getAngle() <= 280) {
          resetDistance();
          state++;
        }
        break;

      case 3:
        drive(0.5, 0.0, false);
        if (getDriveDistance()  >= 180) {
          navx.reset();
          state++;
        }
        break;
      
      case 4:
        drive(-0.5, 0.0, false);
        if (navx.getAngle() <= 280) {
          state++;
        }
      
      case 5:
        drive(0, 0, false);
        break;
    }
  }

  public void go4Feet()
  {
    System.out.println("Left: " + getLeftDriveDistance() + " Right: " + getRightDriveDistance());
    if (getDriveDistance()  < 48)
      drive(0.1, 0.0, false);
    else
      drive(0, 0, false);
  }

public void autoGoAround()
{
  switch (state) {
    case 1: //drives forward 2 feet
      drive(0.5, 0.0, false);
      if (getDriveDistance()  >= 24)
        state++;
      break;
      
    case 2: //turns right 90
      drive(0.0, 0.5, false);
      if (navx.getAngle() >= 90) {
        resetDistance();
        state++;
      }
      break;

    case 3: //drives forward 4 feet
      drive(0.5, 0.0, false);
      if (getDriveDistance()  >= 48) {
        navx.reset();
        state++;
      }
      break;

    case 4: //turns right 90
      drive(0.0, 0.5, false);
      if (navx.getAngle() >= 90) {
        resetDistance();
        state++;
      }
      break;

    case 5: //forward 2 feet
      drive(0.5, 0.0, false);
      if (getDriveDistance()  >= 24)
        state++;
      break;

    case 6:
      drive(0, 0, false);
      break;
  }
}

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {
    if(align){
      align();
    }
    switch (m_autoSelected) {
      case autoTurn90:
        turn90();
        break;

      case autoOutAndBack:
        outAndBack();
        break;
        
      case autoBackAndAround:
        backAndAround();
        break;

      case autoGo4Feet:
      default:
        go4Feet();
        break;

      case autoGoAround:
        autoGoAround();
        break;
        
    }

  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {

    //
    // DRIVER CONTROLLER CODE
    //

    double driverJoystickY = -controllerdriver.getY(GenericHID.Hand.kLeft);
    double driverJoystickX = -controllerdriver.getX(GenericHID.Hand.kRight) * 0.5;
    if (Math.abs(driverJoystickY) < 0.1) // Zero joysticks
      driverJoystickY = 0;
    
    if (Math.abs(driverJoystickX) < 0.1) 
      driverJoystickX = 0;

    // Toggle Swtiches for Driver
    if(controllerdriver.getXButtonPressed())
      trac = !trac;
    if(controllerdriver.getAButtonPressed())
      intakeToggle = !intakeToggle;
    if(controllerdriver.getPOV() == 0)
      forward6 = !forward6;
    if(controllerdriver.getPOV() == 90)
      right30 = !right30;
    if(controllerdriver.getPOV() == 180)
      back6 = !back6;
    if(controllerdriver.getPOV() == 270)
      left30 = !left30;

    if(intakeToggle){
      intake.set(-intakeSpeed);
    }else{
      intake.set(0);
    }

    // System.out.println(controllerdriver.getTriggerAxis(GenericHID.Hand.kLeft));
    // Intense trigger algorithms
    if(controllerdriver.getTriggerAxis(GenericHID.Hand.kLeft) > 0.5) // Complicated algorithm to decide if the left trigger is being held
      System.out.println("alinging");
      align();

    drive(driverJoystickY, driverJoystickX, trac); // Actually calls the driving

    //
    // OPERATOR CONTROLLER
    //

    double operatorJoystickYLeft = -controlleroperator.getY(GenericHID.Hand.kLeft);
    double operatorJoystickYRight = -controlleroperator.getY(GenericHID.Hand.kRight);

    // YOU WOULD SET THESE JOYSTICK VALUES TO THE WINCH MOTOR(S) IF YOU KNEW ANYTHING ABOUT THEM BUT :(

    if(controlleroperator.getBButtonPressed()){
      beltSpeed += 0.1;
    }else if(controlleroperator.getXButtonPressed()){
      beltSpeed -= 0.1;
    }

    if(controlleroperator.getTriggerAxis(GenericHID.Hand.kRight) > 0.5) // Complicated algorithm to decide if the right trigger is being held
      shoot = true;
    else if(controlleroperator.getTriggerAxis(GenericHID.Hand.kRight) < 0.5)
      shoot = false;

    // if(lidar.getDistance() < 250){
    //   shoot = false;
    // }

    if(shoot){
      shoot(shootRate);
    }else{
      stopShooter();
      if(controlleroperator.getTriggerAxis(GenericHID.Hand.kLeft) > 0.5) // Complicated algorithm to decide if the left trigger is being held
        belt.set(ControlMode.PercentOutput, beltSpeed);
      else if(controlleroperator.getTriggerAxis(GenericHID.Hand.kLeft) < 0.5)
        belt.set(ControlMode.PercentOutput, 0);
    }

    if(controlleroperator.getYButtonPressed()){
      shootRate += 10;
    }else if(controlleroperator.getAButtonPressed()){
      shootRate -= 10;
    }

    double lidarDist = lidar.getDistance();

    // shooterD.set(-shooterSpeed);
    // shooterP.set(shooterSpeed);

    System.out.println("Shooter Power: " + shootPower + " and Lidar Dist: " + lidarDist + " and Belt Speed: " + beltSpeed + " Shoot Rate: " + shootEncoder.getRate());

    // if(controllerdriver.getBButtonPressed()){
    //   playMusic();

    //   System.out.println("I'm playing music!");
    // }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  {
    // For testing lidar distance

    // double lidarDist = lidar.getDistance();
    // System.out.println("Cool lidar distance: " + lidarDist);
    System.out.println("Left: " + getLeftDriveDistance() + " Right: " + getRightDriveDistance());
    if(true){
       double driverJoystickY = controllerdriver.getY(GenericHID.Hand.kLeft); // good luck future team members uwu :)
       double driverJoystickX = controllerdriver.getX(GenericHID.Hand.kRight);
       double currentSpeedAvg = getDriveSpeed();

      drive(0, 0, false);

      if(currentSpeedAvg > topSpeed){
        topSpeed = currentSpeedAvg;
      }
      System.out.println("Top Speed: " + topSpeed);
    }
  }

  // public void playMusic(){
  //   /* load the chirp file */
  //   _orchestra.loadMusic(song); 
  //   _orchestra.play();
  //  }
   
}







