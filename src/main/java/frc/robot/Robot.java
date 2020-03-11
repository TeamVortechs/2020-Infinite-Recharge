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
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.UsbCamera;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
//import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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
  private static final String shootAndSit = "shootAndSit";
  private static final String shootAndForward = "shootAndForward";
  private static final String shootAndBackward = "shootAndBackward";


  private double shootPower = 0.0; // Motor current shoot power (adjusted in shoot() function)
  private double shootRate = 530; // Target RPM
  private static final double ticksPerInch = 1075.65;
  private double lidarDist, area, offsetAngle;

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private boolean align, shoot;
  private AHRS navx;
  private DigitalInput firstBallSensor, topBallSensor;
  private AnalogInput ballbeam3, ballbeam4, ballbeam5, ballbeam6, ballbeam7, ballbeam8, ballbeam9, ballbeam10;
  private XboxController controllerdriver, controlleroperator;
  private Spark shooterP, shooterD, backRightS, frontRightS, backLeftS, frontLeftS, intake, colorMotor;
  private Encoder shootEncoder, beltEncoder;
  private NetworkTable limelightTop, limelightBottom;
  private NetworkTableEntry ta;
  private TalonFX elevator, belt, backRightT, frontRightT, backLeftT, frontLeftT, winchL, winchR;
  private Timer timer;
  private int state, talonFXMode = ControlMode.PercentOutput;
  private final double intakeSpeed = 0.4;
  
  private double topSpeed = 20857, maxSpeedDiff = 0.1, minSpeedDiff = 0.1, beltSpeed = -0.7, LLOffset = 2.5;

  private double leftEncoderZero, rightEncoderZero;
  private double bottomLLOffset, topLLOffset;

  private boolean trac = false, longshot = false, intakeToggle = false, autoisbeingdumb = false;
  final boolean driveWheelsAreTalonsAndNotSparks = true; // If you change this to false it will try to run the wheels off something

  private pulsedLightLIDAR lidar;
  private DigitalSource lidarPort = new DigitalInput(9);

    /* The orchestra object that holds all the instruments */
   //private Orchestra _orchestra;

  //   /* Talon FXs to play music through.  
  //   More complex music MIDIs will contain several tracks, requiring multiple instruments.  */
  //  private TalonFX [] _fxes =  { new TalonFX(1), new TalonFX(2), new TalonFX(3), new TalonFX(4) };

  //   /* An array of songs that are available to be played, can you guess the song/artists? */
   //String song = "crabRave.chrp";
  

  /* A list of TalonFX's that are to be used as instruments */

   //ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {

   // For USB Camera, 1678 gave us this code so play nice with it
   UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
   camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
   MjpegServer camServer = new MjpegServer("serve_USB camera 0", 5810);
   camServer.setSource(camera);

    // playMusic();
    m_chooser.setDefaultOption("Shoot and Back off Line", shootAndBackward);
    m_chooser.addOption("Shoot and Forward off Line", shootAndForward);
    m_chooser.addOption("Shoot and Sit Motionless but dont choose this one because you would give up 5 points so throw a fit well before even considering this option", shootAndSit);
    SmartDashboard.putData("Auto choices", m_chooser);

    // NavX sensor
    navx = new AHRS(I2C.Port.kMXP);

    // Magazine sensors
    firstBallSensor = new DigitalInput(7);
    topBallSensor = new DigitalInput(4);

    // Xbox Controllers
    controllerdriver = new XboxController(0);
    controlleroperator = new XboxController(1);

    // BUTTON LAYOUT FOR CONTROLLERS:
    //
    // Driver:
    //   A: N/A
    //   B: N/A
    //   X: tracON/tracOFF
    //   Y: N/A
    //   Left Joystick X-Axis: N/A
    //   Left Joystick Y-Axis: Forward and Backward Desired Speeds
    //   Left Joystick press: slow mode ::: TO-DO
    //   Right Joystick X-Axis: Direction
    //   Right Joystick Y-Axis: N/A
    //   D-Pad Up: N/A
    //   D-Pad Down: N/A
    //   D-Pad Left: N/A
    //   D-Pad Right: N/A
    //   Right Trigger: Run the belt forward
    //   Right Bumper: Run the belt backward
    //   Left Trigger: Intake In
    //   Left Bumper: Intake Out
    //   Start Button: Break from any loop
    
    // Operator:
    //   A: Color Wheel Stuff ::: TO-DO all four being left as adjustments for belt and shooter speeds for time being
    //   B: Color Wheel Stuff ::: TO-DO
    //   X: Color Wheel Stuff ::: TO-DO
    //   Y: Color Wheel Stuff ::: TO-DO
    //   Left Joystick X-Axis: N/A
    //   Left Joystick Y-Axis: Left Winch Up/Down ::: TO-DO
    //   Right Joystick X-Axis: N/A
    //   Right Joystick Y-Axis: Right Winch Up/Down ::: TO-DO
    //   D-Pad Up: Elevator Up Toggle ::: TO-DO also wait for last 30 to be able to use
    //   D-Pad Down: Elevator Down Toggle ::: TO-DO
    //   D-Pad Left: adjust slightly left ::: or this could adjust the limelight offset value to change the alignment for the whole match
    //   D-Pad Right: adjust slightly right
    //   Right Trigger: Shoot (Until Released)
    //   Right Bumper:  Shoot reverse
    //   Left Trigger: Align
    //   Left Bumper: N/A
    //   Start Button: Break from any loop

    // Drive motors
    if(driveWheelsAreTalonsAndNotSparks){
      backRightT = new TalonFX(1);
      frontRightT = new TalonFX(2);
      backLeftT = new TalonFX(3);
      frontLeftT = new TalonFX(4);
      backLeftT.setInverted(true);
      frontLeftT.setInverted(true);
      backLeftT.set(talonFXMode, 0);
      frontLeftT.set(talonFXMode, 0);
      backRightT.set(talonFXMode, 0);
      frontRightT.set(talonFXMode, 0);
    }else{
      backRightS = new Spark(0);
      frontRightS = new Spark(1);
      backLeftS = new Spark(2);
      frontLeftS = new Spark(3);
      backLeftS.set(0);
      frontLeftS.set(0);
      backRightS.set(0);
      frontRightS.set(0);
    }

    shootEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k2X); // ideal for 0.7 is +530
    beltEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k2X);

    limelightTop = NetworkTableInstance.getDefault().getTable("limelight-top");
    limelightBottom = NetworkTableInstance.getDefault().getTable("limelight-bottom");

    limelightTop.getEntry("ledMode").setNumber(2); // setNumber(2) makes the limelights blink for targeting
    limelightBottom.getEntry("ledMode").setNumber(2);

    lidar = new pulsedLightLIDAR(lidarPort);

    align = false;
    shoot = false;

    // Intake motors
    intake = new Spark(0);

    // Belt motor
    belt = new TalonFX(6);

    //Winches
    winchL = new TalonFX(8);
    winchR = new TalonFX(7);
    // winchL.setBrakeMode(true);
    // winchR.setBrakeMode(true);

    //elevator
    elevator = new TalonFX(9);

    // Shooter motor
    shooterD = new Spark(1); // Driver Side
    shooterP = new Spark(3); // Passenger Side

    // Arm motor
    // arm = new PWMTalonSRX(0);

    //Timer
    timer = new Timer();

  }

  //
  //
  //                ROBOT PERIODIC
  //
  //

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
    m_autoSelected = m_chooser.getSelected();
  }

  //
  //
  //                         AUTONOMOUS CODE 
  //
  //

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
  public void autonomousPeriodic()
  {
    switch(m_autoSelected) {
      case shootAndBackward:
        shootAndBackward();
        break;
      case shootAndForward:
        shootAndForward();
        break;
      case shootAndSit:
        shootAndSit();
        break;
    }
  }

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

    resetDistance();
  }

  // Assumes on line pointed relatively directly at goal
  public void shootAndBackward(){
    switch (state){
      case 1:
        limelightTop.getEntry("ledMode").setNumber(1);
        drive(0, 0, false);
        shoot(shootRate);
        if(Timer.getMatchTime() < 13 && shootRate == 0.0){
          autoisbeingdumb = true;
        }
        else{
          autoisbeingdumb = false;
        }
        if(Timer.getMatchTime() < 5.0){
          autoisbeingdumb = false;
          shoot(0);
          state++;
        }
        break;

      case 2:
        shoot(0);
        if(getDriveDistance() < -23){
          drive(0, 0, false);
          navx.reset();
          state++;
        }else{
          drive(-0.2, 0, false);
        }
        break;
    }
  }

  public void shootAndForward(){
    switch (state){
      case 1:
        limelightTop.getEntry("ledMode").setNumber(1);
        drive(0, 0, false);
        shoot(shootRate);
        if(Timer.getMatchTime() < 13 && shootRate == 0.0){
          autoisbeingdumb = true;
        }
        else{
          autoisbeingdumb = false;
        }
        if(Timer.getMatchTime() < 5.0){
          autoisbeingdumb = false;
          shoot(0);
          state++;
        }
        break;

      case 2:
        shoot(0);
        if(getDriveDistance() < 23){
          drive(0, 0, false);
          navx.reset();
          state++;
        }else{
          drive(0.2, 0, false);
        }
        break;
    }
  }

  public void shootAndSit(){
    switch (state){
      case 1:
        limelightTop.getEntry("ledMode").setNumber(1);
        drive(0, 0, false);
        shoot(shootRate);
        if(Timer.getMatchTime() < 13 && shootRate == 0.0){
          autoisbeingdumb = true;
        }
        else{
          autoisbeingdumb = false;
        }
        if(Timer.getMatchTime() < 5.0){
          autoisbeingdumb = false;
          shoot(0);
          state++;
        }
        break;
    }
  }

  //
  //
  //                HELPFUL FUNCTIONS
  //
  //

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

  // Boolean determines if belt should be running
  public void runBelt(boolean on, double speed){
    if(on && topBallSensor.get()){
      belt.set(talonFXMode, speed);
    }else{
      belt.set(talonFXMode, speed);
    }
  }

  public void print(String toPrint){
    System.out.println(toPrint);
  }

  //
  //
  //                  AUTO ALIGNING CODE
  //
  //

  public double directionToBalls(){
    NetworkTableEntry tx = limelightBottom.getEntry("tx");
    double x = tx.getDouble(0.0);
    if(x > -3 && x < 3){              // Dead Zone
      controllerdriver.setRumble(RumbleType.kLeftRumble, 1); // Operator gets rumble so they know to shoot
      controllerdriver.setRumble(RumbleType.kRightRumble, 1);
      return 0.0;
    }else if(x > -15 && x < -3){      // Move from left to center
      controllerdriver.setRumble(RumbleType.kLeftRumble, 0);
      controllerdriver.setRumble(RumbleType.kRightRumble, 0);
      return -0.1;
    }else if(x < -15){
      controllerdriver.setRumble(RumbleType.kLeftRumble, 0);
      controllerdriver.setRumble(RumbleType.kRightRumble, 0);
      return -0.3;
    }else if(x > 3 && x < 15){        // Move from right to center
      controllerdriver.setRumble(RumbleType.kLeftRumble, 0);
      controllerdriver.setRumble(RumbleType.kRightRumble, 0);
      return 0.1;
    }else if(x > 15){
      controllerdriver.setRumble(RumbleType.kLeftRumble, 0);
      controllerdriver.setRumble(RumbleType.kRightRumble, 0);
      return 0.3;
    }else{                            // If it finds nothing it won't change direction
      controllerdriver.setRumble(RumbleType.kLeftRumble, 0);
      controllerdriver.setRumble(RumbleType.kRightRumble, 0);
      return 0.0;
    }
  }

  public double directionToTarget()
  {
    NetworkTableEntry tx = limelightTop.getEntry("tx");
    double x = tx.getDouble(0.0);
    x -= LLOffset; // slop, tuning center of target control
    double prop = x / 45;
    if(x > -0.5 && x < 0.5){
      controlleroperator.setRumble(RumbleType.kLeftRumble, 1);
      controlleroperator.setRumble(RumbleType.kRightRumble, 1);
      return 0;
    }else{
      controlleroperator.setRumble(RumbleType.kLeftRumble, 0);
      controlleroperator.setRumble(RumbleType.kRightRumble, 0);   
    }
    return prop;
  }

  public void align()
  {
    // if(controlleroperator.getXButtonPressed()){
    //   LLOffset -= 0.5;
    // }else if(controlleroperator.getBButtonPressed()){
    //   LLOffset += 0.5;
    // }
    System.out.println("Offset: " + LLOffset);
    double autoDirection = directionToTarget();
    drive(0, -autoDirection, false);
  }

  public double autonomousAlign()
  {
    double autoDirection = directionToTarget();
    drive(0, -autoDirection, false);
    return autoDirection;
  }

  //
  //
  //                      DRIVE CODE
  //
  //

  public void drive(double desiredSpeed, double direction, boolean tracON){ // Both desiredSpeed and direction should be sent as positive values as you would expect
    if(true){
      double currentSpeedAvg = getDriveSpeed() / topSpeed;
      if(desiredSpeed > (currentSpeedAvg + maxSpeedDiff)){
        desiredSpeed = (currentSpeedAvg + maxSpeedDiff);
      }else if(desiredSpeed < (currentSpeedAvg - minSpeedDiff)){
        desiredSpeed = (currentSpeedAvg - minSpeedDiff);
      }
    }

    if(trac){
      direction -= directionToBalls();
    }

    double leftSpeedFinal = desiredSpeed - direction;
    double rightSpeedFinal = desiredSpeed + direction;

    if(driveWheelsAreTalonsAndNotSparks){
      backLeftT.set(talonFXMode, leftSpeedFinal);
      frontLeftT.set(talonFXMode, leftSpeedFinal);
      backRightT.set(talonFXMode, rightSpeedFinal);
      frontRightT.set(talonFXMode, rightSpeedFinal);
    }else{
      backLeftS.set(-leftSpeedFinal);
      frontLeftS.set(-leftSpeedFinal);
      backRightS.set(rightSpeedFinal);
      frontRightS.set(rightSpeedFinal);
    }
  }

  //
  //
  //                   SHOOT CODE
  //
  //

  public void shoot(double targetRate)
  {
    double rate = shootEncoder.getRate();
    double speedChange = (targetRate - rate) * 0.0001;
    shootPower += speedChange;
    shootPower = Math.max(0.3, Math.min(1.0, shootPower));
    if(targetRate == 0)
      shootPower = 0;

    //                         Epic shoot belt deciding algorithm code
    // An epic algorithm designed by a genius with intelligence sometimes believed to be superior
    // to Albert Einstein himself. Or Steven ? Stephen ? Stefan ? Hawking. Whomsoever you would like
    // to compare the dude who wrote this to. So it just decdides who gets control of the belt because
    // like four toddlers playing Minecraft, not everyone can be in creative mode, or someone will go 
    // home crying. In this case it would be team 3257 from Roseville, CA.
    //


    if(autoisbeingdumb){
      belt.set(talonFXMode, beltSpeed);
    }else if(rate < (targetRate + 15) && rate > (targetRate - 15)){
      if(longshot)
        belt.set(talonFXMode, -1.0); // Long shot gets full power. yee haw
      else
        belt.set(talonFXMode, beltSpeed - 0.2);
    }else if(controllerdriver.getTriggerAxis(GenericHID.Hand.kRight) > 0.5){
      belt.set(talonFXMode, beltSpeed);
    }else{
      belt.set(talonFXMode, 0.0);
    }
    shooterD.set(shootPower);
    shooterP.set(shootPower);
  }

  public void stopShooter(){
    shooterD.set(0);
    shooterP.set(0);
  }

  //
  //
  //                  TELEOP PERIODIC
  //
  //

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {

    autoisbeingdumb = false; //resets because we aren't in auto anymore silly goose

    //
    //
    //                      DRIVER CONTROLLER CODE
    //
    //

    double driverJoystickY = -controllerdriver.getY(GenericHID.Hand.kLeft);
    double driverJoystickX = -controllerdriver.getX(GenericHID.Hand.kRight) * 0.4;
    if (Math.abs(driverJoystickY) < 0.1) // Zero joysticks
      driverJoystickY = 0;
    
    if (Math.abs(driverJoystickX) < 0.1) 
      driverJoystickX = 0;

    // Toggle Swtiches for Driver
    if(controllerdriver.getXButtonPressed())
      trac = !trac;


    NetworkTableEntry llx = limelightBottom.getEntry("tx");
    double deltaX = llx.getDouble(0.0);
    if(Math.abs(deltaX) > 1){
      intake.set(-0.5);
    }

    if(controllerdriver.getAButtonPressed()) 
      intakeToggle = !intakeToggle;

    if(intakeToggle)
      intake.set(-intakeSpeed);
    else
      intake.set(0);
    



    //
    //
    //                       OPERATOR CONTROLLER
    //
    //

    double operatorJoystickYLeft = -controlleroperator.getY(GenericHID.Hand.kLeft);
    double operatorJoystickYRight = -controlleroperator.getY(GenericHID.Hand.kRight);

    if(Math.abs(operatorJoystickYLeft) < 0.1) {
      operatorJoystickYLeft = 0;
    }
    winchL.set(talonFXMode, operatorJoystickYLeft);
    if(Math.abs(operatorJoystickYRight) < 0.1) {
      operatorJoystickYRight = 0;
    }
    winchR.set(talonFXMode, operatorJoystickYRight);

    if(controlleroperator.getPOV() == 0) {
      elevator.set(talonFXMode, 0.5);
      winchR.set(talonFXMode, -0.5);
      winchL.set(talonFXMode, -0.5);
    } else if (controlleroperator.getPOV() == 180) {
      elevator.set(talonFXMode, -0.5);
    } else {
      elevator.set(talonFXMode, 0);
    }

    if(controlleroperator.getTriggerAxis(GenericHID.Hand.kRight) > 0.5){ // Complicated algorithm to decide if the right trigger is being held
      shoot = true;
    }else if(controlleroperator.getTriggerAxis(GenericHID.Hand.kRight) < 0.5){
      shoot = false;
    }
    if(lidar.getDistance() < 150 && lidar.getDistance() > 0.0){
      shoot = false;
    }

    if(shoot){
      shoot(shootRate);
      longshot = false;
    }else if(controlleroperator.getBumper(GenericHID.Hand.kRight)){
      longshot = true;
      shoot(810);
    }else{ // driver can only operate the belt manually when not trying to shoot to avoid stutter
      longshot = false;
      stopShooter();
      if(controllerdriver.getTriggerAxis(GenericHID.Hand.kRight) > 0.5) // Complicated algorithm to decide if the left trigger is being held
        runBelt(true, beltSpeed);
      else if(controllerdriver.getBumper(GenericHID.Hand.kRight))
        runBelt(true, -beltSpeed);
      else
        // belt.set(ControlMode.PercentOutput, 0);
        runBelt(false, 0);
    }

    //
    //                Intense Drive Control Algorithms
    // This bit of code basically just checks some button presses to see if anyone
    // else is trying to do anything with the drivetrain that is more important than
    // just manually controlling the robot, such as small adjustments in rotation or
    // the limelight is aligning to the target. It also manages the limelight's led state
    // as FIRST did not appreciate their power. Also fun note because this block of
    // comments has existed for a while and I doubt no one will ever read it, Noah is a loser.
    // Not the one from 2019 that was one of the two Project Managers, the programming
    // one that has been declared a loser. Not for any reason, he just thinks he is cooler than me
    // because he has the side protectors on his sunglasses and I did not get any. (IS THIS STILL TRUE JIMMY?!?!)
    // I find this
    // unfair but I will continue living life to its fullest. Also to end this comment so it looks real
    // the limelight can be weird to work with so maybe play with ignoring the
    // index in the web browser a bit.
    //

    if(controlleroperator.getTriggerAxis(GenericHID.Hand.kLeft) > 0.1){
      limelightTop.getEntry("ledMode").setNumber(3);
    }
    if(controlleroperator.getTriggerAxis(GenericHID.Hand.kLeft) > 0.9){ // Complicated algorithm to decide if the left trigger is being held
      limelightTop.getEntry("ledMode").setNumber(3);
      align();
    }else{
      limelightTop.getEntry("ledMode").setNumber(1);
      drive(driverJoystickY, driverJoystickX, trac); // Actually calls the driving when not aligning to avoid stutter
    }

    // Following lines print out helpful data to drive station
    // double lidarDist = lidar.getDistance();
    // System.out.println("Shooter Power: " + shootPower + " and Lidar Dist: " + lidarDist + " and Belt Speed: " + beltSpeed + " Shoot Rate: " + shootEncoder.getRate());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  {
    // Test code would go here
  }
   
}
