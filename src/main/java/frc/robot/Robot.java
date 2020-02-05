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
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

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

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  {

  }
}

/**
 * This class initializes the controllers our robot uses
 */
private class ControllerPorts
{
    controllerdriver = new XboxController(0);
    controlleroperator = new XboxController(1);
}

/**
 *  This class initializes the sensors our robot uses
 */
private class SensorPorts
{
    navx = new AHRS(I2C.Port 1);

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
}

/**
 * This class initializes the motors our robot uses
 */
private class MotorPorts
{
    BackRight = new Spark(0);
    FrontRight = new Spark(1);
    BackLeft = new Spark(2);
    FrontLeft = new Spark(3);

    Intake = new Spark(4);

    Belt1 = new Spark(5);
    Belt2 = new Spark(6);
    Belt3 = new Spark(7);
    Belt4 = new Spark(8);
    Loader = new Spark(9);

    Arm = new PWMTalonSRX(0);
}


