// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CANSparkMax m_leftDrive = new CANSparkMax(6, MotorType.kBrushed);
  private final CANSparkMax m_leftDrive2 = new CANSparkMax(4, MotorType.kBrushed);
  private final CANSparkMax m_rightDrive2 = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_rightDrive = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_dumpMotor = new CANSparkMax(5, MotorType.kBrushed);
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftDrive, m_leftDrive2);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightDrive, m_rightDrive2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
    // m_rightDrive2.setInverted(true);
    m_leftDrive2.follow(m_leftDrive);
    m_rightDrive2.follow(m_rightDrive);
    m_rightDrive.setIdleMode(IdleMode.kBrake);
    m_leftDrive2.setIdleMode(IdleMode.kBrake);
    m_leftDrive.setIdleMode(IdleMode.kBrake);
    m_leftDrive2.setIdleMode(IdleMode.kBrake);
    CameraServer.startAutomaticCapture();
    m_chooser.addOption(kDefaultAuto, kDefaultAuto);
    m_chooser.addOption(kCustomAuto, kCustomAuto);
    SmartDashboard.putData("Auto Choice",m_chooser);


  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    m_autoSelected=m_chooser.getSelected();


  }
  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch(m_autoSelected){
      case kCustomAuto:
        //TODO: Put Auto code here
        break;
      default:
        break;
    }
    // Drive for 2 seconds
    if (m_timer.get() < 1.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      // m_robotDrive.arcadeDrive(0.5, 0.0, false);
     m_dumpMotor.set(0.2);
    } 
    else if(m_timer.get() > 1.0 && m_timer.get() < 2.0)
    {
      m_dumpMotor.set(-0.2);
      // m_robotDrive.arcadeDrive(0.5, -0.5, false);
    }
    // else if (m_timer.get() > 2.0 && m_timer.get() < 4.0){
    //   m_robotDrive.arcadeDrive(0.5, 0, false);
    // }
    else {
      m_dumpMotor.set(0);
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

 // 2023-03-23, DMW, the following value represents applying a minimum weighting or
    // no reduction the speed of left/right turning
  public static final double MIN_TURN_WEIGHT_MULT_VAL = 1.0d;
    // 2023-03-23, DMW< the following value represents applying a maximum weighting or
    // REDUCTION in the speed of left/right turning
  public static final double MAX_TURN_WEIGHT_MULT_VAL = 0.90;
  public static double turnbyspeedweight(double drive_speed, double turn_speed)
  {
    double abs_drive = Math.abs(drive_speed);
    // double abs_turn = Math.abs(turn_speed);

    return turn_speed * (MIN_TURN_WEIGHT_MULT_VAL - ((MIN_TURN_WEIGHT_MULT_VAL - MAX_TURN_WEIGHT_MULT_VAL) * abs_drive));
  }

  

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // // 2023-03-23, DMW, the following value represents applying a minimum weighting or
    // // no reduction the speed of left/right turning
    final double MIN_TURN_WEIGHT_MULT_VAL = 1.0d;
    // // 2023-03-23, DMW< the following value represents applying a maximum weighting or
    // // REDUCTION in the speed of left/right turning
    final double MAX_TURN_WEIGHT_MULT_VAL = 0.85;

   m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
  // 2023-03-23, DB, removed negative sign from this; Added weight to x  
  //    Tested .75 left and right 
    double y = -m_controller.getLeftY();
    double x = m_controller.getRightX();
    double turn_weight = turnbyspeedweight(y, x);
    // m_robotDrive.arcadeDrive(-m_controller.getLeftY(), m_controller.getRightX()*MAX_TURN_WEIGHT_MULT_VAL);
    m_robotDrive.arcadeDrive(y, turn_weight);
    m_dumpMotor.set((m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis()) * 0.2);

    // TO-DO: 2023-03-23, DMW, complete figuring out the weighting function and update the code above

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
