/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * <p>The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 *
 * <p>WARNING: While it may look like a good choice to use for your code if
 * you're inexperienced, don't. Unless you know what you are doing, complex code
 * will be much more difficult under this system. Use TimedRobot or
 * Command-Based instead if you're new.
 */
public class Robot extends SampleRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  private final DifferentialDrive m_robotDrive
      = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
  private final Joystick m_stick = new Joystick(0);
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public Robot() {
    m_robotDrive.setExpiration(0.1);
  }

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto modes", m_chooser);
  }

  /**
   * Select auton mode to trigger a climb
   */
  @Override
  public void autonomous() {
    String autoSelected = m_chooser.getSelected();
    // String autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + autoSelected);

    // MotorSafety improves safety when motors are updated in loops
    // but is disabled here because motor updates are not looped in
    // this autonomous mode.
    m_robotDrive.setSafetyEnabled(false);
    motionmagicclimber();
  }

  /**
   * Runs the motors with arcade steering.
   *
   * <p>If you wanted to run a similar teleoperated mode with an TimedRobot
   * you would write:
   *
   * <blockquote><pre>{@code
   * // This function is called periodically during operator control
   * public void teleopPeriodic() {
   *     myRobot.arcadeDrive(stick);
   * }
   * }</pre></blockquote>
   */
  @Override
  public void operatorControl() {
    m_robotDrive.setSafetyEnabled(true);
    while (isOperatorControl() && isEnabled()) {
      // Drive arcade style
      m_robotDrive.arcadeDrive(-m_stick.getY(), m_stick.getX());

      // The motors will be updated every 5ms
      Timer.delay(0.005);
    }
  }

  /**
   * Runs during test mode.  Used to find max sensor velocity on 1 Talon.
   */
  @Override
  public void test() {
    TalonSRX upDownRear = new TalonSRX(31);
    upDownRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    upDownRear.getSensorCollection().setQuadraturePosition(0, 10);
    upDownRear.set(ControlMode.PercentOutput, 1);
    long startTime = System.currentTimeMillis(); 
    while(System.currentTimeMillis() - startTime < 1000){
      if(System.currentTimeMillis() % 100 == 0){
        System.out.println("Encoder ticks\t" +upDownRear.getSensorCollection().getQuadraturePosition());
      }
    }
    upDownRear.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Initial effort at PID tuning for Motion Magic on climber.
   */
  public void motionmagicclimber(){
    TalonSRX upDownRear = new TalonSRX(31);
    upDownRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    upDownRear.getSensorCollection().setQuadraturePosition(0, 10);
    upDownRear.configMotionAcceleration(4784);
    upDownRear.configMotionCruiseVelocity(1196);
    upDownRear.configPeakOutputForward(1.0);
    upDownRear.configPeakOutputReverse(-1.0);
    upDownRear.config_kP(0, 0);
    upDownRear.config_kI(0, 0);
    upDownRear.config_kD(0, 0);
    upDownRear.config_kF(0, 0.427799073);
    upDownRear.config_IntegralZone(0, 0);
    upDownRear.configClosedLoopPeakOutput(0, 1.0);
    upDownRear.selectProfileSlot(0, 0);
    upDownRear.set(ControlMode.MotionMagic, 8000, DemandType.ArbitraryFeedForward, 1196);
    Timer.delay(2);
    System.out.println("Encoder Position at 2 seconds\t" + upDownRear.getSensorCollection().getQuadraturePosition());
    Timer.delay(2);
    System.out.println("Encoder Position at 4 seconds\t" + upDownRear.getSensorCollection().getQuadraturePosition());
    upDownRear.set(ControlMode.PercentOutput, 0);

   }
}
