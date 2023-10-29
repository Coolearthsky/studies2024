// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.arm.ArmTrajectory;
import frc.robot.armMotion.ArmAngles;
import frc.robot.armMotion.ArmKinematics;

public class Robot extends TimedRobot {
  public static class Config {
    public double softStop = -0.594938;
    public double kUpperArmLengthM = 0.92;
    public double kLowerArmLengthM = 0.93;
    public double filterTimeConstantS = 0.06; // TODO: tune the time constant
    public double filterPeriodS = 0.02;
    public double safeP = 2.5;
    public double safeI = 0;
    public double safeD = 0;
    public double normalLowerP = 2;
    public double normalLowerI = 0;
    public double normalLowerD = 0.1;
    public double normalUpperP = 2;
    public double normalUpperI = 0;
    public double normalUpperD = 0.05;
    public double tolerance = 0.001;
  }

  private final Config m_config = new Config();
  private final ArmKinematics m_kinematics = new ArmKinematics(.93,.92);
  private Command m_autonomousCommand;
  private ArmTrajectory m_trajec;
  private ArmKinematics m_armKinematicsM;
  private LinearFilter m_lowerMeasurementFilter;
  private LinearFilter m_upperMeasurementFilter;
  private PIDController m_lowerController;
  private PIDController m_upperController;
  private FRCNEO lowerArmMotor;
  private FRCNEO upperArmMotor;
  private AnalogInput lowerArmInput;
  private AnalogInput upperArmInput;
  private AnalogEncoder lowerArmEncoder;
  private AnalogEncoder upperArmEncoder;
  private ArmAngles m_reference;
  private ArmAngles m_feedForward;
  double u1;
  double u2;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_trajec = new ArmTrajectory(this, new Translation2d(.6, .6),m_kinematics);
    m_armKinematicsM = new ArmKinematics(m_config.kLowerArmLengthM, m_config.kUpperArmLengthM);
    m_lowerMeasurementFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
    m_upperMeasurementFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
    m_lowerController = new PIDController(m_config.normalLowerP, m_config.normalLowerI, m_config.normalLowerD);
    m_upperController = new PIDController(m_config.normalUpperP, m_config.normalUpperI, m_config.normalUpperD);
    m_lowerController.setIntegratorRange(1, 1);
    m_upperController.setIntegratorRange(1, 1);
    m_lowerController.setTolerance(m_config.tolerance);
    m_upperController.setTolerance(m_config.tolerance);

    lowerArmMotor = new FRCNEO.FRCNEOBuilder(4)
        .withInverted(false)
        .withSensorPhase(false)
        .withTimeout(10)
        .withCurrentLimitEnabled(true)
        .withCurrentLimit(1)
        .withPeakOutputForward(0.5)
        .withPeakOutputReverse(-0.5)
        .withNeutralMode(IdleMode.kBrake)
        .build();

    upperArmMotor = new FRCNEO.FRCNEOBuilder(30)
        .withInverted(false)
        .withSensorPhase(false)
        .withTimeout(10)
        .withCurrentLimitEnabled(true)
        .withCurrentLimit(1)
        .withPeakOutputForward(0.5)
        .withPeakOutputReverse(-0.5)
        .withNeutralMode(IdleMode.kBrake)
        .withForwardSoftLimitEnabled(false)
        .build();
        upperArmMotor.setkP(1);
        upperArmMotor.setkI(0);
        upperArmMotor.setkD(1);
        upperArmMotor.setkF(0);
        lowerArmMotor.setkP(1);
        lowerArmMotor.setkI(0);
        lowerArmMotor.setkD(1);
        lowerArmMotor.setkF(0);
    lowerArmInput = new AnalogInput(1);
    lowerArmEncoder = new AnalogEncoder(lowerArmInput);
    upperArmInput = new AnalogInput(0);
    upperArmEncoder = new AnalogEncoder(upperArmInput);
    m_reference = getMeasurement();
    ArmAngles e = new ArmAngles(0, 0);
    m_feedForward = e;
    m_robotContainer = new RobotContainer();
  }

  private double getLowerArm() {
    double x = (lowerArmEncoder.getAbsolutePosition() - 0.861614) * 360;
    return (-1.0 * x) * Math.PI / 180;
  }

  /** Upper arm angle (radians), 0 up, positive forward. */
  private double getUpperArm() {
    double x = (upperArmEncoder.getAbsolutePosition() - 0.266396) * 360;
    return x * Math.PI / 180;
  }

  public void setReference(ArmAngles reference) {
    m_reference = reference;
  }
  public void setFeedForward(ArmAngles feedForward) {
    m_feedForward = feedForward;
  }

  public ArmAngles getMeasurement() {
    return new ArmAngles(
        m_lowerMeasurementFilter.calculate(getLowerArm()),
        m_upperMeasurementFilter.calculate(getUpperArm()));
  }

  @Override
  public void robotPeriodic() {
    m_trajec.execute();
    ArmAngles measurement = getMeasurement();
    double lowerControllerOutput = m_lowerController.calculate(measurement.th1, m_reference.th1);
    double lowerFeedForward = m_feedForward.th1/(Math.PI*2);
    u1 = lowerControllerOutput+lowerFeedForward;
    double upperControllerOutput = m_upperController.calculate(measurement.th2, m_reference.th2);
    double upperFeedForward = m_feedForward.th2/(Math.PI*2);
    u2 = upperControllerOutput+upperFeedForward;
    SmartDashboard.putNumber("Lower Encoder: ", measurement.th1);
    SmartDashboard.putNumber("Lower Ref: ", m_reference.th1);
    SmartDashboard.putNumber("Upper Encoder: ", measurement.th2);
    SmartDashboard.putNumber("Upper Ref: ", m_reference.th2);
    SmartDashboard.putNumber("Output Upper: ", u1);
    SmartDashboard.putNumber("Output Lower: ", u2);
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
   m_robotContainer.scheduleAuton(this,m_kinematics);
  }

  private double soften(double x) {
    if (softBand(x))
      return 0;
    return x;
  }

  private boolean softBand(double x) {
    if (getLowerArm() <= m_config.softStop && x < 0)
      return false;
    return true;
  }

  @Override
  public void autonomousPeriodic() {
    lowerArmMotor.set(u1);
    upperArmMotor.set(u2);
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    m_trajec.initialize();
    m_lowerController = new PIDController(m_config.normalLowerP, m_config.normalLowerI, m_config.normalLowerD);
    m_upperController = new PIDController(m_config.normalUpperP, m_config.normalUpperI, m_config.normalUpperD);
    m_lowerController.setTolerance(m_config.tolerance);
    m_upperController.setTolerance(m_config.tolerance);
    lowerArmMotor.setCurrentLimit(1);
    upperArmMotor.setCurrentLimit(1);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // System.out.println(u1);
    // System.out.println(u2);
    lowerArmMotor.driveVelocity(60);
    // upperArmMotor.set(u2);
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    XboxController controller = new XboxController(0);
    upperArmMotor.set(0);
    lowerArmMotor.set(0);
    if (controller.getAButton()) {
    lowerArmMotor.set(.1);}
    if (controller.getBButton()) {
    // upperArmMotor.set(.1);
}
    if (controller.getXButton()) {
    lowerArmMotor.set(-.1);}
    if (controller.getYButton()) {
    // upperArmMotor.set(-.1);
}
  }

  @Override
  public void testExit() {
  }
}
