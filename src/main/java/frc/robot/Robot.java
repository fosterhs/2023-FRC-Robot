package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Timer Clock = new Timer();
  private final XboxController xbox = new XboxController(0);

  private final WPI_TalonFX m_leftMotor = new WPI_TalonFX(2);
  private final WPI_TalonFX m_external = new WPI_TalonFX(1); 
  private final WPI_TalonFX m_rightMotor = new WPI_TalonFX(0);
  private final DifferentialDrive drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
 
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    Clock.start();
    Clock.reset();
    SmartDashboard.putNumber("Clock", 0);
    m_external.setSelectedSensorPosition(0, 0, 30);
    SmartDashboard.putNumber("Encoder", m_external.getSelectedSensorPosition(0));
  }

  @Override
  public void autonomousPeriodic() {

    drive.arcadeDrive(0,0);
    double time = Clock.get();
    SmartDashboard.putNumber("Clock",  time);
    if (time < 2) {
      m_external.set(ControlMode.PercentOutput, 0.1);
    } 
    if (time > 2) {
      m_external.set(ControlMode.PercentOutput, 0);
    }
    SmartDashboard.putNumber("Encoder", m_external.getSelectedSensorPosition(0));
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double leftStickY = xbox.getLeftY();
    double leftStickX = xbox.getLeftX();
    drive.arcadeDrive(leftStickX, leftStickY);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}