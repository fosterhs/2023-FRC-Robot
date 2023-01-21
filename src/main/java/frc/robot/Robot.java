package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Timer Clock = new Timer();
  private final XboxController xbox = new XboxController(0);

  private final WPI_TalonFX m_leftMotor = new WPI_TalonFX(2);
  private final WPI_TalonFX m_external = new WPI_TalonFX(1); 
  private final WPI_TalonFX m_rightMotor = new WPI_TalonFX(0);
  private final DifferentialDrive drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
 
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    Clock.start();
    Clock.reset();
    SmartDashboard.putNumber("Clock", 0);
    m_external.setSelectedSensorPosition(0, 0, 30);
    SmartDashboard.putNumber("Encoder", m_external.getSelectedSensorPosition(0));
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        break;
      case kDefaultAuto:
      default:
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
        break;
    }
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