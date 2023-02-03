package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

/* import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc; */

public class Robot extends TimedRobot {
  private final WPI_TalonFX m_rightMotor = new WPI_TalonFX(2); // Note - Creates a new motor, because the motor exists physically not digitally yet.
  private final WPI_TalonFX m_leftMotor = new WPI_TalonFX(0); // Note - Creates another new motor
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor); // Note - Creates an instance of differential drive that takes in inputs of the left and right motors. */
  private final XboxController m_driverController = new XboxController(0); // Note - Creates an Xbox Controller object/instance. 
  private final Timer elapsedTime = new Timer();
  private final ADIS16448_IMU gyro = new ADIS16448_IMU();
  // Thread m_visionThread;

  @Override
  public void robotInit() {
    m_rightMotor.setInverted(true); // Note - Inverts the motors on one side because of the orientation physically. If not inverted, then it would turn.
    CameraServer.startAutomaticCapture(); 
  }

  @Override
  public void robotPeriodic() {
    /* Note - Arcade Drive function that takes in the controllers left Y axis values and right X values to move the robot. The controllers sticks will return a value between 1 
    and -1, which when inputted into the function, will give the motors speed/movement. Motor movement is simple. It's 0% power at default, meaning it has 0% speed, but if you
    give the motor 20% power, it has 20% speed. Similarlly if you take the 1 and -1 values the sticks give off and assign them to the motors, you have motor speed. Although 1 wouuld
    act as 100%. This function, which is built into the library, has shortened this down for us so we only have to call one function. Because we gave the variable m_robotDrive the motor inputs, we just call the function without having to reference the motors. */
    m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX()); // Note - Takes in the controller's left Y axis & right x axis values to move the robot. LeftY moves back & forth, representing speed, RightX turns, respresenting rotation.
    SmartDashboard.putNumber("Left Joystick's Y Axis Value:" , m_driverController.getLeftY());
    SmartDashboard.putNumber("Right Joystick's X Axis Value:", m_driverController.getRightX());
  }

  @Override
  public void autonomousInit() {
    m_robotDrive.arcadeDrive(0, 0);
    elapsedTime.start();
    elapsedTime.reset();
  }

  @Override
  public void autonomousPeriodic() {
    double matchTime = elapsedTime.get();
    double positionLeft = m_leftMotor.getSelectedSensorPosition(0);
    double positionRight = m_rightMotor.getSelectedSensorPosition(0);
    double angle = gyro.getGyroAngleZ();
    double avgPosition = (positionLeft + positionRight)/2;
    SmartDashboard.putNumber("Match Time:", matchTime);
    /* if (matchTime < 4) {
      m_leftMotor.set(ControlMode.PercentOutput, 0.1);
      m_rightMotor.set(ControlMode.PercentOutput, -0.1);
    } else if (matchTime > 4) {
      m_leftMotor.set(ControlMode.PercentOutput, 0);
      m_rightMotor.set(ControlMode.PercentOutput, 0);
    } */    

    autoMove(1, 0.5, 1);
  }

  public void autoMove(double distance, double speed, double endPosition) {
    if (endPosition < (45315.0 * distance)) {
      m_robotDrive.arcadeDrive(speed, 0);
    } else {
      m_robotDrive.arcadeDrive(0, 0);
    }
  }

  /* public void turn (double desiredAngle, double speed, double angle) {
    if (angle < desiredAngle)
  } */

  @Override
  public void teleopInit() {
    m_robotDrive.arcadeDrive(0, 0);
  }

  @Override
  public void teleopPeriodic() {}

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
