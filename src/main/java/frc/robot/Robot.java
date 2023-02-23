// Erik's Main Code Document
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
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
  double endPoint = 1.0;
  double error;
  PIDController pid = new PIDController(0.5, 0, 0);
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

    /* Note: The way this piece of code works is that you set an endpoint that represents the total distance the robot is meant to travel in autonomous. For example I've set it to 1.0, 
    representing 1 meter. Then we subtract that by the avgPosition. The avgPosition is the average of the total distance traveled by the motors recorded. We receive this information 
    from the encoders we've established that gets the sensor's selected position. We use this to represent how much the motors have currently traveled, and by subtracting that from 
    the endPoint, you get how much you have left to travel, which is the error variable. By plugging in the error variable into the arcadeDrive function, as the avgPosition increases 
    while driving, the lower the difference. Such as 5-1=4, when 1 increases to 2, you have 5-2=3. This proportionate decrease in your distance to travel is how slow the robot down 
    as it reaches 1 meter. We plug this value in for xSpeed to tell the roboRio to move the robot forward 1 meter. 0.5 is the constant that tells us how much percent of the motors it
    should be. Here, it should be 50% at all times of the error value, whatever it may be.*/
    /* Note 2: The encoder records the motor rotations in different units. One full rotation of the motors recorded by the encoders is 2048. We've calculated that 45315.0 is the 1
    meter in encoder units. avgPosition essentially is the average of the 2 variables, positionLeft and positionRight. Those variables are basically giving us the current value of 
    encoders at any moment, through functions that gets the selected sensor position in raw units. Since avgPosition is set to equal raw sensor units, by dividing by 45315.0, it'll
    convert the units into meters. Say for example we have exactly 1 meter left to travel. 1 meter will equal 45315.0, so dividing that by itself will get 1. That will proportinately
    work for any other values lower than 1. We do this because arcadeDrive takes in values from -1, 0, and positive 1, so by dividing by 45315, not only can arcadeDrive actually
    read what we're feeding it, but we can convert the units to meters.*/
    error = endPoint - (avgPosition/45315.0); 
    
    double pidError = pid.calculate(avgPosition/45315.0, endPoint);
    m_robotDrive.arcadeDrive(pidError, 0);

   
  }

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
