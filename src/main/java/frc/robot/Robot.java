package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Robot extends TimedRobot {
  private final WPI_TalonFX left = new WPI_TalonFX(2); // left drive motor
  private final WPI_TalonFX external = new WPI_TalonFX(1); // external motor
  private final WPI_TalonFX right = new WPI_TalonFX(0); // right drive motor
  private final WPI_VictorSPX intake = new WPI_VictorSPX(2); // intake motor
  private final WPI_VictorSPX belt = new WPI_VictorSPX(4); // belt motor
  private final DifferentialDrive drive = new DifferentialDrive(left, right);
  private final XboxController controller = new XboxController(0);
  private final Timer timer = new Timer(); 
  private final ADIS16448_IMU gyro = new ADIS16448_IMU(); // RoboRIO-mounted gyroscope
 
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    initialize();
  }

  @Override
  public void autonomousPeriodic() {
    // updates variables
    double time = timer.get();
    double positionExternal = external.getSelectedSensorPosition(0);
    double positionLeft = left.getSelectedSensorPosition(0);
    double positionRight = right.getSelectedSensorPosition(0);
    double angle = gyro.getGyroAngleZ();

    // publishes updated variables to the dashboard
    SmartDashboard.putNumber("Clock",  time);
    SmartDashboard.putNumber("Encoder (External)", positionExternal);
    SmartDashboard.putNumber("Encoder (Left)", positionLeft);
    SmartDashboard.putNumber("Encoder (Right)", positionRight);
    SmartDashboard.putNumber("Angle", angle);
   
    // runs the external motor at 50% until one motor rotation
    if (positionLeft < 45315.0 ) {
      drive.arcadeDrive(0.25, 0);
      } else {
      drive.arcadeDrive(0, 0);
      }
    }

  @Override
  public void teleopInit() {
    initialize();
  }

  @Override
  public void teleopPeriodic() {
    // updates variables
    double leftStickY = controller.getLeftY();
    double leftStickX = controller.getLeftX();
    double rightTrigger = controller.getRightTriggerAxis();
    double leftTrigger = controller.getLeftTriggerAxis();
    double rightStickY = controller.getRightY();
    double rightStickX = controller.getRightX();
    double positionExternal = external.getSelectedSensorPosition(0);
    double positionLeft = left.getSelectedSensorPosition(0);
    double positionRight = right.getSelectedSensorPosition(0);
    double angle = gyro.getGyroAngleZ();
    
    // publishes updated variables to the dashboard
    SmartDashboard.putNumber("leftStickY", leftStickY);
    SmartDashboard.putNumber("leftStickX", leftStickX);
    SmartDashboard.putNumber("rightStickY", rightStickY);
    SmartDashboard.putNumber("rightStickX", rightStickX);
    SmartDashboard.putNumber("rightTrigger", rightTrigger);
    SmartDashboard.putNumber("leftTrigger", leftTrigger);
    SmartDashboard.putNumber("Encoder (External)", positionExternal);
    SmartDashboard.putNumber("Encoder (Left)", positionLeft);
    SmartDashboard.putNumber("Encoder (Right)", positionRight);
    SmartDashboard.putNumber("Angle", angle);
    
    // sets the drive motors based on the left joystick inputs
    drive.arcadeDrive(-leftStickY, -leftStickX, true);
    belt.set(-rightTrigger);
    intake.set(-leftTrigger);
    external.set(ControlMode.Position, 2048*rightStickY);
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

  // runs manufacturer recommended startup commands for Falcon 500 motors. Should be run at startup for all motors.
  public void motorStartUp(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0 , 30);
    motor.configNeutralDeadband(0.01, 30);
    motor.setSensorPhase(false);
    motor.setInverted(false);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
    motor.configNominalOutputForward(0, 30);
    motor.configNominalOutputReverse(0, 30);
    motor.configPeakOutputForward(1, 30);
    motor.configPeakOutputReverse(-1, 30);
    motor.selectProfileSlot(0, 0);
    motor.configOpenloopRamp(1);
    motor.setSelectedSensorPosition(0, 0, 30);
  }

  public void initialize() {
    motorStartUp(external);

    // PID coefficients 
    external.config_kF(0, 0, 30); 
    external.config_kP(0, 1, 30);
    external.config_kI(0, 0.005, 30);
    external.config_kD(0, 10, 30);

    // motion magic parameters
    external.configMotionCruiseVelocity(20000, 30);
    external.configMotionAcceleration(6000, 30);

    // brake versus coast
    external.setNeutralMode(NeutralMode.Brake); 

    motorStartUp(left);
    left.config_kF(0, 0, 30);
    left.config_kP(0, 1, 30);
    left.config_kI(0, 0.005, 30);
    left.config_kD(0, 10, 30);
    left.configMotionCruiseVelocity(20000, 30);
    left.configMotionAcceleration(6000, 30);
    left.setNeutralMode(NeutralMode.Brake);

    motorStartUp(right);
    right.config_kF(0, 0, 30);
    right.config_kP(0, 1, 30);
    right.config_kI(0, 0.005, 30);
    right.config_kD(0, 10, 30);
    right.configMotionCruiseVelocity(20000, 30);
    right.configMotionAcceleration(6000, 30);
    right.setNeutralMode(NeutralMode.Brake);
    right.setInverted(true);

    gyro.calibrate(); // sets the gyro angle to 0 based on the current robot position
    timer.start();
    timer.reset(); // sets the timer to 0
    CameraServer.startAutomaticCapture(); // starts the webcam stream

    // initializes values on dashboard
    SmartDashboard.putNumber("Angle", gyro.getGyroAngleZ());
    SmartDashboard.putNumber("Clock", timer.get());
    SmartDashboard.putNumber("Encoder (External)", external.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Encoder (Left)", left.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Encoder (Right)", right.getSelectedSensorPosition(0));
  }
}