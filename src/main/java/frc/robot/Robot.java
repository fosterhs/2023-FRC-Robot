package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Robot extends TimedRobot {
  private final WPI_TalonFX left = new WPI_TalonFX(2); // left drive motor
  private final WPI_TalonFX right = new WPI_TalonFX(0); // right drive motor
  private final WPI_TalonFX belt = new WPI_TalonFX(1); // belt motor
  private final WPI_TalonFX intakeInternal = new WPI_TalonFX(3); // internal intake motor
  private final WPI_TalonFX intakeExternal = new WPI_TalonFX(4); // external intake motor
  private final DifferentialDrive drive = new DifferentialDrive(left, right);
  private final XboxController controller = new XboxController(0);
  private final Timer timer = new Timer(); 
  private final ADIS16448_IMU gyro = new ADIS16448_IMU(); // RoboRIO-mounted gyroscope

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture(); // starts the webcam stream
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    initialize();
  }

  @Override
  public void autonomousPeriodic() {
    // updates variables
    double positionLeft = left.getSelectedSensorPosition(0);
    double positionRight = right.getSelectedSensorPosition(0);
    double positionBelt = belt.getSelectedSensorPosition(0);
    double positionInternalIntake = intakeInternal.getSelectedSensorPosition(0);
    double positionExternalIntake = intakeExternal.getSelectedSensorPosition(0);
    double time = timer.get();
    double angle = gyro.getGyroAngleZ();

    // publishes updated variables to the dashboard
    SmartDashboard.putNumber("Encoder (Left)", positionLeft);
    SmartDashboard.putNumber("Encoder (Right)", positionRight);
    SmartDashboard.putNumber("Encoder (Belt)", positionBelt);
    SmartDashboard.putNumber("Encoder (External Intake)", positionExternalIntake);
    SmartDashboard.putNumber("Encoder (Internal Intake)", positionInternalIntake);
    SmartDashboard.putNumber("Clock",  time);
    SmartDashboard.putNumber("Angle", angle);
   
    // runs the drive motors at 25% until the robot has traveled 1 meter
    moveDistance(1, 0.25, positionLeft);
  }

    // Move function unit for distance in meters; Bug - Moving two meters won't work because distance is not reset
  public void moveDistance(double distance, double speed, double position) {
    if (position < 45315.0 * distance) {
     drive.arcadeDrive(speed, 0);
    } else {
     drive.arcadeDrive(0,0);
    }
  }
  
  // Same bug as move function
  public void rotate(double desiredAngle, double speed, double angle) {
    if (angle < desiredAngle + 1) {
     drive.arcadeDrive(0, speed);
    } else {
     drive.arcadeDrive(0,0);
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
    double rightStickY = controller.getRightY();
    double rightStickX = controller.getRightX();
    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();
    double positionLeft = left.getSelectedSensorPosition(0);
    double positionRight = right.getSelectedSensorPosition(0);
    double positionBelt = belt.getSelectedSensorPosition(0);
    double positionInternalIntake = intakeInternal.getSelectedSensorPosition(0);
    double positionExternalIntake = intakeExternal.getSelectedSensorPosition(0);
    double time = timer.get();
    double angle = gyro.getGyroAngleZ();
    
    // publishes updated variables to the dashboard
    SmartDashboard.putNumber("leftStickY", leftStickY);
    SmartDashboard.putNumber("leftStickX", leftStickX);
    SmartDashboard.putNumber("rightStickY", rightStickY);
    SmartDashboard.putNumber("rightStickX", rightStickX);
    SmartDashboard.putNumber("rightTrigger", rightTrigger);
    SmartDashboard.putNumber("leftTrigger", leftTrigger);
    SmartDashboard.putNumber("Encoder (Left)", positionLeft);
    SmartDashboard.putNumber("Encoder (Right)", positionRight);
    SmartDashboard.putNumber("Encoder (Belt)", positionBelt);
    SmartDashboard.putNumber("Encoder (External Intake)", positionExternalIntake);
    SmartDashboard.putNumber("Encoder (Internal Intake)", positionInternalIntake);
    SmartDashboard.putNumber("Clock",  time);
    SmartDashboard.putNumber("Angle", angle);
    
    // sets the drive motors based on the left joystick inputs
    drive.arcadeDrive(-leftStickY, -leftStickX, true);
    belt.set(-rightTrigger);
    intakeExternal.set(leftTrigger);
    intakeInternal.set(leftTrigger);
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
    motorStartUp(left);
    //PID coefficients
    left.config_kF(0, 0, 30);
    left.config_kP(0, 1, 30);
    left.config_kI(0, 0.005, 30);
    left.config_kD(0, 10, 30);
    // motion magic parameters
    left.configMotionCruiseVelocity(20000, 30);
    left.configMotionAcceleration(6000, 30);
    // brake vs coast
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

    motorStartUp(belt);
    belt.config_kF(0, 0, 30); 
    belt.config_kP(0, 1, 30);
    belt.config_kI(0, 0.005, 30);
    belt.config_kD(0, 10, 30);
    belt.configMotionCruiseVelocity(20000, 30);
    belt.configMotionAcceleration(6000, 30);
    belt.setNeutralMode(NeutralMode.Brake); 

    motorStartUp(intakeInternal);
    intakeInternal.config_kF(0, 0, 30);
    intakeInternal.config_kP(0, 1, 30);
    intakeInternal.config_kI(0, 0.005, 30);
    intakeInternal.config_kD(0, 10, 30);
    intakeInternal.configMotionCruiseVelocity(20000, 30);
    intakeInternal.configMotionAcceleration(6000, 30);
    intakeInternal.setNeutralMode(NeutralMode.Brake);

    motorStartUp(intakeExternal);
    intakeExternal.config_kF(0, 0, 30);
    intakeExternal.config_kP(0, 1, 30);
    intakeExternal.config_kI(0, 0.005, 30);
    intakeExternal.config_kD(0, 10, 30);
    intakeExternal.configMotionCruiseVelocity(20000, 30);
    intakeExternal.configMotionAcceleration(6000, 30);
    intakeExternal.setNeutralMode(NeutralMode.Brake);

    timer.start();
    timer.reset(); // sets the timer to 0
    gyro.calibrate(); // sets the gyro angle to 0 based on the current robot position

    // initializes values on dashboard
    SmartDashboard.putNumber("Encoder (Left)", left.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Encoder (Right)", right.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Encoder (Belt)", belt.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Encoder (Internal Intake)", intakeInternal.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Encoder (External Intake)", intakeExternal.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Clock", timer.get());
    SmartDashboard.putNumber("Angle", gyro.getGyroAngleZ());
  }
}