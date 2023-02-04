package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

public class Robot extends TimedRobot {
  WPI_TalonFX left = new WPI_TalonFX(2); // left drive motor
  WPI_TalonFX right = new WPI_TalonFX(0); // right drive motor
  WPI_TalonFX belt = new WPI_TalonFX(1); // belt motor
  WPI_TalonFX intakeInternal = new WPI_TalonFX(3); // internal intake motor
  WPI_TalonFX intakeExternal = new WPI_TalonFX(4); // external intake motor
  DifferentialDrive drive = new DifferentialDrive(left, right);
  XboxController controller = new XboxController(0);
  Timer timer = new Timer(); 
  ADIS16448_IMU gyro = new ADIS16448_IMU(); // RoboRIO-mounted gyroscope
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0,0);
  ProfiledPIDController angleControl = new ProfiledPIDController(0.05, 0.18, 0.013, new TrapezoidProfile.Constraints(200,200));
  double encoderTicksPerMeter = 44000;
  double desiredRobotPosition = 0;
  double desiredRobotAngle = 0;
  // controller inputs
  double leftStickY;
  double leftStickX;
  double rightStickY;
  double rightStickX;
  double leftTrigger;
  double rightTrigger;
  // motor encoder values
  double positionLeft;
  double positionRight;
  double positionBelt;
  double positionInternalIntake;
  double positionExternalIntake;
  double positionAverage;
  double time;
  double angle;
  double robotX;
  double robotY;

  @Override
  public void robotInit() {
    initializeMotors();
    timer.start();
    timer.reset(); // sets the timer to 0
    gyro.calibrate(); // sets the gyro angle to 0 based on the current robot position
    CameraServer.startAutomaticCapture(); // starts the webcam stream
    updateVariables();
    angleControl.setIntegratorRange(-0.8, 0.8);
    angleControl.setTolerance(1);
    angleControl.enableContinuousInput(0, 360);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    timer.reset();
    updateVariables();
  }

  @Override
  public void autonomousPeriodic() {
    updateVariables();
  }

  @Override
  public void teleopInit() {
    timer.reset();
    updateVariables();
  }

  @Override
  public void teleopPeriodic() {
    updateVariables();
    double JoystickTheta = getJoystickTheta(leftStickX, -leftStickY);
    double JoystickR = getJoystickR(leftStickX, -leftStickY);
    SmartDashboard.putNumber("JoystickR", JoystickR);
    SmartDashboard.putNumber("JoystickTheta", JoystickTheta);
    if (JoystickR > 0.1) {
      desiredRobotAngle = JoystickTheta - 90;
      if (desiredRobotAngle < 0) {
        desiredRobotAngle = desiredRobotAngle + 360;
      }
    }

    double rotationSpeed = angleControl.calculate(-angle, new TrapezoidProfile.State(desiredRobotAngle, 0));
    double error = angleControl.getPositionError();
    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("RotationSpeed", rotationSpeed);
    drive.arcadeDrive(0, rotationSpeed);
/* 
    desiredRobotPosition = encoderTicksPerMeter*rightStickY;
    left.set(ControlMode.MotionMagic, desiredRobotPosition);
    right.set(ControlMode.MotionMagic, desiredRobotPosition);
    drive.feed();
*/
    // sets motor speeds based on controller inputs
    // drive.arcadeDrive(-leftStickY, -leftStickX, true);
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

  public double getJoystickTheta(double x, double y) {
    double JoystickTheta;
    if (x != 0) {
      JoystickTheta = Math.atan(y/x)*180/Math.PI;
      if (x < 0) {
        JoystickTheta = 180 + JoystickTheta;    
      } else if (y < 0) {
          JoystickTheta = 360 + JoystickTheta;
      }
    } else if (y >= 0) {
      JoystickTheta = 90;
    } else {
      JoystickTheta = 270;
    }
    return JoystickTheta;
  }

  public double getJoystickR(double x, double y) {
    return Math.sqrt(x*x + y*y);
  }

  public void initializeMotors() {
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
  }

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
  
  // initializes variables and publishes values on dashboard
  public void updateVariables() {
    // updates variables
    leftStickY = controller.getLeftY();
    leftStickX = controller.getLeftX();
    rightStickY = controller.getRightY();
    rightStickX = controller.getRightX();
    leftTrigger = controller.getLeftTriggerAxis();
    rightTrigger = controller.getRightTriggerAxis();
    positionLeft = left.getSelectedSensorPosition(0);
    positionRight = right.getSelectedSensorPosition(0);
    positionBelt = belt.getSelectedSensorPosition(0);
    positionInternalIntake = intakeInternal.getSelectedSensorPosition(0);
    positionExternalIntake = intakeExternal.getSelectedSensorPosition(0);
    time = timer.get();
    angle = gyro.getGyroAngleZ() % 360;

    odometry.update(new Rotation2d(angle*Math.PI/180), positionLeft/encoderTicksPerMeter, positionRight/encoderTicksPerMeter);
    Pose2d robotPosition = odometry.getPoseMeters();
    robotX = robotPosition.getX();
    robotY = robotPosition.getY();
    positionAverage = (positionLeft + positionRight) / 2;
    
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
    SmartDashboard.putNumber("RobotX", robotX);
    SmartDashboard.putNumber("RobotY", robotY);
    SmartDashboard.putNumber("Encoder (Average)", positionAverage);
    SmartDashboard.putNumber("Desired Robot Position",  desiredRobotPosition);
    SmartDashboard.putNumber("Desired Robot Angle", desiredRobotAngle);
  }
}