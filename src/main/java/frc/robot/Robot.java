// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//careers.j&j.com 

package frc.robot;

//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder; 
//import edu.wpi.first.wpilibj.XboxController;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// camera methods
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.networktables.NetworkTable;
// intake - pneumatics
import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DigitalInput;
//controller
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;


public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;

  public static final int can0 = 0;
  public static final int can1 = 1;
  public static final int can2 = 2;
  public static final int can3 = 3;
  public static final int can4 = 4;
  public static final int can5 = 5;

// controller
  Joystick joy1 = new Joystick(0); //inputs for Joystick
  Joystick joy2 = new Joystick(1);// inputs for Joystick 2

  //motors
  WPI_VictorSPX leftFrontMotor = new WPI_VictorSPX(can1);
  WPI_VictorSPX rightFrontMotor = new WPI_VictorSPX(can3);
  WPI_VictorSPX leftRearMotor = new WPI_VictorSPX(can0);
  WPI_VictorSPX rightRearMotor = new WPI_VictorSPX(can2);

  MotorController rightSide = new MotorControllerGroup(rightFrontMotor, rightRearMotor);
  MotorController leftSide = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
  DifferentialDrive _drive = new DifferentialDrive(leftSide, rightSide);

  //motor variables
  NeutralMode currentModeBrake = NeutralMode.Coast;
  boolean isArcadeDrive = true;

  //encoders
  Encoder encoder1 = new Encoder(0,1);
  
  double kP = 0;
  double kI = 0;
  double kD = 0;
  PIDController pid = new PIDController(kP, kI, kD);

  
  @Override
  public void robotInit() {
    System.out.println("robotInit");
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftRearMotor.setNeutralMode(NeutralMode.Coast); 
    rightFrontMotor.setNeutralMode(NeutralMode.Coast); 
    rightRearMotor.setNeutralMode(NeutralMode.Coast);  
    
    CameraServer.startAutomaticCapture();
    CvSink cvSink = CameraServer.getVideo();

    encoder1.reset();
// Creates the CvSource and MjpegServer [2] and connects them
// CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);

//     UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
// try (MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0", 1181)) {
//   cameraServer.setSource(usbCamera);
// }
// try (CvSink vSink = new CvSink("opencv_USB Camera 0")) {
//   cvSink.setSource(usbCamera);
// }
// try (CvSource OutputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30)) {
// }
// try (MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182)) {
//   mjpegServer2.setSource(outputStream);
// }
  }
  

  @Override
  public void robotPeriodic() {
    // System.out.println("robotPeriodic");
  }
  private double startTime;

@Override
 public void autonomousInit() {
  System.out.println("autonomousInit");
 startTime = Timer.getFPGATimestamp();
startTime = Timer.getFPGATimestamp();
}

   @Override
  public void autonomousPeriodic() {
   Double time = Timer.getFPGATimestamp();

   leftFrontMotor.setNeutralMode(NeutralMode.Brake);
   rightFrontMotor.setNeutralMode(NeutralMode.Brake);
   leftRearMotor.setNeutralMode(NeutralMode.Brake);
  rightRearMotor.setNeutralMode(NeutralMode.Brake);
//goes back
    if (time - startTime < 3) {
      leftSide.set(-0.4);
      rightSide.set(0.4);
    }
//go forward
 else if (time - startTime < 8){
  leftSide.set(0.3);                                              
  rightSide.set(-0.3);
}
//turn right
 else if (time - startTime < 10){
  leftSide.set(-0.3);
  rightSide.set(-0.3);
 } 
 //go foward
 else if(time - startTime < 12){
  leftSide.set(0.4);                                              
  rightSide.set(-0.4);
 }
 //turn left
 else if(time - startTime < 14){
  leftSide.set(0.3);
  rightSide.set(0.3);
 }
  else {
  leftSide.set(0);
  rightSide.set(0);
 }
 
    
  }

  @Override
  public void teleopInit() {
    System.out.println("teleopInit");
    
 /* factory default values */
   leftFrontMotor.configFactoryDefault();
   rightFrontMotor.configFactoryDefault();
   leftRearMotor.configFactoryDefault();
   rightRearMotor.configFactoryDefault();
  
  //This is to put motors in brake mode. I literally can't tell what this does.
   leftFrontMotor.setNeutralMode(NeutralMode.Brake);
   rightFrontMotor.setNeutralMode(NeutralMode.Brake);
   leftRearMotor.setNeutralMode(NeutralMode.Brake);
  rightRearMotor.setNeutralMode(NeutralMode.Brake);


 /* flip values so robot moves forward when stick-forward/LEDs-green */
    leftSide.setInverted(true); // <<<<<< Adjust this
    rightSide.setInverted(false); // <<<<<< Adjust this

    }    

  @Override
  public void teleopPeriodic() {
    //gets raw axis on the joystick
    double xSpeed = joy1.getRawAxis(1);
    double zRotation = joy1.getRawAxis(2);
    double tankLeft=joy1.getRawAxis(1);
    double tankRight=joy1.getRawAxis(3);

//System.out.println( "Speed: " + xSpeed +"\nRotation: " + zRotation);   

//if arcade drive = true, call the arcade drive method to enable arcade drive
//else enables tank drive
if(isArcadeDrive){
    _drive.arcadeDrive(Math.pow(xSpeed,3) , Math.pow(zRotation,3), false);
} else{
    _drive.tankDrive(Math.pow(tankLeft,3), Math.pow(tankRight,3), false);
}
  
  
//if button A pressed, switches to coast to brake mode and vice versa
    if(joy1.getRawButtonPressed(2)){
      if (currentModeBrake==NeutralMode.Coast){
        System.out.println("changing to brake");
        currentModeBrake = NeutralMode.Brake;
      }
      else {
        System.out.println("changing to coast");
        currentModeBrake = NeutralMode.Coast;
      }

      //setting motor modes to current value of current mode!!!!!!
      leftFrontMotor.setNeutralMode(currentModeBrake);
      rightFrontMotor.setNeutralMode(currentModeBrake);
      leftRearMotor.setNeutralMode(currentModeBrake);
      rightRearMotor.setNeutralMode(currentModeBrake);
    }

    //if button X pressed, changes between arcade and tank 
    if(joy1.getRawButtonPressed(1)){
      isArcadeDrive = !isArcadeDrive;
      System.out.println("currently arcade?: " + isArcadeDrive);
    }
    //prints out distance value
    System.out.println("current encoder value: " + encoder1.getDistance());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}