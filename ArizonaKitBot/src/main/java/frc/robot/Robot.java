// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  CANSparkMax l1 = new CANSparkMax(RobotMap.L1CANID, MotorType.kBrushless);
  CANSparkMax l2 = new CANSparkMax(RobotMap.L2CANID, MotorType.kBrushless);
  //CANSparkMax l3 = new CANSparkMax(RobotMap.L3CANID, MotorType.kBrushless);
  CANSparkMax r1 = new CANSparkMax(RobotMap.R1CANID, MotorType.kBrushless);
  CANSparkMax r2 = new CANSparkMax(RobotMap.R2CANID, MotorType.kBrushless);
  //CANSparkMax r3 = new CANSparkMax(RobotMap.R3CANID, MotorType.kBrushless);
//comment

  TalonSRX indexer = new TalonSRX(0);
  CANSparkMax shooter = new CANSparkMax(5, MotorType.kBrushless);
  TalonSRX shootIntake = new TalonSRX(6);

  MotorControllerGroup l = new MotorControllerGroup(l1, l2);
  MotorControllerGroup r = new MotorControllerGroup(r1, r2);

  DifferentialDrive drive = new DifferentialDrive(l,r);

  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  boolean toggleOn = false;
  boolean toggleButtonPressed = false;

  private boolean llVT;
  private double llDrive;
  private double llSteer;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    shootIntake();
    //updateToggle();
    //toggleShoot();
    shoot();
    //laurenToggle();
    //laurenUpdateToggle();
    indexer();
    speedButtons();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  
  public void shootIntake(){
    if(operator.getRawButton(1)){
      shootIntake.set(ControlMode.PercentOutput, -0.5);

    }
    if(operator.getRawButton(4)){
      shootIntake.set(ControlMode.PercentOutput, 0.5);

    }
    if(!operator.getRawButton(1) && !operator.getRawButton(4)){
      shootIntake.set(ControlMode.PercentOutput, 0.0);
    }
  }
  public void indexer(){
    if(operator.getRawButton(2)){
      indexer.set(ControlMode.PercentOutput, 0.5);

    }
    if(!operator.getRawButton(2)){
      indexer.set(ControlMode.PercentOutput, 0.0);
    }
  }
  public void shoot(){
    if(operator.getRawButton(3)){
      shooter.set(0.65);
    }
    if(!operator.getRawButton(3)){
      shooter.set(0.0);
    }
  }
  public void toggleShoot(){
    updateToggle();

    if(toggleOn){
      shooter.set(0.65);
    }
    else{shooter.set(0);}
  }
  public void updateToggle(){
    if(operator.getRawButton(3)){
      if(!toggleButtonPressed){
        toggleOn = !toggleOn;
        toggleButtonPressed = true;
      }
      else{toggleButtonPressed = false;}
    }
  }

  public void laurenToggle(){
    toggleButtonPressed = operator.getRawButton(3);
    toggleOn = toggleButtonPressed;
  }
  public void laurenUpdateToggle(){
    if(toggleOn){
      shooter.set(0.65);
    }
    if(toggleOn == false){
      shooter.set(0);
    }
  }
  public void allThree(){
    if(operator.getRawButton(4)){
      shootIntake.set(ControlMode.PercentOutput, -0.5);    
      indexer.set(ControlMode.PercentOutput, 0.5);
      shooter.set(0.65);

    }
    if(!operator.getRawButton(4)){
      shootIntake.set(ControlMode.PercentOutput, 0);    
      indexer.set(ControlMode.PercentOutput, 0);
      shooter.set(0);
    }
  }

  public void speedButtons(){
    //slow button for xbox controller
   if(driver.getRawButton(3)){
    drive.arcadeDrive(driver.getRawAxis(0) * 0.2, driver.getRawAxis(3) * 0.2);
    if(driver.getRawAxis(2) > 0){
    drive.arcadeDrive(driver.getRawAxis(0) * 0.2, -driver.getRawAxis(2) * 0.2);
  }
 }

 //fast bu
 
 

 //default condition for neither buttons active
 else if(!driver.getRawButton(3) || !driver.getRawButton(1)){
  drive.arcadeDrive(driver.getRawAxis(0) * 0.8, driver.getRawAxis(3) * 0.8);
  if(driver.getRawAxis(2) > 0){
    drive.arcadeDrive(driver.getRawAxis(0) * 0.8, -driver.getRawAxis(2) * 0.8);
  }
 }
  } 

  public void updateLimeLight(){
    final double STEER= 0.1;      //how hard to turn
    final double DRIVE= 0.25;     //how hard to drive forward
    final double TARGET_AREA = 10;//percentage of screen that the target covers
    final double MAX_DRIVE = 0.7; //fastest the robot can go

    double dv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double dx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double dy = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double da = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    //if there isnt a target, gives up
    if(dv<1.0){
      llVT = false;
      llDrive = 0;
      llSteer = 0;
      return;
    }
    //there is a target, find amout to turn and move
    llVT = true;
    llSteer = dx*STEER;
    llDrive = (TARGET_AREA- da)*DRIVE;
    llDrive = llDrive>MAX_DRIVE?MAX_DRIVE:llDrive;
  }
}