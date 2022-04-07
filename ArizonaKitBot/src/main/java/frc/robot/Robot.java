// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //drive motors, 4
  CANSparkMax l1 = new CANSparkMax(RobotMap.L1CANID, MotorType.kBrushless);
  CANSparkMax l2 = new CANSparkMax(RobotMap.L2CANID, MotorType.kBrushless);
  //CANSparkMax l3 = new CANSparkMax(RobotMap.L3CANID, MotorType.kBrushless);
  CANSparkMax r1 = new CANSparkMax(RobotMap.R1CANID, MotorType.kBrushless);
  CANSparkMax r2 = new CANSparkMax(RobotMap.R2CANID, MotorType.kBrushless);
  //CANSparkMax r3 = new CANSparkMax(RobotMap.R3CANID, MotorType.kBrushless);

  //leds
  Spark led = new Spark(RobotMap.BLINKINPORT);

  //shoot motors
  TalonSRX indexer = new TalonSRX(RobotMap.INDEXID);
  CANSparkMax shooter = new CANSparkMax(RobotMap.SHOOTID, MotorType.kBrushless);
  TalonSRX shootIntake = new TalonSRX(RobotMap.SHOOTINTAKEID);

  //drive
  MotorControllerGroup l = new MotorControllerGroup(l1, l2);
  MotorControllerGroup r = new MotorControllerGroup(r1, r2);
  DifferentialDrive drive = new DifferentialDrive(l,r);

  //controllers
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  //intake motors
  CANSparkMax intake = new CANSparkMax(RobotMap.INTAKEPORT, MotorType.kBrushless);
  CANSparkMax deployRetract = new CANSparkMax(RobotMap.DRINTAKEID, MotorType.kBrushless);

  //climb motors
  Spark climbB = new Spark(RobotMap.CLIMBAPORT);
  Spark climbA = new Spark(RobotMap.CLIMBBPORT);

  //gross toggling things
  boolean toggleOn = false;
  boolean toggleButtonPressed = false;

  //Auto
  Timer timer = new Timer();

  //limelight
  private boolean llVT;
  private double llDrive;
  private double llSteer;

  //gyro
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  double i = 0;
  double d = 0;
  double p = Math.pow(0.5, 9);
  PIDController PID = new PIDController(p, i, d);

  boolean comboButtonPressed = false;

  //pneumatics
  /*DoubleSolenoid sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.SOLCHANNEL1, RobotMap.SOLCHANNEL2);
  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);*/

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    gyro.calibrate();
    gyro.reset();
    
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    timer.start();
    timer.reset();
  }

  @Override
  public void autonomousPeriodic() {
    //good code
    double delay = 2.5;
    //increased from .5

    if(timer.get()<RobotMap.AUTOSPINUPSHOOTINIT){
      shooter.set(RobotMap.AUTOSHOOTSPEED);

      //deployRetract.set(0.2);
    }
    if(timer.get()<RobotMap.AUTOSHOOTFIRST&& timer.get()> RobotMap.AUTOSPINUPSHOOTINIT){
      //shooter.set(RobotMap.AUTOSHOOTSPEED);
      shootIntake.set(ControlMode.PercentOutput, RobotMap.AUTOSHOOTINTAKESPEED);
      indexer.set(ControlMode.PercentOutput, RobotMap.AUTOINDEXERSPEED);


      //deployRetract.set(0.0);
    }
    if(timer.get()>RobotMap.AUTOSHOOTFIRST &&timer.get()<RobotMap.AUTODRIVEBACK){
      shooter.set(0);
      shootIntake.set(ControlMode.PercentOutput, 0);
      indexer.set(ControlMode.PercentOutput, 0);


      
      drive.arcadeDrive(0, .36);
      //intake.set(-0.7);
    }
    
    if(timer.get()>RobotMap.AUTODRIVEBACK && timer.get()<RobotMap.AUTOSTOPDRIVE){
      drive.arcadeDrive(0,0);
    }
    /*if(timer.get()>RobotMap.AUTOSTOPDRIVE && timer.get()< 12.9 + delay){
      intake.set(0);
      drive.arcadeDrive(0, -.36);
    }
    if(timer.get()>12.9 + delay && timer.get()< 13.4 + delay){
      shooter.set(RobotMap.AUTOSHOOTSPEED);
    }
    if(timer.get()>13.4 + delay && timer.get()< 15.9 + delay){
      shootIntake.set(ControlMode.PercentOutput, RobotMap.AUTOSHOOTINTAKESPEED);
      indexer.set(ControlMode.PercentOutput, RobotMap.AUTOINDEXERSPEED);
    }
    if(timer.get()>15.9 + delay && timer.get()<15.9 + 4.9 + delay){
      shooter.set(0);
      shootIntake.set(ControlMode.PercentOutput, 0);
      indexer.set(ControlMode.PercentOutput, 0);

    }*/
    //end good code
  
   /*if(timer.get()<RobotMap.AUTODEPLOYINTAKE){
      //intake down
      //deployRetract.set(0.5);
      //spin up shooter
      shooter.set(RobotMap.AUTOSHOOTSPEED);
    }
    if(timer.get()<RobotMap.AUTOSPINUPSHOOT1 && timer.get()>RobotMap.AUTODEPLOYINTAKE){
      deployRetract.set(0);
    }
    if(timer.get()<RobotMap.AUTOSHOOT && timer.get()>RobotMap.AUTOSPINUPSHOOT1){
      shootIntake.set(ControlMode.PercentOutput, RobotMap.AUTODRINTAKESPEED);
      indexer.set(ControlMode.PercentOutput, RobotMap.AUTOINDEXERSPEED);
    }
    if(timer.get()<RobotMap.AUTOSHOOT && timer.get()>RobotMap.AUTODRIVEBACK){
      shooter.set(0);
      shootIntake.set(ControlMode.PercentOutput, 0);
      indexer.set(ControlMode.PercentOutput, 0);
    }
    if(timer.get()<RobotMap.AUTODRIVEBACK && timer.get()>RobotMap.AUTOINTAKE){
      drive.arcadeDrive(-RobotMap.AUTODRIVESPEED, RobotMap.AUTODRIVETURN);
      //intake.set(0.7);
    }
    if(timer.get()<RobotMap.AUTOINTAKE && timer.get()>RobotMap.AUTODRIVEFORWARD){
      drive.arcadeDrive(0, 0);
    }
    
    if(timer.get()<RobotMap.AUTODRIVEFORWARD && timer.get()>RobotMap.AUTOSPINUPSHOOT2){
      drive.arcadeDrive(RobotMap.AUTODRIVESPEED, RobotMap.AUTODRIVETURN);
      intake.set(0);
    }
    if(timer.get()<RobotMap.AUTOSPINUPSHOOT2 && timer.get()>RobotMap.AUTOSHOOT2){
      drive.arcadeDrive(0, 0);
      shootIntake.set(ControlMode.PercentOutput, RobotMap.AUTODRINTAKESPEED);
      indexer.set(ControlMode.PercentOutput, RobotMap.AUTOINDEXERSPEED);
    }
    if(timer.get()>RobotMap.AUTOSHOOT2){
      shooter.set(0);
      shootIntake.set(ControlMode.PercentOutput, 0);
      indexer.set(ControlMode.PercentOutput, 0);
    }*/
    
    
  }

  @Override
  public void teleopInit() {
    l1.setOpenLoopRampRate(RobotMap.RAMP_VAL);
    l2.setOpenLoopRampRate(RobotMap.RAMP_VAL);
    r1.setOpenLoopRampRate(RobotMap.RAMP_VAL);
    r2.setOpenLoopRampRate(RobotMap.RAMP_VAL);
  }

  @Override
  public void teleopPeriodic() {
    intake();
    //operatorIntake();
    speedButtons();
    deployRetractIntake();
    retractDeployClimber();
    shootBothControllers();
    driverTransition();
    //timeShoot();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    
  }

  public void operatorIntake(){
    if(operator.getRawAxis(RobotMap.OPINTAKEFORWARDAXIS)>0){
      intake.set(0.7);
    }
    if(operator.getRawAxis(RobotMap.OPINTAKEBACKAXIS)>0){
      intake.set(-0.7);
    }
    if(operator.getRawAxis(RobotMap.OPINTAKEFORWARDAXIS)==0 && operator.getRawAxis(RobotMap.OPINTAKEBACKAXIS)==0){
      intake.set(0);
    }
  }

  public void intake(){
    if(driver.getRawButton(RobotMap.INTAKEBUTTONFOR)){
      intake.set(0.7);
      shootIntake.set(ControlMode.PercentOutput, -0.4);
    }
    if(driver.getRawButton(RobotMap.INTAKEBUTTONBAC)){
      intake.set(-0.7);
      shootIntake.set(ControlMode.PercentOutput, .4);
    }
    if (!driver.getRawButton(RobotMap.INTAKEBUTTONFOR)&&!driver.getRawButton(RobotMap.INTAKEBUTTONBAC))
  {intake.set(0.0);
  shootIntake.set(ControlMode.PercentOutput, 0);}
  
    
  }

  public void reverseAll(){
    if(driver.getPOV() == 180){
      intake.set(-.7);
      deployRetract.set(-.4);
      indexer.set(ControlMode.PercentOutput, -.9);
      shootIntake.set(ControlMode.PercentOutput, -.4);
    }
    else{
      intake.set(0);
      deployRetract.set(0);
      indexer.set(ControlMode.PercentOutput, 0);
      shootIntake.set(ControlMode.PercentOutput, 0);
    }
  }
  public void deployRetractIntake(){
    deployRetract.set(driver.getRawAxis(5)*0.4);
    
  }
  public void shootIntake(){
    if(operator.getRawButton(RobotMap.SHOOTINTAKEBACKBUTTON)){
      shootIntake.set(ControlMode.PercentOutput, -.4);

    }
    if(operator.getRawButton(RobotMap.SHOOTINTAKEFORWARDBUTTON)){
      shootIntake.set(ControlMode.PercentOutput, .4);

    }
    if(!operator.getRawButton(RobotMap.SHOOTINTAKEBACKBUTTON) && !operator.getRawButton(RobotMap.SHOOTINTAKEFORWARDBUTTON)){
      shootIntake.set(ControlMode.PercentOutput, 0.0);
    }
  }
  public void indexer(){
    if(operator.getRawButton(RobotMap.INDEXERBUTTON)){
      indexer.set(ControlMode.PercentOutput, 0.9);

    }
    if(!operator.getRawButton(RobotMap.INDEXERBUTTON)){
      indexer.set(ControlMode.PercentOutput, 0.0);
    }
  }
  public void shoot(){
    if(operator.getRawButton(RobotMap.SHOOTBUTTON)){
      shooter.set(0.7);

    }
    if(!operator.getRawButton(RobotMap.SHOOTBUTTON)){
      shooter.set(0.0);
    }
  }
  public void toggleShoot(){
    updateToggle();

    if(toggleOn){
      shooter.set(0.7);
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

public void driverTransition(){
  if(driver.getRawButton(RobotMap.DRIVERSHOOTINTAKE)){
    shootIntake.set(ControlMode.PercentOutput, .4);

  }
  
  if(!driver.getRawButton(RobotMap.DRIVERSHOOTINTAKE)){
    shootIntake.set(ControlMode.PercentOutput, 0.0);
  }
}
  public void timeShoot(){
    if(driver.getRawButton(RobotMap.SHOOTCOMBINATIONBUTTON)){
      timer.start();
      timer.reset();
      
      if(timer.get()>.5){
        //spin up
          shooter.set(RobotMap.AUTOSHOOTSPEED);
      }
      if(timer.get()<.5 + 2.5 && timer.get()> .5){
        //shooter.set(RobotMap.AUTOSHOOTSPEED);
        shootIntake.set(ControlMode.PercentOutput, RobotMap.AUTOSHOOTINTAKESPEED);
        indexer.set(ControlMode.PercentOutput, RobotMap.AUTOINDEXERSPEED);
      }
      if(timer.get()>.5 + 2.5){
        shooter.set(0);
        shootIntake.set(ControlMode.PercentOutput, 0);
        indexer.set(ControlMode.PercentOutput, 0);
      }
    }
    if(!driver.getRawButton(RobotMap.SHOOTCOMBINATIONBUTTON)){
      shooter.set(0);
      shootIntake.set(ControlMode.PercentOutput, 0);
      indexer.set(ControlMode.PercentOutput, 0);
    }
  }

  /**
   * @author Lauren
   * handles shooting for both controllers
   * when button {@code RobotMap.SHOOTCOMBINATIONBUTTON} is pressed, 
   * overrides all operator commands and activates all shooting 
   * motors.
   */
  public void shootBothControllers(){
    if(driver.getRawButton(RobotMap.SHOOTCOMBINATIONBUTTON)){
      shootIntake.set(ControlMode.PercentOutput, -1);    
      indexer.set(ControlMode.PercentOutput, 1);
      shooter.set(0.7);
      comboButtonPressed = true;
    }//else
    if(!driver.getRawButton(RobotMap.SHOOTCOMBINATIONBUTTON)){
      

      comboButtonPressed = false;
      shoot();
      shootIntake();
      indexer();
    }
  }

  public void speedButtons(){
    //slow button for xbox controller
   if(driver.getRawButton(3)){
    drive.arcadeDrive(-driver.getRawAxis(0) * 0.2, -driver.getRawAxis(3) * 0.2);
    if(driver.getRawAxis(2) > 0){
    drive.arcadeDrive(-driver.getRawAxis(0) * 0.2, driver.getRawAxis(2) * 0.2);
    }
  }

 //fast button for xbox controller
  else if(driver.getRawButton(1)){
    drive.arcadeDrive(-driver.getRawAxis(0), -driver.getRawAxis(3));
    if(driver.getRawAxis(2)>0){
      drive.arcadeDrive(-driver.getRawAxis(0), driver.getRawAxis(2));
    }
  }

 //default condition for neither buttons active
  else if(!driver.getRawButton(3) || !driver.getRawButton(1)){
    drive.arcadeDrive(-driver.getRawAxis(0) * 0.8, -driver.getRawAxis(3) * 0.8);
    if(driver.getRawAxis(2) > 0){
      drive.arcadeDrive(-driver.getRawAxis(0) * 0.8, driver.getRawAxis(2) * 0.8);
    }
  }
} 
  public void retractDeployClimber(){
    //if(timer.get()>=120){
      if(operator.getPOV() == 0){
        climbA.set(-0.8);
        climbB.set(-0.8);     
      }
      
      if(operator.getPOV() == 180){
        climbA.set(0.8);
        climbB.set(0.8);     
      }
      if(operator.getPOV()!=0 && operator.getPOV()!= 180){
        climbA.set(0);
        climbB.set(0);
      }

    }
  //}
  
  
  public void turnTo(double targetAngle, double targetSpeed){
    //angle = 0-2^16
    PID.setSetpoint(targetAngle);
    PID.setTolerance(3, 0.1);
    double turn = PID.calculate(gyro.getAngle());
    if(PID.getPositionError()>3){

      //maybe supposed to be tank drive??
      /**drive.arcadeDrive(0,turn); for no forward movement, mess around with sign of turn
        *drive.tankDrive(turn,-turn); for tank drive, mess around with signs and pid values
      */
      drive.tankDrive(turn, -turn);
    }else{
      drive.arcadeDrive(targetSpeed, turn);
    }
  }
  public void leds(){
    if(driver.getPOV() == 0){
      //up red
      led.set(0.61);
    }
    /*if(driver.getPOV() == 180){
      //down blue
      led.set(0.87);}*/
    
    if(driver.getPOV() == 90){
      //right "twinkles ocean palette"
      led.set(-0.51);
    }
    if(driver.getPOV() == 270){
      //left "twinkles lava palette"
      led.set(-0.49);
    }
  }

  

  /*public void UpdateLimeLight(){
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
  }*/
}
