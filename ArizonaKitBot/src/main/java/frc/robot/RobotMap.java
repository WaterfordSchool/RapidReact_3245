package frc.robot;

public class RobotMap {
    //can drive motors; CAN IDS CORRECT
    public static final int R1CANID = 1;
    public static final int R2CANID = 2;
    public static final int L1CANID = 3;
    public static final int L2CANID = 4;

   
    //pwm ports
    public static final int BLINKINPORT = 1;

    //auto; change names once designate function
    public static final double AUTOTIME1 = 1.0;
    public static final double AUTOTIME2 = 1.0;
    public static final double AUTOTIME3 = 1.0;
    public static final double AUTOTIME3DURATION = 1.0;
    public static final double AUTODRIVESPEED1 = .5;
    public static final double AUTODRIVETURN1 = 0.0;

    //operator controls
    public static final int OPERATORINDEXERBUTTON = 1;
    public static final int SHOOTERBUTTON = 3;
    public static final int DEPLOYRETRACTINTAKEAXIS = 3;
    public static final int RETRACTINTAKEAXIS = 4;
    public static final int INTAKEBUTTONFOR = 5;
    public static final int INTAKEBUTTONBAC = 6;
    public static final int CLIMBERBUTTON = 4;
    //driver controls 
    public static final int DRIVERINDEXERSORTINGBUTTON = 2;
    
    //pneumatics
    public static final int SOLCHANNEL1 = 1;
    public static final int SOLCHANNEL2 = 2;

    //pneumatics controls
    public static final int SOLOFFBUTTON = 5;
    public static final int SOLONBUTTON = 2;
    
    //shooter motor controller IDs
    public static final int INDEXID = 0;
    public static final int SHOOTID = 5;
    public static final int SHOOTINTAKEID = 6;

    //intake motor controller ports (pwm)
    public static final int INTAKEPORT = 4;
    public static final int DRINTAKEPORT = 5;

    //leds (NOT finalized)
    public static final int BLINKINPORT = 10;

    //climb motor ids (NOT finalized, not tested yet)
    public static final int CLIMBAID = 99;
    public static final int CLIMBBID = 100;

}
