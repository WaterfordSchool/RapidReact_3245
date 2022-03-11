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
    public static final double AUTODEPLOYINTAKE = 2.0;
    public static final double AUTOSPINUPSHOOT1 = RobotMap.AUTODEPLOYINTAKE + 1.0;
    public static final double AUTOSHOOT = RobotMap.AUTOSPINUPSHOOT1 + 2.0;
    public static final double AUTODRIVEBACK = RobotMap.AUTOSHOOT + 2.0;
    public static final double AUTOINTAKE = RobotMap.AUTODRIVEBACK + 1.0;
    public static final double AUTODRIVEFORWARD = RobotMap.AUTOINTAKE + 1.0;
    public static final double AUTOSPINUPSHOOT2 = RobotMap.AUTODRIVEFORWARD + 1.0;
    public static final double AUTOSHOOT2 = RobotMap.AUTOSPINUPSHOOT2 + 1.0;
    

    public static final double AUTODRIVESPEED = .5;
    public static final double AUTOINDEXERSPEED = .5;
    public static final double AUTODRINTAKESPEED = .5;
    public static final double AUTODRIVETURN = 0.0;
    public static final double AUTOSHOOTSPEED = 0.65;

    //operator controls
    public static final int OPERATORINDEXERBUTTON = 1;
    public static final int SHOOTERBUTTON = 3;
    public static final int DEPLOYRETRACTINTAKEAXIS = 3;
    public static final int RETRACTINTAKEAXIS = 4;
    public static final int INTAKEBUTTONFOR = 5;
    public static final int INTAKEBUTTONBAC = 6;
    public static final int CLIMBERBUTTON = 4;

    //driver controls...
    

    
    /*
    pneumatics
    public static final int SOLCHANNEL1 = 1;
    public static final int SOLCHANNEL2 = 2;

    pneumatics controls
    public static final int SOLOFFBUTTON = 5;
    public static final int SOLONBUTTON = 2;
    */
    
    //shooter motor controller IDs
    public static final int INDEXID = 0;
    public static final int SHOOTID = 5;
    public static final int SHOOTINTAKEID = 6;

    //intake motor controller ports (pwm)
    public static final int INTAKEPORT = 4;
    public static final int DRINTAKEPORT = 5;

    
<<<<<<< HEAD
=======

>>>>>>> aa15ec98e4264fbab895c90cd6720fbc97ac134e
    //climb motor ids (NOT finalized, not tested yet)
    public static final int CLIMBAID = 99;
    public static final int CLIMBBID = 100;

}
