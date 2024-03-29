package frc.robot;
/*dear god this is hell
though hell cowers in the face of my agony*/

public class RobotMap {
    //can drive motors; CAN IDS CORRECT
    public static final int R1CANID = 3;
    public static final int R2CANID = 2;
    public static final int R3CANID = 54;
    public static final int L1CANID = 1;
    public static final int L2CANID = 4;
    public static final int L3CANID = 55;

    //pwm ports
    public static final int BLINKINPORT = 9;

    //auto durations
    public static final double AUTOSPINUPSHOOTINIT = 1.5;
    public static final double AUTOTUNEDRIVEBACK = AUTOSPINUPSHOOTINIT + 2;
    public static final double AUTOSHOOTFIRST = AUTOSPINUPSHOOTINIT + 2.5;
    public static final double AUTODRIVEBACK = AUTOSHOOTFIRST + 4.9;

    //AUTOSPINUPSHOOTINIT + 7.4;

    public static final double AUTOSTOPDRIVE = AUTODRIVEBACK + .6;

    //AUTOSPINUPSHOOTINIT + 8;

    public static final double AUTODRIVEFOR = AUTOSTOPDRIVE + 3.9;

    //AUTOSPINUPSHOOTINIT + 12.9;

    public static final double AUTOSPINUPSHOOTSEC = AUTODRIVEFOR + AUTOSPINUPSHOOTINIT;

    public static final double AUTOSHOOTSEC = AUTOSPINUPSHOOTSEC + 2.5;

    public static final double AUTOSTOPSHOOTSEC = AUTOSHOOTSEC + 4.9;




    /*public static final double AUTODEPLOYINTAKE = 1.5;
    public static final double AUTOSPINUPSHOOT1 = RobotMap.AUTODEPLOYINTAKE + 0.15;
    public static final double AUTOSHOOT = RobotMap.AUTOSPINUPSHOOT1 + 2.0;
    public static final double AUTODRIVEBACK = RobotMap.AUTOSHOOT + 2.0;
    public static final double AUTOINTAKE = RobotMap.AUTODRIVEBACK + 7.4;
    public static final double AUTODRIVEFORWARD = RobotMap.AUTOINTAKE + 1.0;
    public static final double AUTOSPINUPSHOOT2 = RobotMap.AUTODRIVEFORWARD + 2.5;
    public static final double AUTOSHOOT2 = RobotMap.AUTOSPINUPSHOOT2 + 0.3;*/
    
    //auto speeds/magnitudes
    public static final double AUTODRIVESPEED = .15;
    public static final double AUTOINDEXERSPEED = .9;
    public static final double AUTODRINTAKESPEED = .5;
    public static final double AUTOSHOOTINTAKESPEED = 0.4;





    //^ changed from -0.5, change back if too strong
    //not change in teleop

    public static final double AUTODRIVETURN = 0.0;
    public static final double AUTOSHOOTSPEED = 0.625;

    //operator controls
    
    public static final int INDEXERBUTTON = 3;
    public static final int DEPLOYRETRACTINTAKEAXIS = 1;
    public static final int SHOOTINTAKEFORWARDBUTTON = 5;
    public static final int SHOOTINTAKEBACKBUTTON = 6;
    public static final int SHOOTBUTTON = 4;
    //y

    public static final int OPINTAKEFORWARDAXIS = 2;
    public static final int OPINTAKEBACKAXIS = 3;
    

    //intake forward:3 axis
    //bac 2

    //shoot intake: bumpers, for6 bac5

    //driver controls...
    public static final int SHOOTCOMBINATIONBUTTON = 2;
    public static final int INTAKEBUTTONBAC = 6;
    public static final int INTAKEBUTTONFOR = 5;
    public static final int DRIVERSHOOTINTAKE = 4;


    
    /*
    pneumatics (henry project)
    public static final int SOLCHANNEL1 = 1;
    public static final int SOLCHANNEL2 = 2;

    pneumatics controls
    public static final int SOLOFFBUTTON = 5;
    public static final int SOLONBUTTON = 2;
    */
    
    //shooter motor controller IDs
    public static final int INDEXID = 7; //good
    public static final int SHOOTID = 5; //good
    public static final int SHOOTINTAKEID = 6; //good

    //intake motor controller id
    public static final int INTAKEPORT = 9;
    public static final int DRINTAKEID = 8;
    //problem motor controller^.

    //climb motor ids (not tested yet)
    public static final int CLIMBAPORT = 0;
    public static final int CLIMBBPORT = 1;

    //ramping
    public static final double RAMP_VAL = 0.1;

}
