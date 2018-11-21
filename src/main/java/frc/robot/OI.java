package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI{
    
    private static OI instance;
    public static OI getInstance(){
        if (instance == null)
            instance = new OI();
        return instance;
    }

    private static final int kThrottleNubID = 1;
    private static final int kTurnNubID = 0;

    private static final int kXbox = 2;

    private static final int kTurnAxis = 4;
    private static final int kThrottleAxis = 1;

    private static final int kQuickTurnButtonID = 6;


    private Joystick throttleNub;
    private Joystick turnNub;
    private Joystick xbox;
     
    private OI(){
        //throttleNub = new Joystick(kThrottleNubID);
        //turnNub = new Joystick(kTurnNubID);
        xbox = new Joystick(kXbox);
    }

    public double getLeft(){
        return xbox.getRawAxis(kTurnAxis);
    }

    public double getForward(){
        return -xbox.getRawAxis(kThrottleAxis);
    }

    public boolean getQuickTurn(){
        return xbox.getRawButton(kQuickTurnButtonID);
    }
}