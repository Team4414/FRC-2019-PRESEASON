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

    private static final int kTurnAxis = 0;
    private static final int kThrottleAxis = 1;

    private static final int kQuickTurnButtonID = 11;


    private Joystick throttleNub;
    private Joystick turnNub;
     
    private OI(){
        throttleNub = new Joystick(kThrottleNubID);
        turnNub = new Joystick(kTurnNubID);
    }

    public double getLeft(){
        return turnNub.getRawAxis(kTurnAxis);
    }

    public double getForward(){
        return throttleNub.getRawAxis(kThrottleAxis);
    }

    public boolean getQuickTurn(){
        return turnNub.getRawButton(kQuickTurnButtonID);
    }
}