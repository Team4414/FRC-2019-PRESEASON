package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI{
    
    private static OI instance;
    public static OI getInstance(){
        if (instance == null)
            instance = new OI();
        return instance;
    }

    private static final int kThrottleNubID = 0;
    private static final int kTurnNubID = 1;

    private double kThrottleStickOffset = 0;
    private double kTurnStickOffset = 0;

    private static final double kThrottleScaler = 1;
    private static final double kTurnScalar = 1;

    private static final int kXbox = 2;

    private static final int kTurnAxis = 0;
    private static final int kThrottleAxis = 1;

    private static final int kQuickTurnButtonID = 6;


    private Joystick throttleNub;
    private Joystick turnNub;
    private Joystick xbox;
     
    private OI(){
        throttleNub = new Joystick(kThrottleNubID);
        turnNub = new Joystick(kTurnNubID);
        xbox = new Joystick(kXbox);

        kThrottleStickOffset = 0;
        kTurnStickOffset = 0;

        kThrottleStickOffset = getForward();
        kTurnStickOffset = getLeft();
    }

    public double getLeft(){
        return -kTurnScalar * turnNub.getRawAxis(kTurnAxis) - kTurnStickOffset;
    }

    public double getForward(){
        return kThrottleScaler * throttleNub.getRawAxis(kThrottleAxis) - kThrottleStickOffset;
    }

    public boolean getQuickTurn(){
        return ((throttleNub.getRawButton(11))
            ||  (throttleNub.getRawButton(10)));
    }
}