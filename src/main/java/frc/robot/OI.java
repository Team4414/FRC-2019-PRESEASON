package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI{
    
    private static OI instance;
    public static OI getInstance(){
        if (instance == null)
            instance = new OI();
        return instance;
    }

    private Joystick throttleNub;
    private Joystick turnNub;
     
    private OI(){
        throttleNub = new Joystick(0);
        turnNub = new Joystick(1);
    }

    public double getLeft(){
        return turnNub.getRawAxis(0);
    }

    public double getForward(){
        return throttleNub.getRawAxis(1);
    }
}