package frc.Mechanisms;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.CounterBase.EncodingType;


public class CatzLift
{
    private static CANSparkMax liftMtrCtrlLT;
    private static CANSparkMax liftMtrCtrlRT;
    
    private static SpeedControllerGroup liftMotors;

    private static final int LIFT_RT_MC_CAN_ID = 11; 
    private static final int LIFT_LT_MC_CAN_ID = 10;

    /*public static Encoder liftEnc;              
    private static final int LIFT_ENCODER_A_DIO_PORT = ;     if encoder in neos isn't good
    private static final int LIFT_ENCODER_B_DIO_PORT = ;*/

    public static DigitalInput liftLimitTop;
    public static DigitalInput liftLimitBot;

    public CatzLift()
    {
        liftMtrCtrlLT = new CANSparkMax(LIFT_LT_MC_CAN_ID, MotorType.kBrushless);
        liftMtrCtrlRT = new CANSparkMax(LIFT_RT_MC_CAN_ID, MotorType.kBrushless);
        
        liftMotors = new SpeedControllerGroup(liftMtrCtrlLT, liftMtrCtrlRT);
       /* liftEnc = new Encoder(LIFT_ENCODER_A_DIO_PORT, 
                              LIFT_ENCODER_B_DIO_PORT, false, EncodingType.k4X);*/
    } 
    public static void lift(double speed)
    {
        liftMotors.set(speed);
    }
    public static double liftCounts()
    {
        return liftMtrCtrlLT.getEncoder().getPosition();
    }
    public static boolean getLiftLimitTop()
    {
        return liftMtrCtrlLT.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen).get(); //
    }
    public static boolean getLiftLimitBot()
    {
        return liftMtrCtrlRT.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen).get(); //
    }
}