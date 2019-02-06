
/*
 *  Author : Jean Kwon

 *  Methods : lift, get LiftCounts, isLiftLimitSwitchTop, is LiftLimitSwitchBot, setLiftHeight
 *  Functionality : controlls the lift by the speed, gets the status of each limit switch
 *                  gets the counts of the encoder,  sets the lift in to the target
 *   
 *  Revision History : 
 *  02-01-19 Added the method setLiftHeight JK
 * 
 */

package frc.Mechanisms;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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

    private static final int LIFT_COUNT_THRESHOLD = 100; //TBD
    private static       double targetCounts;

    private static final double LIFT_COUNTS_PER_INCH  = 0; //TBD

    /*public static Encoder liftEnc;              
    private static final int LIFT_ENCODER_A_DIO_PORT = ;     if encoder in neos isn't good
    private static final int LIFT_ENCODER_B_DIO_PORT = ;*/


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

    public static double getLiftCounts()
    {
        return liftMtrCtrlLT.getEncoder().getPosition();
    }
  
    public static boolean isLiftLimitSwitchTopActivated()
    {
        return liftMtrCtrlLT.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen).get();
    }
  
    public static boolean isLiftLimitSwitchBotActivated()
    {
        return liftMtrCtrlRT.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen).get();
    }

    public static void setLiftHeight(double targetHeight, double speed) 
    { 

        while(getLiftCounts() < targetHeight - LIFT_COUNT_THRESHOLD) 
        {
            liftMotors.set(speed);
        }
    

        while(getLiftCounts() > targetHeight + LIFT_COUNT_THRESHOLD) 
        {
            liftMotors.set(-speed);
        }
    
        liftMotors.stopMotor();
      
    }
}