
/*
 *  Author : Jean Kwon

 *  Methods : lift, get LiftCounts, isLiftLimitSwitchTop, is LiftLimitSwitchBot, moveLift
 *  Functionality : controlls the lift by the speed, gets the status of each limit switch
 *                  gets the counts of the encoder,  sets the lift in to the target
 *   
 *  Revision History : 
 *  02-09-19 fixed the thread JK
 * 
 */

package frc.Mechanisms;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class CatzLift
{
    private static WPI_TalonSRX liftMtrCtrlLT;
    private static WPI_VictorSPX liftMtrCtrlRT;
    
    private static SpeedControllerGroup liftMotors;

    private static final int LIFT_RT_MC_CAN_ID = 11; 
    private static final int LIFT_LT_MC_CAN_ID = 10;

    private static final int LIFT_COUNT_THRESHOLD = 100; //TBD

    private static final int LIFT_COUNTS_PER_INCHES = 0; //TBD

    

    /*public static Encoder liftEnc;              
    private static final int LIFT_ENCODER_A_DIO_PORT = ;     if encoder in neos isn't good
    private static final int LIFT_ENCODER_B_DIO_PORT = ;*/


    public CatzLift()
    {
        liftMtrCtrlLT = new WPI_TalonSRX (LIFT_LT_MC_CAN_ID);
        liftMtrCtrlRT = new WPI_VictorSPX (LIFT_RT_MC_CAN_ID);

        liftMtrCtrlRT.follow(liftMtrCtrlLT);
        
        liftMotors = new SpeedControllerGroup(liftMtrCtrlLT, liftMtrCtrlRT);
        /*liftEnc = new Encoder(LIFT_ENCODER_A_DIO_PORT, 
                              LIFT_ENCODER_B_DIO_PORT, false, EncodingType.k4X); */
    } 

    public static void lift(double speed)
    {
        liftMotors.set(speed);
    }

    public static double getLiftCounts()
    {
        return liftMtrCtrlLT.getSensorCollection().getQuadraturePosition();
    }
  
    public static boolean isLiftLimitSwitchTopActivated()
    {
        return liftMtrCtrlLT.getSensorCollection().isFwdLimitSwitchClosed();
    }
  
    public static boolean isLiftLimitSwitchBotActivated()
    {
        return liftMtrCtrlLT.getSensorCollection().isRevLimitSwitchClosed();
    }

    public static void moveLift(double targetHeight, double speed) 
    {        
        Thread liftThread = new Thread(() ->
        {
           double currentCount = getLiftCounts();
           double targetCount  = targetHeight * LIFT_COUNTS_PER_INCHES;
           
           double upperLimit = targetCount + LIFT_COUNT_THRESHOLD;
           double lowerLimit = targetCount - LIFT_COUNT_THRESHOLD;

           if(currentCount < lowerLimit) 
            {
                lift(speed);
            }
            
            if(currentCount > upperLimit) 
            {
                lift(-speed);
            }

            while(!Thread.interrupted()) 
            {
                if(lowerLimit < currentCount && upperLimit > currentCount) 
                { 
                    lift(0);
                    Thread.currentThread().interrupt();
                }
            }
        }); 
        
        liftThread.start();

          
    }
}

