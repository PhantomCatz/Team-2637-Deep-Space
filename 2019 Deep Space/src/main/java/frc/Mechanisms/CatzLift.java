
/*
 *  Author : Jean Kwon
 * 
 *  Functionality : controlls the lift by the speed, gets the status of each limit switch
 *                  gets the counts of the encoder,  moves the lift in to the target
 *   
 *  Methods :  lift, get LiftCounts, isLiftAtTop, isLiftAtBottom, moveLiftThread
 
 *  Revision History : 
 *  02-09-19 fixed the thread JK
 * 
 */

package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class CatzLift
{
    private static WPI_TalonSRX liftMtrCtrlLT;
    private static WPI_VictorSPX liftMtrCtrlRT;
    
    private static SpeedControllerGroup liftMotors;

    private static final int LIFT_RT_MC_CAN_ID = 11; 
    private static final int LIFT_LT_MC_CAN_ID = 10;

    private static DigitalInput liftExtendedLimitSwitch;
    private static DigitalInput liftRetractedLimitSwitch;


 /* *******************************************************************************
    * Lift Encoder - pulses to inches 
    * Andy Mark Red Line Mag Encoder which provides 1024 CPR
    * The gear reduction is 6 to 1.
    * The diameter of wrench is 1 inch 
    * It attached to the motor
    *****************************************************************************/

    private static final double LIFT_ENCODER_PULSE_PER_REV = 1024.0;
    private static final double LIFT_WINCH_DIAMETER = 1.0;
    private static final double LIFT_GEAR_RATIO = 1.0/6.0;
    private static final double LIFT_COUNTS_PER_INCHES = LIFT_ENCODER_PULSE_PER_REV / 
                                                         (Math.PI*LIFT_WINCH_DIAMETER) * LIFT_GEAR_RATIO;


    private static final double LIFT_COUNT_TOLERANCE = 100 * LIFT_COUNTS_PER_INCHES; //TBD Type it in inches

    public static Encoder liftEnc;              
    private static final int LIFT_ENCODER_A_DIO_PORT = 0; //TBD    
    private static final int LIFT_ENCODER_B_DIO_PORT = 0;

    private static final int LIFT_EXTENDED_LIMIT_SWITCH_ANALOG_PORT = 2;
    private static final int LIFT_RETRACTED_LIMIT_SWITCH_ANALOG_PORT = 3;

    private static final double LIFT_STOP_BUFFER_DISTANCE = 0; //TODO
    private static final double LIFT_STOP_BUFFER_SPEED = 0; //TODO

    public CatzLift()
    {
        liftMtrCtrlLT = new WPI_TalonSRX (LIFT_LT_MC_CAN_ID);
        liftMtrCtrlRT = new WPI_VictorSPX (LIFT_RT_MC_CAN_ID);

        liftMtrCtrlRT.follow(liftMtrCtrlLT);
        
        liftMotors = new SpeedControllerGroup(liftMtrCtrlLT, liftMtrCtrlRT);
        liftEnc = new Encoder(LIFT_ENCODER_A_DIO_PORT, LIFT_ENCODER_B_DIO_PORT, false, EncodingType.k4X); 

        liftExtendedLimitSwitch = new DigitalInput(LIFT_EXTENDED_LIMIT_SWITCH_ANALOG_PORT);
        liftRetractedLimitSwitch = new DigitalInput(LIFT_RETRACTED_LIMIT_SWITCH_ANALOG_PORT);

        
    } 

    public void lift(double power) //to drop it put negative value
    { 

        if(isLiftAtBottom()&&power<0){
            liftMotors.stopMotor();
        }
        else if(isLiftAtTop()&&power>0){
            liftMotors.stopMotor();
        }
        else if(getLiftHeight() < 0 + LIFT_STOP_BUFFER_DISTANCE&&
            power<0)
        {
            liftMotors.set(-LIFT_STOP_BUFFER_SPEED);
        }
        else if(getLiftHeight() > 00000 - LIFT_STOP_BUFFER_DISTANCE&& //change 00000 to height of lift
            power>0)
        {
            liftMotors.set(LIFT_STOP_BUFFER_SPEED);
        }
        else{
            liftMotors.set(power);
        }
    }

    public static int getLiftCounts()
    {
        return liftEnc.get();
    }

    public static double getLiftHeight() 
    {
        return ((double) getLiftCounts()) / LIFT_COUNTS_PER_INCHES;
    }
  
    public static boolean isLiftAtTop()  
    {
       return liftExtendedLimitSwitch.get();
    }
  
    public static boolean isLiftAtBottom()
    {
        return liftRetractedLimitSwitch.get();
    }

    public static void moveLiftThread(double targetHeight, double power, double timeOut) //Absolute 
    {      
        Timer threadTimer = new Timer();
        threadTimer.start();

        Thread liftThread = new Thread(() ->
        {
           int currentCount = getLiftCounts();
           double targetCount  = (targetHeight * LIFT_COUNTS_PER_INCHES) - (double) currentCount;
           
           double upperLimit = targetCount + LIFT_COUNT_TOLERANCE;
           double lowerLimit = targetCount - LIFT_COUNT_TOLERANCE;

           if(currentCount < lowerLimit) 
            {
                liftMotors.set(power);

            } else if(currentCount > upperLimit) 
            {
                liftMotors.set(-power);
            }

            while(!Thread.interrupted())
            {
                currentCount = getLiftCounts(); //update the current count of the lift

                if((lowerLimit < currentCount && upperLimit > currentCount) || threadTimer.get() > timeOut) 
                { 
                    liftMotors.stopMotor();
                    Thread.currentThread().interrupt();
                }
                Timer.delay(0.005);
            }
        }); 
        
        liftThread.start();
    }
}

