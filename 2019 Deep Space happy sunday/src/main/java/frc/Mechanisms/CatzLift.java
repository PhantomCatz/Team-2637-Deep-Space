
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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import frc.robot.CatzConstants;

public class CatzLift
{
    private static WPI_TalonSRX liftMtrCtrlLT;
    private static WPI_VictorSPX liftMtrCtrlRT;
    
    private static SpeedControllerGroup liftMotors;

    private static final int LIFT_RT_MC_CAN_ID = 11; 
    private static final int LIFT_LT_MC_CAN_ID = 10;

    private static DigitalInput liftExtendedLimitSwitch;
    public static DigitalInput liftRetractedLimitSwitch;


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
    //private static final double LIFT_COUNTS_PER_INCHES = LIFT_ENCODER_PULSE_PER_REV / (Math.PI*LIFT_WINCH_DIAMETER) * LIFT_GEAR_RATIO;determined experamentally KH
    
    private static final double LIFT_COUNTS_PER_INCHES = 2325.888888888889;

    private static final double LIFT_COUNT_TOLERANCE = 100 * LIFT_COUNTS_PER_INCHES; //TBD Type it in inches

    public static Encoder liftEnc;              
    private static final int LIFT_ENCODER_A_DIO_PORT = 2; //TBD    
    private static final int LIFT_ENCODER_B_DIO_PORT = 3;

    private static final int LIFT_EXTENDED_LIMIT_SWITCH_DIO_PORT = 4;
    private static final int LIFT_RETRACTED_LIMIT_SWITCH_DIO_PORT = 5;

    private static final int LIFT_ENC_MAX_COUNTS = 43000;

    public static final double LIFT_UP_MAX_POWER =  1.0;
    public static final double LIFT_DN_MAX_POWER = -0.5;

    private static final boolean LIFT_LIMIT_SWITCH_ACTIVATED = true;

    private static double targetHeight;

    public CatzLift()
    {
        liftMtrCtrlLT = new WPI_TalonSRX (LIFT_LT_MC_CAN_ID);
        liftMtrCtrlRT = new WPI_VictorSPX (LIFT_RT_MC_CAN_ID);

        liftMtrCtrlLT.setNeutralMode(NeutralMode.Brake);
        liftMtrCtrlRT.setNeutralMode(NeutralMode.Brake);

        liftMtrCtrlRT.follow(liftMtrCtrlLT);
        liftMtrCtrlLT.setInverted(true);

        liftMotors = new SpeedControllerGroup(liftMtrCtrlLT, liftMtrCtrlRT);

        liftEnc = new Encoder(LIFT_ENCODER_A_DIO_PORT, LIFT_ENCODER_B_DIO_PORT, false, EncodingType.k4X); 

        liftExtendedLimitSwitch = new DigitalInput(LIFT_EXTENDED_LIMIT_SWITCH_DIO_PORT);
        liftRetractedLimitSwitch = new DigitalInput(LIFT_RETRACTED_LIMIT_SWITCH_DIO_PORT);

        
    }
    public void setLiftTargetHeight(double height)
    {
        targetHeight = height;
    } 
    public double getLiftPower()
    {
        return liftMotors.get();
    }
    public void lift(double power) //to drop it put negative value. are you sure?? KH
    {
        liftMotors.set(power);
    }

    public int getLiftCounts()
    {
        return liftEnc.get();
    }

    public double getLiftHeight() 
    {
        return ((double) getLiftCounts()) / LIFT_COUNTS_PER_INCHES;
    }
  
    public boolean isLiftAtTop()  
    {
       return liftExtendedLimitSwitch.get();
    }
  
    public boolean isLiftAtBottom()
    {
        return liftRetractedLimitSwitch.get();
    }

    public void resetLiftEnc()
    {
        liftEnc.reset();
    }
    
    public void liftPID()
    {
        final double LIFT_THREAD_WAITING_TIME = 0.005;

        Thread t = new Thread(() ->
        {
            final double kP = 0.0; //TODO
            final double kI = 0.0;
            final double kD = 0.0;
            final double kF = 0.0;
            
            double power;            

            Timer liftTimer = new Timer();
            liftTimer.start();

            double integral = 0;

            double previousError = 0;
            double currentError;
            double deltaError = 0;
            double errorRate;

            double previousTime = 0;
            
            double currentDerivative;
            double previousDerivative = 0; // in case you want to filter derivative
            
            double deltaTime;
            
            double currentTime  = liftTimer.get();
            double currentHeight = getLiftHeight();

            while(true)
            {
                if(targetHeight == -1)
                {
                    Timer.delay(CatzConstants.CONTROLLER_INPUT_WAIT_TIME);
                }
                else
                {
                    currentTime  = liftTimer.get();
                    currentHeight = getLiftHeight();

                    currentError = targetHeight - currentHeight;
                
                    deltaError = currentError - previousError;
                    deltaTime  = currentTime  - previousTime;

                    //Riemann Sum
                    integral += deltaTime * currentError;

                    currentDerivative = (deltaError/deltaTime);
                    
                    power =  (kP * currentError) + (kI * integral) + (kD * currentDerivative) + kF;
                    //negative power is up
                    lift(-power);

                    previousError = currentError;
                    previousTime = currentTime;
                    previousDerivative = currentDerivative;
                
                    Timer.delay(LIFT_THREAD_WAITING_TIME);
                }
                
            }
             
        });
        t.start();
    }
    
}

