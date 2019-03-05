/*
 *  Author : Jean Kwon
 * 
 * Functionality : controls the arm extension by the power, controls the arm pivot by the power,
 *                 gets the status of each limit switch, gets the angle of the arm pivot,  
 *                 moves the arm extension to the target distance, moves the arm pivot to the targetAngle
 * 
 *  Methods : moveArm, movePivot, getExtensionEncoderCounts, isArmLimitExtendedActivated, isArmLimitRetractedActivated,
 *           getPivotAngle, moveArmThread, moveArmThread
 * 
 *  Revision History : 
 *  02-04-19 Added the thread and the encoder JK
 * 
 */
package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzConstants;


public class CatzArm 
{
    private static WPI_TalonSRX  armExtensionMtrCtrlA;  //A and B are designators
    private static WPI_VictorSPX armExtensionMtrCtrlB;

    private static CANSparkMax armPivotMtrCtrlLT;
    private static CANSparkMax armPivotMtrCtrlRT;

    private final int ARM_EXTENSION_A_MC_CAN_ID = 20;
    private final int ARM_EXTENSION_B_MC_CAN_ID = 21;

    private final int ARM_PIVOT_LT_MC_CAN_ID = 40;
    private final int ARM_PIVOT_RT_MC_CAN_ID = 41;

    private static DigitalInput armExtendedLimitSwitch;
    private static DigitalInput armRetractedLimitSwitch;

    private final int ARM_EXTENSION_LIMIT_EXTENDED_DIO_PORT  = 0; //TODO, TBD, same placeholding values woul conflict
    private final int ARM_EXTENSION_LIMIT_RETRACTED_DIO_PORT = 1;

    private static AnalogInput armPivotEnc;
  
    private static final int ARM_PIVOT_ENCODER_ANALOG_PORT = 1; //TBD
    private static final double ARM_PIVOT_ENC_MAX_VOLTAGE = 5.0;
  
    private static final int ARM_PIVOT_ANGLE_TOLERANCE = 0; //TBD
    
    private static final double ARM_PIVOT_ANGLE_MAX = 270.0;

    private static final double MAX_EXTENSION_LIMIT_INCHES = 30 / Math.cos(Math.abs(getPivotAngle()));



     /* **************************************************************************
    * Arm Extension Encoder - pulses to inches 
    * SRX Magnetic Encoder which provides 4096 pulses per revolution. 
    * The gear reduction is 2 to 1.
    * The diameter of winch is 0.984 inch 
    * It attached to the same shaft
    *****************************************************************************/

    private static final double ARM_EXTENSION_ENCODER_PULSE_PER_REV = 4096;
    private static final double ARM_EXTENSION_WINCH_DIAMETER = 0.984;
    private static final double ARM_EXTENSION_GEAR_RATIO = 1/2; //TBD
    private static final double ARM_COUNTS_PER_INCHES = ARM_EXTENSION_ENCODER_PULSE_PER_REV / 
                                                       (ARM_EXTENSION_WINCH_DIAMETER * Math.PI) * ARM_EXTENSION_GEAR_RATIO ;

    private static final double ARM_EXTENSION_COUNT_TOLERANCE = 100 * ARM_COUNTS_PER_INCHES; //TBD Type it in inches

    private static final double ARM_PIVOT_MAX_ANGLE = 0;
    private static final double ARM_PIVOT_MIN_ANGLE = 0;

    private static double PIVOT_VOLTAGE_OFFSET;

    public CatzArm()
    {
        armExtensionMtrCtrlA = new WPI_TalonSRX (ARM_EXTENSION_A_MC_CAN_ID);
        armExtensionMtrCtrlB = new WPI_VictorSPX(ARM_EXTENSION_B_MC_CAN_ID);

        armExtensionMtrCtrlB.follow(armExtensionMtrCtrlA);
        armExtensionMtrCtrlA.setInverted(true);

        armPivotMtrCtrlLT = new CANSparkMax(ARM_PIVOT_LT_MC_CAN_ID, MotorType.kBrushless);
        armPivotMtrCtrlRT = new CANSparkMax(ARM_PIVOT_RT_MC_CAN_ID, MotorType.kBrushless);

        armPivotMtrCtrlLT.follow(armPivotMtrCtrlRT);
        //armPivotMtrCtrlLT.follow(armPivotMtrCtrlRT, true); if needs to be inverted

        armPivotMtrCtrlRT.setIdleMode(IdleMode.kBrake);
        armPivotMtrCtrlLT.setIdleMode(IdleMode.kBrake);
 
        armPivotEnc = new AnalogInput(ARM_PIVOT_ENCODER_ANALOG_PORT);

        armExtendedLimitSwitch = new DigitalInput(ARM_EXTENSION_LIMIT_EXTENDED_DIO_PORT);
        armRetractedLimitSwitch = new DigitalInput(ARM_EXTENSION_LIMIT_RETRACTED_DIO_PORT);

        if(CatzConstants.USING_COMPETITION_ROBOT)
        {
            PIVOT_VOLTAGE_OFFSET = 0; // value for competition robot
        }
        else
        {
            PIVOT_VOLTAGE_OFFSET = 0; // value for robot 2
        }
    }


    public void extendArm(double power) 
    {
        armExtensionMtrCtrlA.set(power);
        /*

        if(getArmExtensionEncoderCounts() / ARM_COUNTS_PER_INCHES >= MAX_EXTENSION_LIMIT_INCHES)    //if extending past 30in, stop motor
        {
            armExtensionMtrCtrlA.set(0);
        }

        if(getArmExtensionEncoderCounts() <= 0 || getArmExtensionEncoderCounts() / ARM_COUNTS_PER_INCHES >= 46 ||
           isArmLimitExtendedActivated() || isArmLimitRetractedActivated()) 
         {
            armExtensionMtrCtrlA.set(0);
            
         }
        */
    }

    public void turnPivot(double power)
    {
        /*
        double pivotAngle = this.getPivotAngle();
        if(pivotAngle >= ARM_PIVOT_MAX_ANGLE && power >0) 
        {
            armPivotMtrCtrlLT.set(0);   
        }
        else if (pivotAngle <= ARM_PIVOT_MIN_ANGLE && power < 0)
        { 
            armPivotMtrCtrlLT.set(0);
        }  
        else 
        {
            armPivotMtrCtrlLT.set(power); 
        } 

        */
        armPivotMtrCtrlRT.set(power); 

    }
    public static int getArmExtensionEncoderCounts()
    {
        return 1;//armExtensionMtrCtrlA.getSensorCollection().getQuadraturePosition();
    }
    public static boolean isArmLimitExtendedActivated()
    {
        return armExtendedLimitSwitch.get();
    }
    public static boolean isArmLimitRetractedActivated()
    {
        return armRetractedLimitSwitch.get();
    }

    public static double getPivotAngle() 
    {   
        return ((armPivotEnc.getVoltage() + PIVOT_VOLTAGE_OFFSET) /ARM_PIVOT_ENC_MAX_VOLTAGE)*360.0;
    }
  
    public static void moveArmThread(double targetLength, double power, double timeOut)  //absolute
    {
        final double ARM_THREAD_WAITING_TIME = 0.005;

        Timer threadTimer = new Timer();
        threadTimer.start();

        Thread armExtensionThread = new Thread(() -> {

                int   currentCount = getArmExtensionEncoderCounts();
                double targetCount = (targetLength * ARM_COUNTS_PER_INCHES) - (double) currentCount;

                double upperLimit = targetCount + ARM_EXTENSION_COUNT_TOLERANCE;
                double lowerLimit = targetCount - ARM_EXTENSION_COUNT_TOLERANCE;

                if (currentCount < lowerLimit) 
                {
                    armExtensionMtrCtrlA.set(power);
                } 
                else if (currentCount > upperLimit)
                {
                    armExtensionMtrCtrlA.set(-power);
                }
                
                while (!Thread.interrupted()) 
                {
                    currentCount = getArmExtensionEncoderCounts(); //update the arm extension current Count

                    if((lowerLimit < currentCount && upperLimit > currentCount) || threadTimer.get() > timeOut)
                    {
                        armExtensionMtrCtrlA.stopMotor();
                        Thread.currentThread().interrupt();
                    }

                    Timer.delay(ARM_THREAD_WAITING_TIME);

                }
            } );

            armExtensionThread.start();
       
    }

    //Kyle custom pid loop
    public static void kPivot(double targetAngle, double timeout)
    {
        double kP = 0;
        double kPBase = 0;
        double kPComp = 0;



        Double prevError[] = {0.0,0.0,0.0};
        Double prevTime[] = {0.0,0.0,0.0};

        Thread kPivotThread = new Thread(() ->
        {
            while(!Thread.interrupted())
            {
                //gett rid of cosine to optimize code
                //calc kP
                //kP = kPComp * (maxArmextend + currentextend)/maxarmextend * Math.abs(Math.cos(armangle))+kPBase;

                prevError[2] = prevError[1];
                prevError[1] = prevError[0];
                //prevError[2] = currentError;


            }
        });
    }

    public static void turnArmPivotThread(double targetAngle, double power, double timeOut) { //no more than 270 deg

        final double ARM_PIVOT_THREAD_WAITING_TIME = 0.005;

        Timer threadTimer = new Timer();
        threadTimer.start();

        Thread armPivotThread = new Thread(() ->
        {
            double currentAngle = getPivotAngle();

            double errorAngle = Math.abs(targetAngle-currentAngle);

            double upperLimit = targetAngle + ARM_PIVOT_ANGLE_TOLERANCE;
            double lowerLimit = targetAngle - ARM_PIVOT_ANGLE_TOLERANCE;

            /* why are we extending the arm here? KH
            if (errorAngle < ARM_PIVOT_ANGLE_MAX/2.0) {  
                armExtensionMtrCtrlA.set(power);
            } else if(errorAngle > ARM_PIVOT_ANGLE_MAX/2.0) {
                armExtensionMtrCtrlA.set(-power);
            }
            */
            while(!Thread.interrupted()) 
            {
                currentAngle = getPivotAngle(); //update the currentAngle

                if((lowerLimit < currentAngle && upperLimit > currentAngle) || threadTimer.get() > timeOut) 
                {

                 armExtensionMtrCtrlA.stopMotor();
                 Thread.currentThread().interrupt();

                }

                Timer.delay(ARM_PIVOT_THREAD_WAITING_TIME);
            }
       
        });

        armPivotThread.start();
         
    }

}