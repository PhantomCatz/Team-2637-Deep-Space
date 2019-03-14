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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.Robot;


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

    private static DigitalInput armRetractedLimitSwitch;

    private final int ARM_EXTENSION_LIMIT_EXTENDED_DIO_PORT  = 0; //TODO, TBD, same placeholding values woul conflict
    private final int ARM_EXTENSION_LIMIT_RETRACTED_DIO_PORT = 1;

    public static AnalogInput armPivotEnc;
  
    private static final int    ARM_PIVOT_ENCODER_ANALOG_PORT = 1;
    private static final double ARM_PIVOT_ENC_MAX_VOLTAGE     = 5.0;
  
    private static final int    ARM_PIVOT_ANGLE_TOLERANCE = 3; //TBD
    
    //private static final double ARM_PIVOT_ANGLE_MAX = 270.0;

  //  private static final double MAX_EXTENSION_LIMIT_INCHES = 30 / Math.cos(Math.abs(getPivotAngle()));

    public static Encoder armExtensionEnc;

     /* **************************************************************************
    * Arm Extension Encoder - pulses to inches 
    * SRX Magnetic Encoder which provides 4096 pulses per revolution. 
    * The gear reduction is 2 to 1.
    * The diameter of winch is 0.984 inch 
    * It attached to the same shaft
    *****************************************************************************/

    private static final double ARM_EXTENSION_ENCODER_PULSE_PER_REV = 4096;
    private static final double ARM_EXTENSION_WINCH_DIAMETER        = 0.984;
    private static final double ARM_EXTENSION_GEAR_RATIO            = 0.5; //TBD
    private static final double ARM_COUNTS_PER_INCHES = (ARM_EXTENSION_ENCODER_PULSE_PER_REV / 
                                                        (ARM_EXTENSION_WINCH_DIAMETER * Math.PI)) * ARM_EXTENSION_GEAR_RATIO ;

    private static final double ARM_EXTENSION_COUNT_TOLERANCE = 100 * ARM_COUNTS_PER_INCHES; //TBD Type it in inches

    private static final double ARM_PIVOT_MAX_ANGLE = 120.0;   //Robot 0 deg = Arm pointing down -45 deg
    private static final double ARM_PIVOT_MIN_ANGLE = 5.0;          //TBD

    private static double PIVOT_VOLTAGE_OFFSET;

    private static double ARM_EXTENSION_MAX_POWER_RAMP_TIME = 0.5; //sec
    private static double ARM_PIVOT_MAX_POWER_RAMP_TIME     = 0.5; //sec

    private final double ARM_PIVOT_UP_LIMIT = 0.8;
    private final double ARM_PIVOT_DN_LIMIT = 0.4;

    private final double PIVOT_STOWED_ANGLE; // = getPivotAngle();
    private final double CONTROLLER_INPUT_WAIT_TIME = 0.020;

    public static volatile double PIVOT_DEBUG_KP = 0;
    public static volatile double PIVOT_DEBUG_KD = 0;
    public static volatile double PIVOT_DEBUG_KA = 0;
    public static volatile double PIVOT_DEBUG_KC = 0;

    public static volatile double WRIST_DEBUG_KP = 0;
    public static volatile double WRIST_DEBUG_KD = 0;
    public static volatile double WRIST_DEBUG_KA = 0;
    public static volatile double WRIST_DEBUG_KC = 0;

    private static volatile double targetAngle;

    public CatzArm()
    {
        armExtensionMtrCtrlA = new WPI_TalonSRX (ARM_EXTENSION_A_MC_CAN_ID);
        armExtensionMtrCtrlB = new WPI_VictorSPX(ARM_EXTENSION_B_MC_CAN_ID);

        armExtensionMtrCtrlA.setNeutralMode(NeutralMode.Brake);
        armExtensionMtrCtrlB.setNeutralMode(NeutralMode.Brake);

        armExtensionMtrCtrlB.follow(armExtensionMtrCtrlA);
        armExtensionMtrCtrlA.setInverted(true);
        //armExtensionMtrCtrlA.configOpenloopRamp(ARM_EXTENSION_MAX_POWER_RAMP_TIME);
        //armExtensionMtrCtrlB.configOpenloopRamp(ARM_EXTENSION_MAX_POWER_RAMP_TIME);

        armExtensionEnc = new Encoder(6,7, false, EncodingType.k4X);
        
        //armExtensionEnc.setDistancePerPulse(1/ARM_COUNTS_PER_INCHES);

        armPivotMtrCtrlLT = new CANSparkMax(ARM_PIVOT_LT_MC_CAN_ID, MotorType.kBrushless);
        armPivotMtrCtrlRT = new CANSparkMax(ARM_PIVOT_RT_MC_CAN_ID, MotorType.kBrushless);

        armPivotMtrCtrlLT.follow(armPivotMtrCtrlRT);

        armPivotMtrCtrlRT.setIdleMode(IdleMode.kBrake);
        armPivotMtrCtrlLT.setIdleMode(IdleMode.kBrake);

        armPivotMtrCtrlLT.setOpenLoopRampRate(ARM_PIVOT_MAX_POWER_RAMP_TIME);
        armPivotMtrCtrlRT.setOpenLoopRampRate(ARM_PIVOT_MAX_POWER_RAMP_TIME);
 
        armPivotEnc = new AnalogInput(ARM_PIVOT_ENCODER_ANALOG_PORT);

        armRetractedLimitSwitch = new DigitalInput(ARM_EXTENSION_LIMIT_RETRACTED_DIO_PORT);

        if(CatzConstants.USING_COMPETITION_ROBOT)
        {
            PIVOT_VOLTAGE_OFFSET = -3.55; // value for lift inner stage serial #2
        }
        else
        {
            PIVOT_VOLTAGE_OFFSET = -3.55; // value for lift inner stage serial #2
        }

        PIVOT_STOWED_ANGLE = 5.0;   //getPivotAngle();
    }


    public void extendArm(double power) 
    {
        armExtensionMtrCtrlA.set(power);
        SmartDashboard.putNumber("arm pow", power);
        //System.out.println(power);
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
        if(false)//CatzConstants.USING_SOFT_LIMITS)
        {
            double pivotAngle = this.getPivotAngle();
            System.out.println(power);
            // Value from X-Box controller is negative when joystick is pushed UP
            if(power == 0.0)
            {
                setPivotTargetAngle(pivotAngle);
            }
            else if (power >  0.0) 
            {
                // Pivot is being commanded CCW (Increasing Angle)
                setPivotTargetAngle(CatzConstants.INVALID_ANGLE);
                
                if(pivotAngle >= ARM_PIVOT_MAX_ANGLE)
                {
                    armPivotMtrCtrlRT.set(0);   
                }
                else
                {
                    armPivotMtrCtrlRT.set(ARM_PIVOT_UP_LIMIT * power);
                }
            }
            
            else
            { 
                // Pivot is being commanded to Stop or go CW (Decreasing Angle)
                setPivotTargetAngle(CatzConstants.INVALID_ANGLE);
                if (pivotAngle <= ARM_PIVOT_MIN_ANGLE)
                {
                    armPivotMtrCtrlRT.set(0);
                }
                else
                {
                    armPivotMtrCtrlRT.set(ARM_PIVOT_DN_LIMIT * power);
                }
            }
        }
        else
        {
            armPivotMtrCtrlRT.set(power); 
        }
    }

    /* Matt's jank code. Please ignore.
    public void lockPivot(double power)
    {
        isLocked = true;
        lockedPower = power;
    }

    public boolean isLocked()
    {
        return isLocked;
    }

    public void unlockPivot()
    {
        isLocked = false;
    }
    */

    public static int getArmExtensionEncoderCounts()
    {
        return armExtensionEnc.get();
    }
    //returns extension in inches
    public double getArmExtensionDistance()
    {
        return (((double) armExtensionEnc.get()) / ARM_COUNTS_PER_INCHES);
    }

    public static boolean isArmLimitRetractedActivated()
    {
        return armRetractedLimitSwitch.get();
    }

    public static double getPivotAngle() 
    {   
        return (((armPivotEnc.getVoltage() + PIVOT_VOLTAGE_OFFSET) / ARM_PIVOT_ENC_MAX_VOLTAGE) * 360.0);
    }
  
    public static void moveArmThread(double targetLength, double timeOut)  //absolute
    {
        final double ARM_THREAD_WAITING_TIME = 0.005;

        Timer threadTimer = new Timer();
        threadTimer.start();

        Thread armExtensionThread = new Thread(() -> {

                
                double triggerInput = Robot.xboxAux.getTriggerAxis(Hand.kRight) - Robot.xboxAux.getTriggerAxis(Hand.kLeft);

                int   currentCount = getArmExtensionEncoderCounts();
                double targetCount = (targetLength * ARM_COUNTS_PER_INCHES) - (double) currentCount;
                /*
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
                why are we setting power here? this should be a PID loop KH
                */ 
                while (!Thread.interrupted()) 
                {
                    currentCount = getArmExtensionEncoderCounts(); //update the arm extension current Count

                    if(triggerInput != 0|| threadTimer.get() > timeOut)
                    {
                        armExtensionMtrCtrlA.stopMotor();
                        Thread.currentThread().interrupt();
                    }

                    //PID loop here



                    Timer.delay(ARM_THREAD_WAITING_TIME);

                }
            } );

            armExtensionThread.start();
       
    }


    public void setPivotTargetAngle(double angle)
    {
        targetAngle = angle;
    }

    public void pivotPID()
    {
        Thread t = new Thread(() ->
        {
            setPivotTargetAngle(PIVOT_STOWED_ANGLE);
            final double ARM_PIVOT_THREAD_WAITING_TIME = 0.005;
            final double kP = 0.008; //TODO
            final double kD = 0.0005;
            final double kA = 0.0077;
            
            double power;            

            Timer armTimer = new Timer();
            armTimer.start();

            double previousError = 0;
            double currentError;
            double deltaError = 0;

            /*double previousDerivative;
            double currentDerivative;    // in case you want to filter derivative
            double filteredDerivative;*/
            
            double previousTime = 0;
            
            double deltaTime;
            
            double currentTime  = armTimer.get();
            double currentAngle = getPivotAngle();
            while(true)
            {

                if(targetAngle >= CatzConstants.INVALID_ANGLE)
                {
                    Timer.delay(CatzConstants.CONTROLLER_INPUT_WAIT_TIME);
                }
                else
                {
                currentTime  = armTimer.get();
                currentAngle = getPivotAngle();

                currentError = targetAngle - currentAngle;
                
                deltaError = currentError - previousError;
                deltaTime = currentTime - previousTime;

                //currentDerivative = (deltaError / deltaTime);
                //filteredDerivative = (0.7 * currentDerivative) + (0.3 * previousDerivative)

                power = kP * currentError +
                        kD * (deltaError / deltaTime);
                     //   kA * (Math.cos(currentAngle-45)*(getArmExtensionDistance()+13));//ka compensates for angle of arm
                        //arm extension distaNce + 13 is the distance from pivot to wrist

                        if (power > 0.8)
                        {
                            power = 0.8;
                        }
                        else if (power < -0.4)
                        {
                            power = -0.4;
                        }

                turnPivot(-power);
                System.out.println("AP "+ currentAngle + ",  "+ -power);

                previousError = currentError;
                previousTime = currentTime;

                //previousDerivative = currentDerivative;
                
                Timer.delay(ARM_PIVOT_THREAD_WAITING_TIME);

                    
                }
                
            }
             
        });
        t.start();
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