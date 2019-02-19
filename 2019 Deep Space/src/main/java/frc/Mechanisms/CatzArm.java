/*
 *  Author : Jean Kwon
 * 
 * Functionality : controls the armextension by the power, controls the arm pivot by the power,
 *                 gets the status of each limit switch, gets the angle of the arm pivot,  
 *                 moves the arm extension to the target distance, moves the arm pivot to the targetAngle
 * 
 *  Methods : extendArm, turnPivot, getArmExtensionEncoderCounts, isArmExtended, isArmRetracted,
 *           getPivotAngle, moveArmThread, turnArmPivot
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

    private static AnalogInput armPivotEnc;

  
    private static final int ARM_PIVOT_ENCODER_ANALOG_PORT = 1;
    private static final double ARM_PIVOT_ENC_MAX_VOLTAGE = 5.0;
  
    private static final double ARM_PIVOT_ANGLE_TOLERANCE = 5; //TBD,  place holding value

    
    private static final double ARM_PIVOT_ANGLE_MAX = 270.0;


     /* **************************************************************************
    * Arm Extension Encoder - pulses to inches 
    * Andy Mark Redline mag Encoder which provides 1024CPR
    * The gear reduction is 2 to 1.
    * The diameter of winch is 0.984 inch 
    * It attached to the same shaft
    *****************************************************************************/

    private static final double ARM_EXTENSION_ENCODER_PULSE_PER_REV = 1024;
    private static final double ARM_EXTENSION_WINCH_DIAMETER = 0.984;
    private static final double ARM_EXTENSION_GEAR_RATIO = 1/2; //TBD
    private static final double ARM_COUNTS_PER_INCHES = ARM_EXTENSION_ENCODER_PULSE_PER_REV / 
                                                       (ARM_EXTENSION_WINCH_DIAMETER * Math.PI) * ARM_EXTENSION_GEAR_RATIO ;

    private static final double ARM_EXTENSION_COUNT_TOLERANCE = 100 * ARM_COUNTS_PER_INCHES; //TBD Type it in inches

    private static AnalogInput armExtensionHallEffectSensor; 
    private static final int ARM_EXTENSION_HALL_EFFECT_SENSOR_PORT = 0; //TBD
    private static final double ARM_EXTENSION_EXTENDED = 4.0; //voltage
    private static final double ARM_EXTENSION_RETRACTED = 1.0; //voltage


    public CatzArm()
    {

        armExtensionMtrCtrlA = new WPI_TalonSRX(ARM_EXTENSION_A_MC_CAN_ID);
        armExtensionMtrCtrlB = new WPI_VictorSPX(ARM_EXTENSION_B_MC_CAN_ID);

        armExtensionMtrCtrlB.follow(armExtensionMtrCtrlA);

        armPivotMtrCtrlLT = new CANSparkMax(ARM_PIVOT_LT_MC_CAN_ID, MotorType.kBrushless);
        armPivotMtrCtrlRT = new CANSparkMax(ARM_PIVOT_RT_MC_CAN_ID, MotorType.kBrushless);

        armPivotMtrCtrlLT.follow(armPivotMtrCtrlRT);
        //armPivotMtrCtrlLT.follow(armPivotMtrCtrlRT, true); if needs to be inverted

        armPivotMtrCtrlRT.setIdleMode(IdleMode.kBrake);
        armPivotMtrCtrlLT.setIdleMode(IdleMode.kBrake);


        armPivotEnc = new AnalogInput(ARM_PIVOT_ENCODER_ANALOG_PORT);

        armExtensionHallEffectSensor = new AnalogInput(ARM_EXTENSION_HALL_EFFECT_SENSOR_PORT);
    }


    public void extendArm(double power) 
    {
        armExtensionMtrCtrlA.set(power);
    }
    public void turnPivot(double power)
    {
        armPivotMtrCtrlRT.set(power);
    }
    public static int getArmExtensionEncoderCounts()
    {
        return armExtensionMtrCtrlA.getSensorCollection().getQuadraturePosition();
    }
    public static boolean isArmExtended()
    {
       
        double currentHallEffectSensorVoltage = armExtensionHallEffectSensor.getVoltage();

        if (currentHallEffectSensorVoltage >= ARM_EXTENSION_EXTENDED) 
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    public static boolean isArmRetracted()
    {
    
        double currentHallEffectSensorVoltage = armExtensionHallEffectSensor.getVoltage();

        if (currentHallEffectSensorVoltage > ARM_EXTENSION_EXTENDED) //4
        {
            return true;
        }   
        else
        {
            return false;
        }
    }

    public static double getPivotAngle() 
    {   
        return (armPivotEnc.getVoltage()/ARM_PIVOT_ENC_MAX_VOLTAGE)*360.0;
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

    public static void turnArmPivot(double targetAngle, double power, double timeOut) { //no more than 270 deg

        final double ARM_PIVOT_THREAD_WAITING_TIME = 0.005;

        Timer threadTimer = new Timer();
        threadTimer.start();

        Thread armPivotThread = new Thread(() ->
        {
            double currentAngle = getPivotAngle();

            double errorAngle = Math.abs(targetAngle-currentAngle);

            double upperLimit = targetAngle + ARM_PIVOT_ANGLE_TOLERANCE;
            double lowerLimit = targetAngle - ARM_PIVOT_ANGLE_TOLERANCE;

            if (currentAngle < lowerLimit) 
            {  
                armExtensionMtrCtrlA.set(power);
            } 
            else if(currentAngle > upperLimit) 
            {
                armExtensionMtrCtrlA.set(-power);
            }

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