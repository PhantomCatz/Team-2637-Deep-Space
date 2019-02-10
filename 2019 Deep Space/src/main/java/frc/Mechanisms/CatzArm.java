/*
 *  Author : Jean Kwon
 * 
 * unctionality : controls the armextension by the speed, controls the arm pivot by the speed,
 *                  gets the status of each limit switch, gets the angle of the arm pivot,  
 *                  moves the arm extension into the target distance, moves the arm pivot into the targetAngle
 * 
 *  Methods : extension, pivot, getExtensionEncoderCounts, isArmLimitTipActivated, isArmLimitBaseActivated,
 *           getPivotAngle, moveArmExtension, moveArmPivot
 * 
 *  Revision History : 
 *  02-04-19 Added the theread and the encoder JK
 * 
 */
package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.CANSparkMax;
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

    private static DigitalInput armExtensionLimitTip;   
    private static DigitalInput armExtensionLimitBase;

    private final int ARM_EXTENSION_LIMIT_TIP_DIO_PORT  = 0; //TBD
    private final int ARM_EXTENSION_LIMIT_BASE_DIO_PORT = 0; 

    private static AnalogInput armPivotEnc;

    private static final int ARM_PIVOT_ENCODER_ANALOG_PORT = 0; //TBD
    private static final int ARM_PIVOT_ANGLE_TOLERANCE = 0; //TBD

     /* **************************************************************************
    * Arm Extension Encoder - pulses to inches 
    * SRX Magnetic Encoder which provides 4096 pulses per revolution. 
    * The gear reduction is 2 to 1.
    * The diameter of wrench is 0.984 inch 
    *****************************************************************************/

    private static final double ARM_EXTENSION_ENCODER_PULSE_PER_REV = 4096;
    private static final double ARM_EXTENSION_WINCH_DIAMETER = 0.984;
    private static final double ARM_EXTENSION_GEAR_RATIO = 1/2;
    private static final double ARM_COUNTS_PER_INCHES = ARM_EXTENSION_ENCODER_PULSE_PER_REV / 
                                                       (ARM_EXTENSION_WINCH_DIAMETER * Math.PI) * ARM_EXTENSION_GEAR_RATIO ;

    private static final double ARM_EXTENSION_COUNT_TOLERANCE = 100 * ARM_COUNTS_PER_INCHES; //TBD Type it in inches



    public CatzArm() {

        armExtensionMtrCtrlA = new WPI_TalonSRX(ARM_EXTENSION_A_MC_CAN_ID);
        armExtensionMtrCtrlB = new WPI_VictorSPX(ARM_EXTENSION_B_MC_CAN_ID);

        armExtensionMtrCtrlB.follow(armExtensionMtrCtrlA);

        armPivotMtrCtrlLT = new CANSparkMax(ARM_PIVOT_LT_MC_CAN_ID, MotorType.kBrushless);
        armPivotMtrCtrlRT = new CANSparkMax(ARM_PIVOT_RT_MC_CAN_ID, MotorType.kBrushless);

        armPivotMtrCtrlLT.follow(armPivotMtrCtrlRT);
        //armPivotMtrCtrlLT.follow(armPivotMtrCtrlRT, true); if needs to be inverted

        armExtensionLimitTip  = new DigitalInput(ARM_EXTENSION_LIMIT_TIP_DIO_PORT); 
        armExtensionLimitBase = new DigitalInput(ARM_EXTENSION_LIMIT_BASE_DIO_PORT); 

        armPivotEnc = new AnalogInput(ARM_PIVOT_ENCODER_ANALOG_PORT);
    }

    public static void extension(double power) {
        armExtensionMtrCtrlA.set(power);
    }
    public static void pivot(double power)
    {
        armPivotMtrCtrlRT.set(power);
    }
    public static int getExtensionEncoderCounts()
    {
        return armExtensionMtrCtrlA.getSensorCollection().getQuadraturePosition();
    }
    public static boolean isArmLimitTipActivated()
    {
        return armExtensionLimitTip.get();
    }
    public static boolean isArmLimitBaseActivated()
    {
        return armExtensionLimitBase.get();
    }

    public static double getPivotAngle() 
    {   
        return (armPivotEnc.getVoltage()/5)*360;
    }

    public static void moveArmExtension(double targetLength, double power, double timeOut)
    {
        Timer threadTimer = new Timer();
        threadTimer.start();

        Thread armExtensionThread = new Thread(() -> {

                int   currentCount = getExtensionEncoderCounts();
                double targetCount = targetLength * ARM_COUNTS_PER_INCHES;

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
                    currentCount = getExtensionEncoderCounts(); //update the arm extension current Count

                    if((lowerLimit < currentCount && upperLimit > currentCount) || threadTimer.get()>timeOut)
                    {
                        armExtensionMtrCtrlA.stopMotor();
                        Thread.currentThread().interrupt();
                    }

                    Timer.delay(0.005);

                }
            } );

            armExtensionThread.start();
       
    }

    public static void moveArmPivot(double targetAngle, double power, double timeOut) { //no more than 270 deg

        Timer threadTimer = new Timer();
        threadTimer.start();

        Thread armPivotThread = new Thread(() ->
        {
            double currentAngle = getPivotAngle();

            double upperLimit = targetAngle + ARM_PIVOT_ANGLE_TOLERANCE;
            double lowerLimit = targetAngle - ARM_PIVOT_ANGLE_TOLERANCE;

            if (targetAngle < 135) {  //270/2
                armExtensionMtrCtrlA.set(power);
            } else if(targetAngle > 135) {
                armExtensionMtrCtrlA.set(-power);
            }

            while(!Thread.interrupted()) 
            {
                currentAngle = getPivotAngle(); //update the currentAngle

                if((lowerLimit < currentAngle && upperLimit > currentAngle) || threadTimer.get()>timeOut) 
                {

                 armExtensionMtrCtrlA.stopMotor();
                 Thread.currentThread().interrupt();

                }

                Timer.delay(0.005);
            }
       
        });

        armPivotThread.start();
         
    }

}