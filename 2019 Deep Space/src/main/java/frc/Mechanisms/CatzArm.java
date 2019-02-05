/*
 *  Author : Jean Kwon

 *  Methods : extension, pivot, getExtensionEncoderCounts, isArmLimitTipActivated, isArmLimitBaseActivated,
 *           getPivotAngle, setArmExtension
 *  Functionality : controls the armextension by the speed, controls the arm pivot by the speed,
 *                   gets the status of each limit switch, gets the angle of the arm pivot,  
 *                   sets the arm extension in to the target distance
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



public class CatzArm //static variables/objects
{

    private static WPI_TalonSRX armExtensionMtrCtrlA;
    private static WPI_VictorSPX armExtensionMtrCtrlB;

    private static CANSparkMax armPivotMtrCtrlLT;
    private static CANSparkMax armPivotMtrCtrlRT;

    private final int ARM_EXTENSION_A_MC_ID = 20;
    private final int ARM_EXTENSION_B_MC_ID = 21;

    private final int ARM_PIVOT_LT_MC_CAN_ID = 40;
    private final int ARM_PIVOT_RT_MC_CAN_ID = 41;

    private static DigitalInput armExtensionLimitTip;
    private static DigitalInput armExtensionLimitBase;

    private static AnalogInput armPivotEnc;

    private static final int ARM_PIVOT_ENCODER_ANALOG_PORT = 0; //TBD
    
    private static       double targetCount;
    private static final int ARM_COUNT_THRESHOLD = 100; //TBD
    private static final int ARM_COUNTS_PER_INCHES = 0; //TBD

    public CatzArm() {

        armExtensionMtrCtrlA = new WPI_TalonSRX(ARM_EXTENSION_A_MC_ID);
        armExtensionMtrCtrlB = new WPI_VictorSPX(ARM_EXTENSION_B_MC_ID);

        armExtensionMtrCtrlB.follow(armExtensionMtrCtrlA);

        armPivotMtrCtrlLT = new CANSparkMax(ARM_PIVOT_LT_MC_CAN_ID, MotorType.kBrushless);
        armPivotMtrCtrlRT = new CANSparkMax(ARM_PIVOT_RT_MC_CAN_ID, MotorType.kBrushless);

        armPivotMtrCtrlLT.follow(armPivotMtrCtrlRT);
        //armPivotMtrCtrlLT.follow(armPivotMtrCtrlRT, true); if needs to be inverted

        armExtensionLimitTip  = new DigitalInput(0); //TBD
        armExtensionLimitBase = new DigitalInput(0); //TBD

        armPivotEnc = new AnalogInput(ARM_PIVOT_ENCODER_ANALOG_PORT);
    }

    public static void extension(double speed) {
        armExtensionMtrCtrlA.set(speed);
    }
    public static void pivot(double speed)
    {
        armPivotMtrCtrlRT.set(speed);
    }
    public static double getExtensionEncoderCounts()
    {
        return armExtensionMtrCtrlA.getSelectedSensorPosition();
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

    public static void setArmExtension(double trargetLength, double speed)
    {
        targetCount = trargetLength * ARM_COUNTS_PER_INCHES;

        Thread armExtensionThread = new Thread(() -> {

            while (!Thread.interrupted()) 
            {
                while(getExtensionEncoderCounts() < targetCount-ARM_COUNT_THRESHOLD) 
                {
                    extension(speed);
                }
        
                while(getExtensionEncoderCounts() > targetCount+ARM_COUNT_THRESHOLD)
                {
                    extension(-speed);
                }
        
                armExtensionMtrCtrlA.stopMotor();

                if(targetCount-ARM_COUNT_THRESHOLD<getExtensionEncoderCounts()&&targetCount+ARM_COUNT_THRESHOLD>getExtensionEncoderCounts())
                {
                    Thread.currentThread().interrupt();
                }
            }

        } );

        armExtensionThread.start();
        
    }
}