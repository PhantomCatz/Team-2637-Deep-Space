package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

/*
 *  Author : 

 *  Methods : 
 *  Functionality : 
 *   
 *  Revision History : 
 * 
 */
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

    private Thread armThread;

    public CatzArm() {

        armExtensionMtrCtrlA = new WPI_TalonSRX(ARM_EXTENSION_A_MC_ID);
        armExtensionMtrCtrlB = new WPI_VictorSPX(ARM_EXTENSION_B_MC_ID);

        armExtensionMtrCtrlB.follow(armExtensionMtrCtrlA);

        armPivotMtrCtrlLT = new CANSparkMax(ARM_PIVOT_LT_MC_CAN_ID, MotorType.kBrushless);
        armPivotMtrCtrlRT = new CANSparkMax(ARM_PIVOT_RT_MC_CAN_ID, MotorType.kBrushless);

        armPivotMtrCtrlLT.follow(armPivotMtrCtrlRT);
        armPivotMtrCtrlLT.follow(armPivotMtrCtrlLT);

        
        //armPivotMtrCtrlLT.follow(armPivotMtrCtrlRT, true); if needs to be inverted

        armExtensionLimitTip  = new DigitalInput(0); //TBD
        armExtensionLimitBase = new DigitalInput(0); //TBD
    }

    public void extension(double speed) 
    {
        armExtensionMtrCtrlA.set(speed);
    }
    public void pivot(double speed)
    {
        armPivotMtrCtrlRT.set(speed);
    }
    public double extensionEncoderCounts()
    {
        return armExtensionMtrCtrlA.getSelectedSensorPosition();
    }
    public boolean getArmLimitTip()
    {
        return armExtensionLimitTip.get();
    }
    public boolean getArmLimitBase()
    {
        return armExtensionLimitBase.get();
    }
    public void setToBotPos()
    {
        armThread = new Thread(() -> 
        {
            while(true)
                armExtensionMtrCtrlA.set(1);
        });
        armThread.start();
    }
}