package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class CatzArm {

    private WPI_TalonSRX armExtensionMtrCtrlA;
    private WPI_VictorSPX armExtensionMtrCtrlB; 

    private CANSparkMax armPivotMtrCtrlLT;
    private CANSparkMax armPivotMtrCtrlRT;

    private final int ARM_EXTENSION_MTR_CTRL_ID_A = 20;
    private final int ARM_EXTENSION_MTR_CTRL_ID_B = 21;

    private int ARM_PIVOT_MTR_CTRL_ID_LT = 40;
    private int ARM_PIVOT_MTR_CTRL_ID_RT = 41;


public CatzArm() {

armExtensionMtrCtrlA = new WPI_TalonSRX(ARM_EXTENSION_MTR_CTRL_ID_A);
armExtensionMtrCtrlB = new WPI_VictorSPX(ARM_EXTENSION_MTR_CTRL_ID_B);

armExtensionMtrCtrlB.follow(armExtensionMtrCtrlA);

armPivotMtrCtrlLT = new CANSparkMax(ARM_PIVOT_MTR_CTRL_ID_LT, MotorType.kBrushless);
armPivotMtrCtrlRT = new CANSparkMax(ARM_PIVOT_MTR_CTRL_ID_RT, MotorType.kBrushless);


}

public static void armExtension(double speed) {


}

}