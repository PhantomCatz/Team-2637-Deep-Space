package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;

public class CatzArm {

    private WPI_TalonSRX armExtensionMtrCtrlA;
    private WPI_VictorSPX armExtensionMtrCtrlB; 

    private CANSparkMax armPivotMtrCtrlLT;
    private CANSparkMax armPivotMtrCtrlRT;

    private final int ARM_EXTENSION_MTR_CTRL_ID_A = 20;
    private final int ARM_EXTENSION_MTR_CTRL_ID_B = 21;

    private final int ARM_PIVOT_MTR_CTRL_ID_LT = 40;
    private final int ARM_PIVOT_MTR_CTRL_ID_RT = 41;

    public static Encoder armExtensionEncoder;

    private final int ARM_EXTENSION_ENCODER_A_DIO_PORT = 0; //TBD
    private final int ARM_EXTENSION_ENCODER_B_DIO_PORT = 0;


public CatzArm() {

armExtensionMtrCtrlA = new WPI_TalonSRX(ARM_EXTENSION_MTR_CTRL_ID_A);
armExtensionMtrCtrlB = new WPI_VictorSPX(ARM_EXTENSION_MTR_CTRL_ID_B);

armExtensionMtrCtrlB.follow(armExtensionMtrCtrlA);

armPivotMtrCtrlLT = new CANSparkMax(ARM_PIVOT_MTR_CTRL_ID_LT, MotorType.kBrushless);
armPivotMtrCtrlRT = new CANSparkMax(ARM_PIVOT_MTR_CTRL_ID_RT, MotorType.kBrushless);

armExtensionEncoder = new Encoder(ARM_EXTENSION_ENCODER_A_DIO_PORT, ARM_EXTENSION_ENCODER_B_DIO_PORT, false, Encoder.EncodingType.k4X);


}

public void armExtension(double speed) {

    armExtensionMtrCtrlA.set(speed);

}

public void armExtension(double speed, double distance) {
    while(armExtensionEncoder.getDistance()<distance) {
        armExtensionMtrCtrlA.set(speed);
    }
}

}