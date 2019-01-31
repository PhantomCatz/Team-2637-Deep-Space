package frc.Mechanisms;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

//        Header
public class CatzIntake {

    private static WPI_VictorSPX intakeRoller;
    private final int INTAKE_ROLLER_MC_CAN_ID = 31;

    private static WPI_TalonSRX intakeWrist;
    private final int INTAKE_WRIST_MC_CAN_ID = 30;

   

    public CatzIntake() {

        intakeRoller = new WPI_VictorSPX(INTAKE_ROLLER_MC_CAN_ID);
        intakeWrist = new WPI_TalonSRX(INTAKE_WRIST_MC_CAN_ID);

       }

    public void intake(double speed) { 
        intakeRoller.set(speed);

    }
    public void outtake(double speed)
     {
        intakeRoller.set(-speed);
    }
    public void rotateWrist(double speed)
    {
        intakeWrist.set(speed);
    }
    public static double wristEncoderCounts()
    {
        return intakeWrist.getSelectedSensorPosition();
    }
}