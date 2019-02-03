package frc.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.CatzConstants;

//        Header

public class CatzDriveTrain { // static

    private static CANSparkMax drvTrainMtrCtrlLTFrnt;
    private static CANSparkMax drvTrainMtrCtrlLTMidl;
    private static CANSparkMax drvTrainMtrCtrlLTBack;

    private static CANSparkMax drvTrainMtrCtrlRTFrnt;
    private static CANSparkMax drvTrainMtrCtrlRTMidl;
    private static CANSparkMax drvTrainMtrCtrlRTBack;

    private final int DRVTRAIN_LT_FRNT_MC_CAN_ID = 1; 
    private final int DRVTRAIN_LT_MIDL_MC_CAN_ID = 2;
    private final int DRVTRAIN_LT_BACK_MC_CAN_ID = 3;

    private final int DRVTRAIN_RT_FRNT_MC_CAN_ID = 4; 
    private final int DRVTRAIN_RT_MIDL_MC_CAN_ID = 5; 
    private final int DRVTRAIN_RT_BACK_MC_CAN_ID = 6;

    private final int DRVTRAIN_MTR_CTRL_CURRENT_LIMIT = 80; //amps Recommended value from REV Robotics

    private static DifferentialDrive drvTrainDifferentialDrive;

    private SpeedControllerGroup drvTrainLT;
    private SpeedControllerGroup drvTrainRT;

    private static double drvTrainEncCounts = 0;
    private static double drvTrainEncPulsePerInch = 0; //TBD

   /* public static Encoder drvTrainEncoderLT;
    public static Encoder drvTrainEncoderRT;

    private final int DRVTRAIN_LT_ENCODER_A_DIO_PORT = 0; //TBD
    private final int DRVTRAIN_LT_ENCODER_B_DIO_PORT = 0;
    
    private final int DRVTRAIN_RT_ENCODER_A_DIO_PORT = 0;
	private final int DRVTRAIN_RT_ENCODER_B_DIO_PORT = 0; */

    private static Solenoid drvTrainToClimberShifter;
    private static final int DRVTRAIN_TO_CLIMBER_SOLENOID_PCM_PORT = 1;
    public CatzDriveTrain() {
        
        drvTrainMtrCtrlLTFrnt = new CANSparkMax(DRVTRAIN_LT_FRNT_MC_CAN_ID, MotorType.kBrushless);
        drvTrainMtrCtrlLTMidl = new CANSparkMax(DRVTRAIN_LT_MIDL_MC_CAN_ID, MotorType.kBrushless);
        drvTrainMtrCtrlLTBack = new CANSparkMax(DRVTRAIN_LT_BACK_MC_CAN_ID, MotorType.kBrushless);

        drvTrainMtrCtrlRTFrnt = new CANSparkMax(DRVTRAIN_RT_FRNT_MC_CAN_ID, MotorType.kBrushless);
        drvTrainMtrCtrlRTMidl = new CANSparkMax(DRVTRAIN_RT_MIDL_MC_CAN_ID, MotorType.kBrushless);
        drvTrainMtrCtrlRTBack = new CANSparkMax(DRVTRAIN_RT_BACK_MC_CAN_ID, MotorType.kBrushless);

        drvTrainMtrCtrlRTFrnt.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);
        drvTrainMtrCtrlRTMidl.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);
        drvTrainMtrCtrlRTBack.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);

        drvTrainMtrCtrlLTFrnt.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);
        drvTrainMtrCtrlLTMidl.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);
        drvTrainMtrCtrlLTBack.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);

        drvTrainLT = new SpeedControllerGroup(drvTrainMtrCtrlLTFrnt, drvTrainMtrCtrlLTMidl, drvTrainMtrCtrlLTBack);
        drvTrainRT = new SpeedControllerGroup(drvTrainMtrCtrlRTFrnt, drvTrainMtrCtrlRTMidl, drvTrainMtrCtrlRTBack);
 
        drvTrainDifferentialDrive = new DifferentialDrive(drvTrainLT, drvTrainRT); 

        //drvTrainEncoderLT = new Encoder(DRVTRAIN_LT_ENCODER_A_DIO_PORT,DRVTRAIN_LT_ENCODER_B_DIO_PORT,false,Encoder.EncodingType.k4X);
        //drvTrainEncoderRT = new Encoder(DRVTRAIN_RT_ENCODER_A_DIO_PORT,DRVTRAIN_RT_ENCODER_B_DIO_PORT,false,Encoder.EncodingType.k4X);

        drvTrainToClimberShifter = new Solenoid(CatzConstants.NAVX_RESET_WAIT_TIME, DRVTRAIN_TO_CLIMBER_SOLENOID_PCM_PORT); // has 2 ports on excel sheet???
    }

    public static void arcadeDrive(double xSpeed, double zRotataion) {
        drvTrainDifferentialDrive.arcadeDrive(xSpeed, zRotataion);
    }
    public static double getDriveTrainEncoderDistance()
    {
        drvTrainEncCounts = drvTrainMtrCtrlLTBack.getEncoder().getPosition();
        return drvTrainEncCounts / drvTrainEncPulsePerInch;
    }
    public static void resetDriveTrainEncoderCounts()
    {
        drvTrainEncCounts = 0;
    }
    public static void shiftToDrvTrain()
    {
        drvTrainToClimberShifter.set(true);
    }
    public static void shiftToClimber() {
        drvTrainToClimberShifter.set(false);
    }
}