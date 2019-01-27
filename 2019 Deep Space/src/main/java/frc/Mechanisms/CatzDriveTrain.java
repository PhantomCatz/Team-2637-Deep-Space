package frc.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class CatzDriveTrain {

    private CANSparkMax drvTrainMtrCtrlLTFrnt;
    private CANSparkMax drvTrainMtrCtrlLTMidl;
    private CANSparkMax drvTrainMtrCtrlLTBack;

    private CANSparkMax drvTrainMtrCtrlRTFrnt;
    private CANSparkMax drvTrainMtrCtrlRTMidl;
    private CANSparkMax drvTrainMtrCtrlRTBack;

    private final int DRVTRAIN_MTR_CTRL_ID_LT_FRNT = 1; 
    private final int DRVTRAIN_MTR_CTRL_ID_LT_MIDL = 2;
    private final int DRVTRAIN_MTR_CTRL_ID_LT_BACK = 3;

    private final int DRVTRAIN_MTR_CTRL_ID_RT_FRNT = 4; 
    private final int DRVTRAIN_MTR_CTRL_ID_RT_MIDL = 5; 
    private final int DRVTRAIN_MTR_CTRL_ID_RT_BACK = 6;

    private final int DRVTRAIN_MTR_CTRL_CURRENT_LIMIT = 80; //amps Recommended value from REV Robotics

    private  DifferentialDrive drvTrainDifferentialDrive;

    private SpeedControllerGroup drvTrainLT;
    private SpeedControllerGroup drvTrainRT;

    public CatzDriveTrain() {
        
        drvTrainMtrCtrlLTFrnt = new CANSparkMax(DRVTRAIN_MTR_CTRL_ID_LT_FRNT, MotorType.kBrushless);
        drvTrainMtrCtrlLTMidl = new CANSparkMax(DRVTRAIN_MTR_CTRL_ID_LT_MIDL, MotorType.kBrushless);
        drvTrainMtrCtrlLTBack = new CANSparkMax(DRVTRAIN_MTR_CTRL_ID_LT_BACK, MotorType.kBrushless);

        drvTrainMtrCtrlRTFrnt = new CANSparkMax(DRVTRAIN_MTR_CTRL_ID_RT_FRNT, MotorType.kBrushless);
        drvTrainMtrCtrlRTMidl = new CANSparkMax(DRVTRAIN_MTR_CTRL_ID_RT_MIDL, MotorType.kBrushless);
        drvTrainMtrCtrlRTBack = new CANSparkMax(DRVTRAIN_MTR_CTRL_ID_RT_BACK, MotorType.kBrushless);

        drvTrainMtrCtrlRTFrnt.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);
        drvTrainMtrCtrlRTMidl.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);
        drvTrainMtrCtrlRTBack.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);

        drvTrainMtrCtrlLTFrnt.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);
        drvTrainMtrCtrlLTMidl.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);
        drvTrainMtrCtrlLTBack.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);

        drvTrainLT = new SpeedControllerGroup(drvTrainMtrCtrlLTFrnt, drvTrainMtrCtrlLTMidl, drvTrainMtrCtrlLTBack);
        drvTrainRT = new SpeedControllerGroup(drvTrainMtrCtrlRTFrnt, drvTrainMtrCtrlRTMidl, drvTrainMtrCtrlRTBack);
 
        drvTrainDifferentialDrive = new DifferentialDrive(drvTrainLT, drvTrainRT); 


    }

    public void arcadeDrive(double xSpeed, double zRotataion) {
        drvTrainDifferentialDrive.arcadeDrive(xSpeed, zRotataion);
    }

 

}