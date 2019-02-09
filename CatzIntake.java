/**
 * Author : Jeffrey Li

 *  Methods : getCargo, relaseCargo, rotateWrist, stopWrist, closeCargoClamp, openCargoClamp
 *  Functionality : gets cargo and release cargo, start and stop wrist
 *    
 *  02-09-19
 */
package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class CatzIntake {

    private static WPI_VictorSPX intakeRollerMtrCtrl;
    private static WPI_TalonSRX intakeWristMtrCtrl;
    public static Solenoid hatchEjectSolenoid;
    public static Solenoid cargoClampSolenoid;

    private  final double CLAMP_EXECUTION_DURATION = 0.05;
    private final int INTAKE_ROLLER_MC_CAN_ID = 31;
    private final int INTAKE_WRIST_MC_CAN_ID = 30;
    private final int CARGO_CLAMP_PCM_PORT_NUMBER = 2;
    private final int HATCH_EJECT_PCM_PORT_NUMBER = 4;
    


    public static boolean hatchDeployed = false; 
    public static boolean cargoOpen = false;

    public static SolenoidState cargoState = SolenoidState.Closed;


    public enum SolenoidState {
	    Open(true), Closed(false);
		
	private boolean state;
		
		SolenoidState(boolean state){
			this.state = state;
		}
		
	public boolean getState() {
			return state;
		}
    }

//maybe logical arms open/closed
    
    public CatzIntake() {

        intakeRollerMtrCtrl = new WPI_VictorSPX(INTAKE_ROLLER_MC_CAN_ID);
        intakeWristMtrCtrl = new WPI_TalonSRX(INTAKE_WRIST_MC_CAN_ID);
        
        hatchEjectSolenoid = new Solenoid(HATCH_EJECT_PCM_PORT_NUMBER);
        cargoClampSolenoid = new Solenoid(CARGO_CLAMP_PCM_PORT_NUMBER);
    }

    public void closeCargoClamp() {
        cargoState = SolenoidState.Closed;
        cargoClampSolenoid.set(cargoState.getState());
    }

    public void openCargoClamp() {
        cargoState = SolenoidState.Open;
        cargoClampSolenoid.set(cargoState.getState());
    }
               
    public void getCargo(double power) { 
       intakeRollerMtrCtrl.set(power);
       Timer.delay(CLAMP_EXECUTION_DURATION); 
       intakeRollerMtrCtrl.set(0);
    }

    public void releaseCargo(double power) {
        intakeRollerMtrCtrl.set(-power);
        Timer.delay(CLAMP_EXECUTION_DURATION);
        intakeRollerMtrCtrl.set(0);
    }
    
    public void rotateWrist(double power) {
        intakeWristMtrCtrl.set(power);
    }

    public void stopWrist(double power){
        intakeWristMtrCtrl.set(0);
    }
}
