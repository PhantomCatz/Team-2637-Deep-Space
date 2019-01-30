package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

//        Header
public class CatzIntake {

    private WPI_VictorSPX intkMtrCtrlA;

    private final int INTK_MTR_CTRL_ID_A = 31;

    public CatzIntake() {

        intkMtrCtrlA = new WPI_VictorSPX(INTK_MTR_CTRL_ID_A);
    }

    public void intake() { // need to be able to set speed, changing

        intkMtrCtrlA.set(.65);

    }

    public void outtake() {
        intkMtrCtrlA.set(-.65);

    }

}