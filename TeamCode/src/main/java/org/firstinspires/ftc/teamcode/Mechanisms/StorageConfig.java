package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ConstantValues.ConstantValues;

public class StorageConfig {

    private Servo storageServo;

    public void init(HardwareMap hwMap) {
        storageServo = hwMap.get(Servo.class , "storage");
    }

    ServoKick kick = new ServoKick();
    public void setIntakeA(){

        storageServo.setPosition(ConstantValues.sorterIntakeA);
    }
    public void setIntakeB(){

        storageServo.setPosition(ConstantValues.sorterIntakeB);
    }
    public void setIntakeC(){

        storageServo.setPosition(ConstantValues.sorterIntakeC);
    }
    public void setOutA(){

        storageServo.setPosition(ConstantValues.sorterOutTakeA);
    }
    public void setOutB(){

        storageServo.setPosition(ConstantValues.sorterOutTakeB);
    }
    public void setOutC(){

        storageServo.setPosition(ConstantValues.sorterOutTakeC);
    }

    public void setZero(){
        storageServo.setPosition(0.0);
    }


    public double getServoPos(){ return storageServo.getPosition();}

    public void setServo(double pos) {
        storageServo.setPosition(pos);
    }

}
