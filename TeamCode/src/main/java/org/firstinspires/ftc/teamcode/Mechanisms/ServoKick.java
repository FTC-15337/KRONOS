package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ConstantValues.ConstantValues;

public class ServoKick {
    Servo kick;

    public void init(HardwareMap hwMap){
        kick = hwMap.get(Servo.class, "kick2");
        kick.setPosition(ConstantValues.retract);
    }

    public void kick(){
        kick.setPosition(ConstantValues.kick);
    }

    public void retract(){
        kick.setPosition(ConstantValues.retract);
    }

}
