package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Kicker {
    Servo kick;
    //push

    public void init(HardwareMap hwMap){
        kick = hwMap.get(Servo.class, "kick2");
    }

    public void kick(){
        kick.setPosition(ConstantValues.kick);
    }

    public void retract(){
        kick.setPosition(ConstantValues.retract);
    }

    public void zeroKick() {
        kick.setPosition(0.0);
    }
}
