package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ConstantValues.ConstantValues;

public class IntakeConfig {
    private DcMotor intakeMotorRight;
    private DcMotor intakeMotorLeft;

    StorageConfig sorter = new StorageConfig();

    public void init(HardwareMap hwMap){

        intakeMotorRight = hwMap.get(DcMotor.class , "intakeMotorR");
        intakeMotorLeft = hwMap.get(DcMotor.class , "intakeMotorL");

        intakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void IntakeMotorStop(){
        intakeMotorRight.setPower(ConstantValues.intakeStop);
        intakeMotorLeft.setPower(ConstantValues.intakeStop);
    }


    public void IntakeMotorMax(){
        intakeMotorRight.setPower(ConstantValues.intakeMax);
        intakeMotorLeft.setPower(ConstantValues.intakeMax);
    }

    public void OutIntake(){
        intakeMotorRight.setPower(ConstantValues.intakeRev);
        intakeMotorLeft.setPower(ConstantValues.intakeRev);
    }
    public void intake(){
        ElapsedTime timer = new ElapsedTime();
        IntakeMotorMax();
        if(timer.seconds() < 1 && timer.seconds() > 0){
            sorter.setIntakeC();
        }
        if(timer.seconds() < 2 && timer.seconds() > 1){
            sorter.setIntakeB();
        }
        if(timer.seconds() < 3 && timer.seconds() > 2){
            sorter.setIntakeC();
        }
        if(timer.seconds() > 3){
            IntakeMotorStop();
        }
    }
}
