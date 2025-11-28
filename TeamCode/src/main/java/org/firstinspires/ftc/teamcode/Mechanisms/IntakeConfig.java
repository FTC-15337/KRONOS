package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeConfig {
    private DcMotor intake;
    public void init(HardwareMap hwMap){
        intake = hwMap.get(DcMotor.class , "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void intakeMax(){
        intake.setPower(ConstantValues.intakeMax);
        }
        public void intakeStop(){
        intake.setPower(ConstantValues.intakeStop);
        }
        public void reverseIntake(){
        intake.setPower(ConstantValues.intakeReverse
        );
        }
}

