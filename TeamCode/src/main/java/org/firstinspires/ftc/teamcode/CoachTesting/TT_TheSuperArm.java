package org.firstinspires.ftc.teamcode.CoachTesting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TT_TheSuperArm {

    private DcMotorEx motor;

    public TT_TheSuperArm(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "arm");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public int Move(double power) {
        if (power > 0 && motor.getCurrentPosition() >= 0) {
            power = 0;
        } else if (power < 0 && motor.getCurrentPosition() <= -1000) {
            power = 0;
        }

        motor.setPower(power);
        return motor.getCurrentPosition();
    }
}