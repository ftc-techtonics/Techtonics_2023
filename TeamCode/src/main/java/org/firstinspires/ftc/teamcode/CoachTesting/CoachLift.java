package org.firstinspires.ftc.teamcode.CoachTesting;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Comments about author, etc...
public class CoachLift
{
    public static enum LEVEL {DOWN, LOW, MEDIUM, HIGH}
    public int motorEncoderMin = 0;
    public int motorEncoderMax = 2200;

    //todo determine ticks per inch including radius of lift hub or just through testing

    private DcMotorEx motor;
    private LEVEL currentLevel = LEVEL.DOWN;
    private LEVEL targetLevel = LEVEL.DOWN;
    private boolean moving = false;
    private boolean automatic = true;

    private RevTouchSensor motorStartPosition;
    //todo add public variables for accessing current state of current, target, moving, etc...

    public CoachLift(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "lift");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorStartPosition = hardwareMap.get(RevTouchSensor.class, "liftbutton");
    }

    public boolean LiftInStartPosition () {
        return motorStartPosition.isPressed();
    }
    public void SwitchToManual() { automatic = false;}
    public void SwitchToAuto () {  automatic = true; }
    public boolean IsAutomaticOn() { return automatic;}

    public boolean ResetLift(){
        while (!motorStartPosition.isPressed()) {  // Not pressed
            motor.setPower(.3);
        }
        motor.setPower(0);
        return true;
    }
    public void MoveLift (LEVEL level){
        targetLevel = level;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(10);
        motor.setPower(1);
        // todo wait until position
        while (motor.isBusy()) {

        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SetTargetHeight (LEVEL level){
        targetLevel = level;
        //todo translate LEVEL positions to ticks
        motor.setTargetPosition(10);
        if (targetLevel != currentLevel)
            moving = true;
        else
            moving = false;
    }

    public int Move (double power) {

       /* if (targetLevel == currentLevel){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return true;
        }
        else {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
            return false;
        }*/

        if ((power < 0 && motor.getCurrentPosition() >= motorEncoderMax)
            || (power > 0 && motor.getCurrentPosition() <= motorEncoderMin) ) {
                motor.setPower(0);
        }
        else
        {
                motor.setPower(power);
        }
        return motor.getCurrentPosition();
    }
}
