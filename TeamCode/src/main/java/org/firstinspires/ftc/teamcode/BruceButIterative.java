package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "ITERATIVE", group = "nextD")
public class BruceButIterative extends OpMode {
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private Servo leftClaw;
    private Servo rightClaw;

    @Override
    public void init(){
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        leftWheel = hardwareMap.dcMotor.get("left_drive");

        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        double LeftPower = (gamepad1.left_bumper? 1.0 : 0) + ((gamepad1.left_trigger > 0.5) ? -1.0 : 0);
        telemetry.addData("Say", LeftPower);
        double RightPower = (gamepad1.right_bumper ? 1.0 : 0) + ((gamepad1.right_trigger > 0.5) ? -1.0 : 0);
        // Send calculated power to wheels
            /*double powerLeft = (isForward? 1.0 : 0) + (isBackward? -1.0 : 0) + (isLeft? 0.5 : 0) + (isRight? -0.5 : 0);
            double powerRight = (isForward? 1.0 : 0) + (isBackward? -1.0 : 0) +  + (isLeft? -0.5 : 0) + (isRight? 0.5 : 0);
            if(powerLeft > 1.0) powerLeft = 1.0;
            if(powerLeft < -1.0) powerLeft = -1.0;
            if(powerRight > 1.0) powerRight = 1.0;
            if(powerRight < -1.0) powerRight = -1.0;*/
        leftWheel.setPower(LeftPower);
        rightWheel.setPower(RightPower);
        telemetry.update();
    }

    @Override
    public void stop(){

    }
}