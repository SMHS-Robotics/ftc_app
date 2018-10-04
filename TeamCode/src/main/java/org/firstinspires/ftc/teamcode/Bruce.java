package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "THIS ISN'T EVEN MY FINAL FORM", group = "xd")
public class Bruce {
	

    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private Servo leftClaw;
    private Servo rightClaw;

    @Override
    public void runOpMode() throws InterruptedException
    {
        rightWheel = hardwareMap.dcMotor.get("rightMotor");
        leftWheel = hardwareMap.dcMotor.get("leftMotor");

        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            boolean isForward;
            boolean isBackward;

            // Send calculated power to wheels
            leftWheel.setPower((isForward? 1.0 : 0) + (isBackward? -1.0 : 0));
            rightWheel.setPower((isForward? -1.0 : 0) + (isBackward? 1.0 : 0));
        }
    }
}
