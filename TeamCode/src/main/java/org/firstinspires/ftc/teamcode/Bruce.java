package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "ANOTHER FORM", group = "nextD")
public class Bruce extends LinearOpMode
{
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private Servo leftClaw;
    private Servo rightClaw;
    private final double INTERVAL = 0.005;

    @Override
    public void runOpMode() throws InterruptedException
    {
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        leftClaw = hardwareMap.servo.get("left_claw");
        rightClaw = hardwareMap.servo.get("right_claw");

        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.FORWARD);

        float clawPos = 0; //how open or closed the claw is

        waitForStart();

        while (opModeIsActive())
        {
            double leftPower = (gamepad1.left_bumper ? 1.0 : 0) + ((gamepad1.left_trigger > 0.5) ? -1.0 : 0);
            telemetry.addData("Say ", leftPower);
            double rightPower = (gamepad1.right_bumper ? 1.0 : 0) + ((gamepad1.right_trigger > 0.5) ? -1.0 : 0);
            leftWheel.setPower(leftPower);
            rightWheel.setPower(rightPower);
            if (gamepad1.a)
            {
                if (clawPos < 1)
                {
                    clawPos += INTERVAL;
                }
            }
            if (gamepad1.b)
            {
                if (clawPos > 0)
                {
                    clawPos -= INTERVAL;
                }
            }

            leftClaw.setPosition(Range.clip(clawPos, 0, 0.5));
            rightClaw.setPosition(Range.clip(1 - clawPos, 0.5, 1));

            telemetry.addData("LPosition: ", leftClaw.getPosition());
            telemetry.addData("RPosition: ", rightClaw.getPosition());
            telemetry.update();
        }
    }
}
