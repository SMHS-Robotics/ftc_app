package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.HardwareDummybot;

@TeleOp(name = "CummyBoy", group = "Pushbot")

public class WannaSmashBro extends LinearOpMode
{

    HardwareDummybot robot = new HardwareDummybot();

    @Override
    public void runOpMode()
    {
        double left;
        double right;

        boolean forward = true;

        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            if (gamepad1.right_bumper)
            {
                forward = !forward;
            }

            left = Range.clip(gamepad1.left_trigger, 0, 1);
            right = Range.clip(gamepad1.right_trigger, 0, 1);

            left = forward ? Math.abs(left) : -Math.abs(left);
            right = forward ? Math.abs(right) : -Math.abs(right);

            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);

            sleep(25);
        }

    }

}
