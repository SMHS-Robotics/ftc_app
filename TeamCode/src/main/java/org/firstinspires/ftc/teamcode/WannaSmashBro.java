package org.firstinspires.ftc.teamcode;

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

        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            left = Range.clip(gamepad1.left_stick_y, -1, 1);
            right = Range.clip(gamepad1.right_stick_y, -1, 1);

            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);

            sleep(25);
        }
    }

}
