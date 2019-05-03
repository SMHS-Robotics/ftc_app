package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.HardwareDummybot;
import org.firstinspires.ftc.teamcode.utilities.VuforiaPicture;

@TeleOp(name = "CummyBoy", group = "Dummybot")

public class WannaSmashBro extends LinearOpMode
{

    HardwareDummybot robot = new HardwareDummybot();

    @Override
    public void runOpMode()
    {
        double left;
        double right;
        double arm;

        robot.init(hardwareMap, VuforiaPicture.FOOD_BANK);

        waitForStart();

        while (opModeIsActive())
        {

            //y being pressed power is 1, a being pressed power is -1, both cancels
            arm = (gamepad1.y ? 1 : 0) - (gamepad1.a ? 1 : 0);
            left = Range.clip(-gamepad1.left_stick_y, -1, 1);
            right = Range.clip(-gamepad1.right_stick_y, -1, 1);

            telemetry.addData("left", left);
            telemetry.addData("right", right);
            telemetry.addData("arm", arm);
            telemetry.update();

            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);
            robot.arm.setPower(arm);

            sleep(25);
        }

    }

}
