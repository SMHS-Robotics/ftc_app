package org.firstinspires.ftc.teamcode;

import android.print.PageRange;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "LEFT SIDE", group = "asdofijefj")
public class AutonomousWithSensorsLeft extends LinearOpMode
{
    HardwarePushbot robot = new HardwarePushbot();
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        time.startTime();
        while (opModeIsActive())
        {
            if (time.seconds() >= 5)
            {
                time.reset();
                telemetry.addData("Angle since 5 seconds: ", robot.getAngle());
                telemetry.update();
            }
        }
    }

    private void rotate(int degrees, double power)
    {
        power = Range.clip(power, 0, 1);
        robot.resetAngle();
        double leftPower, rightPower;

        //set the initial power of the motors depending on rotation
        if(degrees < 0){
            leftPower = -power;
            rightPower = power;
        }else if(degrees > 0){
            leftPower = power;
            rightPower = -power;
        } else return;

        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        //rotate within 3 degrees of error before stopping, and adjust as necessary
        while (robot.getAngle() > degrees + 3 && robot.getAngle() < degrees - 3) {
            if(robot.getAngle() > degrees + 3 && leftPower > 0){
                leftPower = -power;
                rightPower = power;
                robot.leftDrive.setPower(leftPower);
                robot.rightDrive.setPower(rightPower);
            }
            if(robot.getAngle() < degrees - 3 && leftPower < 0){
                leftPower = power;
                rightPower = -power;
                robot.leftDrive.setPower(leftPower);
                robot.rightDrive.setPower(rightPower);
            }
        }

        //stop the robot
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        sleep(1000);

        robot.resetAngle();
    }
}
