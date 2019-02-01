package org.firstinspires.ftc.teamcode;

import android.print.PageRange;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "LEFT SIDE", group = "asdofijefj")
public class AutonomousWithSensorsLeft extends LinearOpMode
{
    HardwarePushbot robot = new HardwarePushbot();
    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        while(opModeIsActive())
        {

        }
    }

    private void rotate(int degrees, double power)
    {
        power = Range.clip(power, 0, 1);
    }
}
