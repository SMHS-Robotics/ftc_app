package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name = "LEFT SIDE", group = "asdofijefj")
public class AutonomousWithSensorsLeft extends LinearOpMode
{
    HardwarePushbot robot = new HardwarePushbot();

    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);

        while (opModeIsActive())
        {
            telemetry.addAction(() -> angles = robot.imu.getAngularOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.DEGREES));

            telemetry.addLine()
                    .addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle))
                    .addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle))
                    .addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));

            telemetry.addLine()
                    .addData("distance",() -> robot.distanceSensor.getDistance(DistanceUnit.CM));
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
