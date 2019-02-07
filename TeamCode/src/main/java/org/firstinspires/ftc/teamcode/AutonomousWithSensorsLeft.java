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
    AutonomousState state;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException
    {
        autonomousStart();

        waitForStart();
        state = AutonomousState.TO_WALL;

        while (opModeIsActive())
        {
            switch (state)
            {
                case TO_WALL:
                    autonomousToWall();
                    break;
                case TO_DEPOT:
                    autonomousToDepot();
                    break;
                case DROP_MARKER:
                    autonomousDropMarker();
                    break;
                case TO_CRATER:
                    autonomousToCrater();
                    break;
                case END:
                    break;
                default:
                    throw new InterruptedException("invalid state");
            }

            telemetry.addAction(() -> angles = robot.imu.getAngularOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.DEGREES));

            telemetry.addLine()
                    .addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle))
                    .addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle))
                    .addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));

            telemetry.addLine()
                    .addData("distance", () -> robot.distanceSensor.getDistance(DistanceUnit.CM));
        }
    }

    private void autonomousStart()
    {
        robot.init(hardwareMap);
    }

    private void autonomousToWall()
    {

    }

    private void autonomousToDepot()
    {

    }

    private void autonomousDropMarker()
    {

    }

    private void autonomousToCrater()
    {

    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /*
        Represents the current "stage" of the autonomous program
        START: Robot has just been placed. Not moving.
        TO_WALL: Robot is driving directly perpendicular to wall towards wall
        TO_DEPOT: Robot is driving to depot
        DROP_MARKER: Robot is dropping marker in depot
        TO_CRATER: Robot is driving to crater to park
        END: Robot has parked. Not moving
     */
    public enum AutonomousState
    {
        START, TO_WALL, TO_DEPOT, DROP_MARKER, TO_CRATER, END;
    }
}
