package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RIGHT SID", group = "asdofijefj")

public class AutoWithSensorsRightPushbot extends AutoOpModePushbot
{
    final double ANGLE_TO_DEPOT = -58.3469043;
    final double ANGLE_TO_CRATER = -190.2361801;

    @Override
    public void runOpMode() throws InterruptedException
    {
        autonomousStart();

        waitForStart();
        state = AutonomousState.TO_WALL;

        composeTelemetry();

        while (opModeIsActive())
        {
            /*
            Controls the current stage of the autonomous program based on an enum
             */
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
                    robot.leftDrive.setPower(0);
                    robot.rightDrive.setPower(0);
                    break;
                default:
                    throw new InterruptedException("invalid state");
            }

            telemetry.update();
        }
    }

    private void autonomousStart()
    {
        robot.init(hardwareMap);
    }

    private void autonomousToWall()
    {
        driveUntilImpact(DRIVE_POWER);
        state = AutonomousState.TO_DEPOT;
    }

    private void autonomousToDepot()
    {
        //turn towards depot
        rotate(ANGLE_TO_DEPOT, TURN_POWER);

        //go forward until hit wall
        driveUntilImpact(DRIVE_POWER);
        state = AutonomousState.DROP_MARKER;

    }

    private void autonomousDropMarker()
    {
        //ARBITRARY VALUE HAHA
        rotate(ANGLE_TO_CRATER, TURN_POWER);
        robot.flagDrop.setPosition(0);
        sleep(100);
        robot.flagDrop.setPosition(1);
        state = AutonomousState.TO_CRATER;
    }

    private void autonomousToCrater()
    {
        driveUntilImpact(DRIVE_POWER);
        state = AutonomousState.END;
    }
}
