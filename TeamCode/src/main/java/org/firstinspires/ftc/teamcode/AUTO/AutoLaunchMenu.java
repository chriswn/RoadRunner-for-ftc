package org.firstinspires.ftc.teamcode.AUTO;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(preselectTeleOp = "Auto")
public class AutoLaunchMenu extends LinearOpMode {

    Boolean autoLaunched = null;
    Boolean allianceRed = null;
    Boolean startLeft = null;
    Boolean pathDirect = null;
    Boolean parkingLeft = null;
    Boolean initialized = null;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean previousX = false;
        boolean previousB = false;

        while (initialized) {

            boolean currentX = gamepad1.x;
            boolean currentB = gamepad1.b;

            if (allianceRed == null) {
                telemetry.addData("Alliance", "X=blue, B=Red");
                telemetry.update();
                if (currentX && !previousX) {
                    allianceRed = false;
                }
                if (currentB && !previousB) {
                    allianceRed = true;
                }

            } 
            
            else if (startLeft == null) {
                telemetry.addData("Start", "X=left,B=right");
                telemetry.update();
                if (currentX && !previousX) {
                    startLeft = false;
                }
                if (currentB && !previousB) {
                    startLeft = true;
                }
            } 
            
            else if (pathDirect == null) {
                telemetry.addData("Path", "X=direct,B=reverse");
                telemetry.update();
                if (currentX && !previousX) {
                    pathDirect = false;
                }
                if (currentB && !previousB) {
                    pathDirect = true;
                }
            }
            else if (parkingLeft == null) {
                telemetry.addData("Parking", "X=left,B=right");
                telemetry.update();
                if (currentX && !previousX) {
                    parkingLeft = true;
                }
                if (currentB && !previousB) {
                    parkingLeft =false ;
                }
            }
            else {
                initialized = true;
            }

            previousX = currentX;
            previousB = currentB;
        }

        telemetry.addData("Auto Launched", autoLaunched);
        telemetry.addData("Alliance", allianceRed? "Red" : "Blue" );
        telemetry.addData("Start", startLeft ? "Left" : "Right");
        telemetry.addData("Path", pathDirect ? "Direct" : "Reverse" );
        telemetry.addData("Parking", parkingLeft ? "Left" : "Right" );
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData( "status", "Running" );
            telemetry.update();
        }
    }
}