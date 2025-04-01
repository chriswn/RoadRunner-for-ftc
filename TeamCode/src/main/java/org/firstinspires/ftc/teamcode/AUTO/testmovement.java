parkage org.firstinspires.ftc.teamcode.AUTO
import com.firstinspires.hardwareMap;
import com.firstinspires.myOpMode;
@Auto(name= "movetest")
public class testmovement extend LinearOPmode{
DriveSubsystem driveSubsystem;
DriveSubsystemAuto driveSubsystemAuto;

private DcmotorEX 

@overide 
public void runOpMode(){
driveSubsystem = new driveSubsystem (HardwareMap hardwareMap, Telemetry telemetry);
driveSubsystemAuto  = new driveSubsystemAuto (HardwareMap hardwareMap, Telemetry telemetry);
Wait for start();
DriveSubsystem. FORWARD (12); 
DriveSubsystemAuto.forwardForDistance(24, telemetry);

}



