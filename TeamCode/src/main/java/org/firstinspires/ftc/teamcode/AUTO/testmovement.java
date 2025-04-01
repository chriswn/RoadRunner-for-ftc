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

telemetry .addData(FORWARD);
telemetry .update;
DriveSubsystem. FORWARD (12); 

telemetry .addData(forwardForDistance);
telemetry .update;
DriveSubsystemAuto.forwardForDistance(24, telemetry);

telemetry .addData(done);
telemetry .update;
}



