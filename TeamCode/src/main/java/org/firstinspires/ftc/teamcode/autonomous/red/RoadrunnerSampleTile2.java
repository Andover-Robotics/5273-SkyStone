package org.firstinspires.ftc.teamcode.autonomous.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.RobotAlliance;
import org.firstinspires.ftc.teamcode.autonomous.base_classes.RoadrunnerSampleTile2Base;

@Autonomous(name = "RR Red Sample (Tile 2)", group = "AAR")
public class RoadrunnerSampleTile2 extends RoadrunnerSampleTile2Base {
    @Override
    protected RobotAlliance getCurrentAlliance() {
        return RobotAlliance.RED;
    }
}
