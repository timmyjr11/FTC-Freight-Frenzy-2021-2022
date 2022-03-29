package org.firstinspires.ftc.teamcode.drive;

public class ConfigurationStorage {
    public enum capStonePosition {
        toBeDetermined,
        left,
        center,
        right
    }
    public enum sideStart {
        toBeDetermined,
        leftSide,
        rightSide
    }
    public enum parking {
        toBeDetermined,
        storageUnit,
        warehouse
    }
    public enum warehouseParking {
        toBeDetermined,
        right,
        left,
        top
    }

    public enum intakeMode {
        manual,
        objectDetected
    }

    public enum runOuttake {
        openToRun,
        doNotRunAgain
    }

    public enum liftMode {
        manual,
        runToTop
    }
}
