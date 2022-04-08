package org.firstinspires.ftc.teamcode.drive;

// Class used to keep states in both Tele-op and autonomous
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

    public enum rotationForDuck {
        freeToRotate,
        doNotLOl
    }

    public enum goForDuck {
        toBeDetermined,
        doNotGoForDuck,
        goForDuck
    }

    public enum boxState {
        inside,
        halfway,
        outside
    }

    public enum intakeMode {
        manual,
        objectDetected
    }

    public enum intakeSpeed {
        normal,
        slow
    }

    public enum runOuttake {
        openToRun,
        doNotRunAgain
    }
}
