package org.firstinspires.ftc.teamcode.drive.opmode;

//Was a great idea to remove all the ones and zeros for the configuration, but then I realized there
//is no point to doing this because everything already works and this would be a waste of my time, oop
public class ConfigurationStorage {
    public enum capStonePosition {
        toBeDetermined,
        left,
        center,
        right
    }
    public enum sideStart {
        leftSide,
        rightSide
    }
    public enum parking {
        storageUnit,
        warehouse
    }
    public enum warehouseParking {
        right,
        Left,
        top
    }
}
