#!/usr/bin/env python3

from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_POS_ALG_TRACKING,               # Settings
                     POZYX_2D, POZYX_2_5D, POZYX_3D,                               # Settings
                     FILTER_TYPE_MOVINGMEDIAN, FILTER_TYPE_FIR, FILTER_TYPE_NONE,  # Settings
                     POZYX_SUCCESS, POZYX_ERROR_GENERAL,                           # Return codes
                     Coordinates, PozyxConstants, DeviceCoordinates,               # Classes
                     PozyxSerial, PozyxRegisters, PositionError,                   # Classes
                     Quaternion, EulerAngles, SensorData,                          # Classes
                     get_first_pozyx_serial_port)                                  # Functions

from dp_uwb_utils import (connect_device, printPosition, printErrorCode) #needs refactoring

class PozyxMeasure():
    # remote device network ID
    remote_id = 0x6a2c

    # height of device, required in 2.5D positioning
    height = 1000

    # anchor positions measured 
    # X HAS TO BE POINTING FORWARD TO COOPERATE WITH RVIZ AND GAZEBO
    anchors = [DeviceCoordinates(0x6a78, 1, Coordinates(-450,+330, 1000)), #back left
                DeviceCoordinates(0x6a30, 1, Coordinates(-450,-330, 1000)), #back right
                DeviceCoordinates(0x6a43, 1, Coordinates(-4,  -330, 1000)), #front right
                DeviceCoordinates(0x6a13, 1, Coordinates(-4,  +330, 1000))] #front left
    
    # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_TRACKING
    algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
    
    # positioning dimension. Others are PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D
    dimension = PozyxConstants.DIMENSION_2D

    def __init__(self, remote_id=None):
        # Specify a pozyx device to run on
        self.pozyx = connect_device()
        assert self.pozyx is not None, f"Unable to connect built-in Pozyx device ID:{remote_id}, check connection."

        self.anchors = [DeviceCoordinates(0x6a78, 1, Coordinates(-450,+330, 1000)), #back left
                        DeviceCoordinates(0x6a30, 1, Coordinates(-450,-330, 1000)), #back right
                        DeviceCoordinates(0x6a43, 1, Coordinates(-4,  -330, 1000)), #front right
                        DeviceCoordinates(0x6a13, 1, Coordinates(-4,  +330, 1000))] #front left

        if remote_id is not None: 
            self.remote_id = remote_id
        
        # these are also asserted without proper init the node (or robot) will not work thus are mandatory
        self.set_anchors(save_to_flash=False)
        self.set_filter(filter_strength=10)

    def get_position(self):
        """Performs positioning and displays/exports the results."""
        position = Coordinates()
        status = self.pozyx.doPositioning(position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)
        
        if status == POZYX_SUCCESS:
            #printPosition(position, self.remote_id)
            position.x = position.x / 1000
            position.y = position.y / 1000
            position.z = position.z / 1000

            return position
        else:
            printErrorCode(self.pozyx, "getting position", self.remote_id)

    def get_quaterion(self):
        quaternion = Quaternion()
        status = self.pozyx.getQuaternion(quaternion, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            return quaternion
        else: 
            printErrorCode(self.pozyx, "getting orientation", self.remote_id)


    def get_eulerAngles(self):
        angles = EulerAngles()
        status = self.pozyx.getEulerAngles_deg(angles, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            return angles
        else: 
            printErrorCode(self.pozyx, "getting angles", self.remote_id)

    def get_allData(self):
        data = SensorData()
        status = self.pozyx.getAllSensorData(data, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            return data
        else: 
            printErrorCode(self.pozyx, "getting data", self.remote_id)

    def set_filter(self, filter_type=FILTER_TYPE_MOVINGMEDIAN, filter_strength=10):
        status = self.pozyx.setPositionFilter(filter_type, filter_strength, self.remote_id)
        assert status == POZYX_SUCCESS, f"Unable to set filter as {filter_type}, with strength {filter_strength}" 

    def set_anchors(self, save_to_flash=False):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(remote_id=self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, remote_id=self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors),
                                                       remote_id=self.remote_id)
        if save_to_flash:
            self.pozyx.saveAnchorIds(remote_id=self.remote_id)
            self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], remote_id=self.remote_id)
        assert status == POZYX_SUCCESS, f"Unable to save anchors with remote_id {hex(self.remote_id)}" 

if __name__ == "__main__":
    pm = PozyxMeasure()

    while True:
        try:
            #print(pm.get_position())
            #print(pm.get_quaterion())
            #print(pm.get_eulerAngles())
            print(pm.get_allData())
        except KeyboardInterrupt:
            break
