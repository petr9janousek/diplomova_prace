#!/usr/bin/env python3

from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_POS_ALG_TRACKING,               # Settings
                     POZYX_2D, POZYX_2_5D, POZYX_3D,                               # Settings
                     FILTER_TYPE_MOVINGMEDIAN, FILTER_TYPE_FIR, FILTER_TYPE_NONE,  # Settings
                     POZYX_SUCCESS, POZYX_ERROR_GENERAL,                           # Return codes
                     Coordinates, PozyxConstants, DeviceCoordinates,               # Classes
                     PozyxSerial, PozyxRegisters, PositionError,                   # Classes
                     Quaternion, EulerAngles, SensorData, Acceleration,
                     AngularVelocity, LinearAcceleration, UWBSettings,             # Classes
                     get_first_pozyx_serial_port,                                  # Functions
                     SingleRegister, DeviceList)

from dp_uwb_utils import (set_anchors, set_filter,
                          set_uwb_setting, get_uwb_setting, set_uwb_default_settings,
                          printPosition, printErrorCode) #needs refactoring

class PozyxMeasure():
    # remote device network ID
    remote_id = 0x6a5f

    # anchor positions measured 
    # X HAS TO BE POINTING FORWARD TO COOPERATE WITH RVIZ AND GAZEBO
    anchors = [DeviceCoordinates(0x6a78, 1, Coordinates(-450,+330, 400)), #back left
                DeviceCoordinates(0x6a30, 1, Coordinates(-450,-330, 400)), #back right
                DeviceCoordinates(0x6a43, 1, Coordinates(-4,  -330, 400)), #front right
                DeviceCoordinates(0x6a13, 1, Coordinates(-4,  +330, 400))] #front left
    
    # positioning dimension. Others are PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D
    dimension = PozyxConstants.DIMENSION_2_5D #anchors and tag can have different heights

    # height of device, required in 2.5D positioning
    height = 1000

    # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_TRACKING
    algorithm = PozyxConstants.POSITIONING_ALGORITHM_TRACKING #probably using kalman filter
    
    filter = FILTER_TYPE_MOVINGMEDIAN
    filter_length = 8

    def __init__(self, remote_id=None):
        # Specify a pozyx device to run on, there should only be one
        serial_port = get_first_pozyx_serial_port()
        assert serial_port is not None, "No Pozyx device connected. Check your USB cable or your driver!"

        self.pozyx = PozyxSerial(serial_port)

        if remote_id is not None: 
            self.remote_id = remote_id
        
        # these are also asserted without proper init the node (or robot) will not work thus are mandatory
        set_anchors(self.pozyx, self.remote_id, self.anchors, save_to_flash = False)
        set_filter(self.pozyx, self.remote_id, filter_type = self.filter, filter_strength = self.filter_length)
        print("READ SETTINGS:")
        get_uwb_setting(self.pozyx,[0x6a5f,0x6a43,0x6a78,0x6a30])
        
        settings = UWBSettings(5,1,2,0x34,11.5) # higher bitrate and lower PLEN 512
        print("SET SETTINGS:")
        set_uwb_setting(self.pozyx, [0x6a5f,0x6a43,0x6a78,0x6a30, None], settings) # set all the devices and than self
        self.pozyx.setRangingProtocol(PozyxConstants.RANGE_PROTOCOL_FAST, self.remote_id)  

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

    def get_quaterion(self, normalized=False):
        quaternion = Quaternion()
        if normalized:
            status = self.pozyx.getQuaternion(quaternion, remote_id=self.remote_id)
        else:
            status = self.pozyx.getNormalizedQuaternion(quaternion, remote_id=self.remote_id)
        
        if status == POZYX_SUCCESS:
            return quaternion
        else: 
            printErrorCode(self.pozyx, "getting orientation", self.remote_id)

    def get_acceleration(self, linear=False):
        if linear:
            result = Acceleration()
            status = self.pozyx.getAcceleration_mg(result,self.remote_id)
        else:
            result = LinearAcceleration()
            status = self.pozyx.getLinearAcceleration_mg(result,self.remote_id)
            
        if status == POZYX_SUCCESS:
            return result
        else: 
            printErrorCode(self.pozyx, "getting orientation", self.remote_id)

    def get_angularVelocity(self, linear=False):
        velocity = AngularVelocity()
        status = self.pozyx.getAngularVelocity_dps(velocity,self.remote_id)
        if status == POZYX_SUCCESS:
            return velocity
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

if __name__ == "__main__":
    pm = PozyxMeasure()
    while True:
        try:
            pos = pm.get_position()
            #print(pm.get_quaterion())
            #print(pm.get_eulerAngles())
            #print(pm.get_allData())
        except KeyboardInterrupt:
            break
