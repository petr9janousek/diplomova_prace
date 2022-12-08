#!/usr/bin/env python3
from cv2 import KeyPoint_convert
from matplotlib import collections
from numpy import positive
from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_2D, POZYX_SUCCESS, FILTER_TYPE_MOVINGMEDIAN,
                     Coordinates, PozyxConstants, DeviceCoordinates, PozyxSerial, PozyxRegisters, PositionError,
                     get_first_pozyx_serial_port)

from dp_uwb_utils import (printPosition,printErrorCode)
    
remote_id = 0x6a5f                 # remote device network ID
#remote_id = None

anchors = [DeviceCoordinates(0x6a43, 1, Coordinates(0, 0, 1000)),
            DeviceCoordinates(0x6a30, 1, Coordinates(660, 0, 1000)),
            DeviceCoordinates(0x6a78, 1, Coordinates(660, -450, 1000)),
            DeviceCoordinates(0x6a13, 1, Coordinates(0, -450, 1000))]

# positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_TRACKING
algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY

# positioning dimension. Others are PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D
dimension = PozyxConstants.DIMENSION_2D

# height of device, required in 2.5D positioning
height = 1000

class Positioning_Alg1(object):
    def __init__(self, pozyx, anchors=None, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_2D, height=1000, remote_id=None, publish=False):
        self.pozyx = pozyx
        #X HAS TO BE POINTING FORWARD TO COOPERATE WITH RVIZ AND GAZEBO
        self.anchors = [DeviceCoordinates(0x6a78, 1, Coordinates(-450,+330, 1000)), #back left
                        DeviceCoordinates(0x6a30, 1, Coordinates(-450,-330, 1000)), #back right
                        DeviceCoordinates(0x6a43, 1, Coordinates(-4,  -330, 1000)), #front right
                        DeviceCoordinates(0x6a13, 1, Coordinates(-4,  +330, 1000))] #front left

        if anchors is not None: 
            self.anchors = anchors
            
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height

        self.remote_id = remote_id

        self.set_anchors(save_to_flash=False)
        self.set_filter(filter_strength=10)

    def get_position(self):
        """Performs positioning and displays/exports the results."""
        position = Coordinates()
        status = self.pozyx.doPositioning(
            position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            #printPosition(position, self.remote_id)
            position.x = position.x / 1000
            position.y = position.y / 1000
            position.z = position.z / 1000

            return position
        else:
            #if error code is 
            printErrorCode(self.pozyx, "positioning", self.remote_id)

    def set_filter(self, filter_type=FILTER_TYPE_MOVINGMEDIAN, filter_strength=10):
        self.pozyx.setPositionFilter(filter_type, filter_strength, self.remote_id)

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
        return status

if __name__ == "__main__":
   
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    pozyx = PozyxSerial(serial_port)

    r = Positioning_Alg1(pozyx, anchors, algorithm, dimension, height,remote_id)

    while True:
        try:
            xy = r.get_position()
            print(xy)
        except KeyboardInterrupt:
            break
