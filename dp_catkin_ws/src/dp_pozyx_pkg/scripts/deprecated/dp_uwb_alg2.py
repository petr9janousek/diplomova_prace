#!/usr/bin/env python
from numpy import positive
from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_2D, POZYX_SUCCESS, FILTER_TYPE_MOVINGMEDIAN,
                     Coordinates, PozyxConstants, DeviceCoordinates, PozyxSerial, PozyxRegisters, DeviceRange,
                     get_first_pozyx_serial_port)

from dp_uwb_utils import (printPosition,printErrorCode)
import numpy as np

class Positioning_Alg2(object):
    def __init__(self, pozyx, anchors=None, remote_id=None):
        self.pozyx = pozyx

        self.anchors = [DeviceCoordinates(0x6a43, 1, Coordinates(0, 0, 1000)),
                        DeviceCoordinates(0x6a30, 1, Coordinates(660, 0, 1000)),
                        DeviceCoordinates(0x6a78, 1, Coordinates(660, -450, 1000)),
                        DeviceCoordinates(0x6a13, 1, Coordinates(0, -450, 1000))]
        
        if anchors is not None: 
            self.anchors = anchors
        
        self.errors = [[np.inf,-np.inf],[np.inf,-np.inf],[np.inf,-np.inf],[np.inf,-np.inf]]
        self.remote_id = remote_id
        self.set_anchors(save_to_flash=False)
        self.set_filter(filter_strength=6)

    def get_errors(self):
        for i, destination in enumerate(self.anchors):
            range = DeviceRange()
            status = self.pozyx.doRanging(destination.network_id, range, self.remote_id)
            if status == POZYX_SUCCESS:
                self.errors[i][0] = min(range.distance, self.errors[i][0])
                self.errors[i][1] = max(range.distance, self.errors[i][1])
            
            print(str(self.errors[i][0]) + " " + str(self.errors[i][1]) + " " + str(self.errors[i][1] - self.errors[i][0]))

        print("--------------------")

    def get_position(self):
        """Performs positioning and displays/exports the results."""
        measurements = []
        for destination in self.anchors:
            range = DeviceRange()
            status = self.pozyx.doRanging(destination.network_id, range, self.remote_id)
            if status == POZYX_SUCCESS:
                range.timestamp = destination.network_id
                measurements.append(range)
            else:
                #if error code is 
                printErrorCode(self.pozyx, "positioning", destination.network_id)

        measurements.sort(key=lambda x: x.RSS, reverse=True) 
        measurements = measurements[0:3]
        for x in measurements: print(x)

        print("--------------------")

    def set_filter(self, filter_type=FILTER_TYPE_MOVINGMEDIAN, filter_strength=5):
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
    
    #remote_id = 0x6a5f                 # remote device network ID
    remote_id = None

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

    pozyx = PozyxSerial(serial_port)

    r = Positioning_Alg2(pozyx, anchors, remote_id)
    
    while True:
        try:
            r.get_errors()
        except KeyboardInterrupt:
            break
