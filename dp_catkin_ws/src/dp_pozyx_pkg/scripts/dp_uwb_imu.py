#!/usr/bin/env python
from numpy import positive
from pypozyx import (POZYX_SUCCESS,
                     Coordinates, PozyxConstants, PozyxSerial, Quaternion, EulerAngles, SensorData,
                     get_first_pozyx_serial_port)

class IMUsensor(object):
    def __init__(self, pozyx, remote_id=None):
        self.pozyx = pozyx
        self.remote_id = remote_id

    def get_quaterion(self):
        q = Quaternion()
        status = self.pozyx.getQuaternion(q, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            #print("OK")
            return q
        else: 
            print("NOT OK")

    def get_eulerAngles(self):
        ea = EulerAngles()
        status = self.pozyx.getEulerAngles_deg(ea, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            #print("OK")
            return ea
        else: 
            print("NOT OK")

    def get_allData(self):
        sd = SensorData()
        status = self.pozyx.getAllSensorData(sd, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            #print("OK")
            return sd
        else: 
            print("NOT OK")

if __name__ == "__main__":
   
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()
    
    #remote_id = 0x6a5f                 # remote device network ID
    remote_id = None

    pozyx = PozyxSerial(serial_port)

    r = IMUsensor(pozyx, remote_id)
    
    while True:
        try:
            r.get_eulerAngles()
        except KeyboardInterrupt:
            break
