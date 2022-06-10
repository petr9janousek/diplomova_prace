#!/usr/bin/env python3
import re
from pypozyx import *
from pypozyx.tools.version_check import perform_latest_version_check
from pypozyx.structures.device_information import DeviceDetails

'''
class UWBtools(object):
    def __init__(self, connect=True) -> None:
        # DONT CONNECT YET, MIGHT BE USED WITH ALREADY CONNECTED CLASS
        self.pozyx = None 
        if connect:
            self.connect_device()
'''

def connect_device(port = None):
    serial_port = port if not port is None else get_first_pozyx_serial_port()
    print(f"Trying port: {serial_port}... ", end="")
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        return None
        
    pozyx = PozyxSerial(serial_port)
    print(f"Pozyx connected.")
    return pozyx

def check_error(pozyx, remote_id=None):
    error_reg = SingleRegister()

    if pozyx.getErrorCode(error_reg, remote_id) == POZYX_SUCCESS:
        message = pozyx.getErrorMessage(error_reg)
        if remote_id == None:
            print(f"CHECK_ERROR:[dev:local]: {message}")
        else:
            print(f"CHECK_ERROR:[dev:{hex(remote_id)}]: {message}")
    else:
        print("Couldn't retrieve error code")

############### UTILS FOR DEBUGGING
def check_device(pozyx, remote_id=None):
    """check_device Check status of pozyx local device (connected)

    Keyword Arguments:
        remote_id -- Perform on remote device (default: {None})
    """
    system_details = DeviceDetails()
    pozyx.getDeviceDetails(system_details, remote_id=remote_id)

    if remote_id is None:
        print("Local %s with id 0x%0.4x" % (system_details.device_name, system_details.id))
    else:
        print("%s with id 0x%0.4x" % (system_details.device_name.capitalize(), system_details.id))

    print("\tWho am i: 0x%0.2x" % system_details.who_am_i)
    print("\tFirmware version: v%s" % system_details.firmware_version_string)
    print("\tHardware version: v%s" % system_details.hardware_version_string)
    print("\tSelftest result: %s" % system_details.selftest_string)
    print("\tError: 0x%0.2x" % system_details.error_code)
    print("\tError message: %s" % system_details.error_message)

def check_network(pozyx, remote_id=None):
    """check_network Discovery proces for all devices and return printed results

    Keyword Arguments:
        remote_id -- Perform on remote device (default: {None})

    TODO: check why found devices is currently empty when using some anchors
    """
    pozyx.clearDevices(remote_id)
    if pozyx.doDiscovery(discovery_type=PozyxConstants.DISCOVERY_ALL_DEVICES, remote_id=remote_id) == POZYX_SUCCESS:
        print("Found devices:")
        print(pozyx.printDeviceList(remote_id, include_coordinates=False))
    else:
        print("Discovery process unsuccessful, check for errors...")

############### UTILS FOR PRINTING STUFF
def printPosition(position, remote_id):
    device = "local" if remote_id is None else str(hex(remote_id))
    print(f"Device: {device}, x(mm): {position.x} y(mm): {position.y} z(mm): {position.z}")
    
def printErrorCode(pozyx, operation, remote_id):
    error_code = SingleRegister()
    if remote_id is None:
        pozyx.getErrorCode(error_code)
        print("LOCAL ERROR %s, %s" % (operation, pozyx.getErrorMessage(error_code)))
        return
    status = pozyx.getErrorCode(error_code, remote_id)
    if status == POZYX_SUCCESS:
        print("ERROR %s on ID %s, %s" %
                (operation, "0x%0.4x" % remote_id, pozyx.getErrorMessage(error_code)))
    else:
        pozyx.getErrorCode(error_code)
        print("ERROR %s, couldn't retrieve remote error code, LOCAL ERROR %s" %
                (operation, pozyx.getErrorMessage(error_code)))
        # should only happen when not being able to communicate with a remote Pozyx.

def printConfigurationResult(pozyx, anchors, remote_id):
    list_size = SingleRegister()

    pozyx.getDeviceListSize(list_size, remote_id)
    print("List size: {0}".format(list_size[0]))
    if list_size[0] != len(anchors):
        printErrorCode("configuration")
        return
    device_list = DeviceList(list_size=list_size[0])
    pozyx.getDeviceIds(device_list, remote_id)
    print("Calibration result:")
    print("Anchors found: {0}".format(list_size[0]))
    print("Anchor IDs: ", device_list)

    for i in range(list_size[0]):
        anchor_coordinates = Coordinates()
        pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, remote_id)
        print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))

def printAnchorConfiguration(anchors):
    for anchor in anchors:
        print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.coordinates)))

def printFilterConfiguration(pozyx, remote_id):
    filt = FilterData()
    pozyx.getPositionFilterData(filt, remote_id)
    device = "local" if remote_id is None else str(hex(remote_id))
    print(f"Filter at device {device} is set as: {filt}")  # "Moving average filter with strength 10"

def set_UWBparams(pozyx, settings, remote_id):
    pass
############### UTIL FOR READING ARGS
def get_parsed_args():
    """get_parsed_args Get input from console

    TODO: umžnit vybrat port, remote, id?
    """
    
    import sys
    global check_pypozyx_version

    print(f"Arguments count: {len(sys.argv)}")
    for i, arg in enumerate(sys.argv):
        print(f"Argument {i:2}: {arg}")
    
    if arg[0] == "-v":
        check_pypozyx_version = True

############### UWB parameters
def get_uwb_setting(pozyx, devices):
    if not None in devices: 
        devices.insert(0, None) #ensure tag is inserted
    
    for dev in devices:
        settings = UWBSettings()
        
        if pozyx.getUWBSettings(settings, remote_id = dev) == POZYX_SUCCESS:
            d = "local" if dev is None else str(hex(dev))
            print(f"Device {d} has settings: {settings}")
        else:
            print(f"Unable to obtain settings from {str(hex(dev))}, check if tag has same settings")


def set_uwb_settings(pozyx, channel=5, bitrate=0, prf=2, plen=0x08, gain_db=11.5, remote_id=None):
    default_settings = UWBSettings(channel=5, bitrate=0, prf=2, plen=0x08, gain_db=11.5)
    status = pozyx.setUWBSettings(default_settings, remote_id) #return to local setting
    if status == POZYX_SUCCESS:
        dev = "local" if remote_id is None else str(hex(remote_id))
        print(f"Succesfully returned device {dev} to default settings")
    else:
        print(f"Error setting UWB settings of device {dev} to default values.")

if __name__ == "__main__":
    perform_latest_version_check()
    pozyx = connect_device()
    check_device(pozyx)
    #check_device(pozyx, 0x6a30)
    #check_network(pozyx)
    set_uwb_settings(pozyx, remote_id = None)