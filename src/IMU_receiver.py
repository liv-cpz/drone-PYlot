# Imports
import struct
from bluepy.btle import UUID, Peripheral, Scanner, DefaultDelegate

# List of characteristis UUIDs with IMU data
IMU_UUID = ["082b9438-e83c-11e8-9f32-f2801f1b9fd1",
            "082b9622-e83c-11e8-9f32-f2801f1b9fd1", 
            "082b976c-e83c-11e8-9f32-f2801f1b9fd1", 
            "082b9439-e83c-11e8-9f32-f2801f1b9fd1", 
            "082b9623-e83c-11e8-9f32-f2801f1b9fd1",
            "082b976d-e83c-11e8-9f32-f2801f1b9fd1"]

class MyDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleNotification(self, cHandle, data):
        match cHandle:
            case 12:
                handle = "Accel x"
            case 15:
                handle = "Accel y"
            case 18:
                handle = "Accel z"
            case 21:
                handle = "Gyro x"
            case 24:
                handle = "Gyro y"
            case 27:
                handle = "Gyro z"
            case _:
                handle = "UNKNOWN"
        print (handle, ": ", struct.unpack('f', data)[0])

def main():
    # Scan for devices
    scanner = Scanner()
    devices = scanner.scan(3.0)

    # Find the device named "BLE_IMU"
    for dev in devices:
        for (adtype, desc, value) in dev.getScanData():
            if value == "BLE_IMU":
                p = Peripheral(dev.addr, "public")

    # Enable notifications
    p.setDelegate(MyDelegate())
    charList = []
    for x in IMU_UUID:
        charList.extend(p.getCharacteristics(uuid=x))
        cccd = charList[-1].getHandle() + 1
        p.writeCharacteristic(cccd, b'\x01\x00', withResponse=True)

    # Main loop
    while True:
        # Wait for a notification
        if p.waitForNotifications(1.0):
            continue

        print("Waited more than one sec for notification")

if __name__ == "__main__":
    main()