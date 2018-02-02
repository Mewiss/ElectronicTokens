

from bluepy.btle import Peripheral, ADDR_TYPE_RANDOM, AssignedNumbers

import time

class HRM(Peripheral):
    def __init__(self, addr):
        Peripheral.__init__(self, addr, addrType=ADDR_TYPE_RANDOM)

if __name__=="__main__":
    cccid = AssignedNumbers.client_characteristic_configuration
    
#Services UUIDs
    press_id = 0x280D#AssignedNumbers.heart_rate
    ori_id = 0x280E
    accel_id = 0x280F
    linaccel_id = 0x280A

#Characteristics UUIDs
    sensor1_id = AssignedNumbers.heart_rate_measurement
    orisensor_id = '00002e3700001000800000805f9b34fb'
    accelsensor_id = '00002f3700001000800000805f9b34fb'
    linaccelsensor_id = '00002a3700001000800000805f9b34fb'
	
    hrm = None
    try:
        hrm = HRM('DC:2A:B2:0B:FD:78')#HRM('E8:5D:F2:F1:9F:17')#

#	print hrmmid 
        service, = [s for s in hrm.getServices() if s.uuid==press_id]
        ccc, = service.getCharacteristics(forUUID=str(sensor1_id))

        service2, = [s for s in hrm.getServices() if s.uuid==ori_id]
        ccc2, = service2.getCharacteristics(forUUID=orisensor_id)

        service3, = [s for s in hrm.getServices() if s.uuid==accel_id]
        ccc3, = service3.getCharacteristics(forUUID=accelsensor_id)

        service4, = [s for s in hrm.getServices() if s.uuid==linaccel_id]
        ccc4, = service4.getCharacteristics(forUUID=linaccelsensor_id)

        if 0: # This doesn't work
            ccc.write('\1\0')
            ccc2.write('\1\0')
            ccc3.write('\1\0')
            ccc4.write('\1\0')

        else:
            desc = hrm.getDescriptors(service.hndStart,
                                      service.hndEnd)
            d, = [d for d in desc if d.uuid==cccid]

            hrm.writeCharacteristic(d.handle, '\1\0')

            print d.handle

            desc2 = hrm.getDescriptors(service2.hndStart,
                                      service2.hndEnd)
	
            d2, = [d2 for d2 in desc2 if d2.uuid==cccid]

            hrm.writeCharacteristic(d2.handle, '\1\0')

            print d2.handle

            desc3 = hrm.getDescriptors(service3.hndStart,
                                      service3.hndEnd)
	
            d3, = [d3 for d3 in desc3 if d3.uuid==cccid]

            hrm.writeCharacteristic(d3.handle, '\1\0')

            print d3.handle

            desc4 = hrm.getDescriptors(service3.hndStart,
                                      service3.hndEnd)
	
            d4, = [d4 for d4 in desc4 if d4.uuid==cccid]

            hrm.writeCharacteristic(d4.handle, '\1\0')

            print d4.handle


        t0=time.time()
        def print_hr(cHandle, data):
          #  val0 = ord(data[0])
          #  val1 = ord(data[1])
          #  val2 = ord(data[2])
          #  print cHandle, data,"%.2f"%(time.time()-t0)
		print cHandle, data.split('/')
         
	hrm.delegate.handleNotification = print_hr

        for x in range(100):
            hrm.waitForNotifications(3.)

    finally:
        if hrm:
            hrm.disconnect()
