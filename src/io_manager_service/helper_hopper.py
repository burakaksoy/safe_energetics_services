class Hopper(object):
    def __init__(self, rapid,inputs,outputs):
        self.rapid = rapid
        self.inputs = inputs
        self.outputs = outputs

    def openLid(self):
        pin = self.outputs['hopperLid1']
        self.rapid.set_digital_io(pin, 1)
        pin = self.outputs['hopperLid2'] 
        self.rapid.set_digital_io(pin, 0)

    def isLidOpen(self):
        pin = self.inputs['hopperLidSensorUp']
        SensorUpState = bool(self.rapid.get_digital_io(pin))
        pin = self.inputs['hopperLidSensorDown']
        SensorDownState = bool(self.rapid.get_digital_io(pin))

        if (not SensorUpState) & (not SensorDownState):
            return True
        else:
            return False

    def closeLid(self):
        pin = self.outputs['hopperLid1']
        self.rapid.set_digital_io(pin, 0)
        pin = self.outputs['hopperLid2'] 
        self.rapid.set_digital_io(pin, 1)

    def isLidClosed(self):
        pin = self.inputs['hopperLidSensorUp']
        SensorUpState = bool(self.rapid.get_digital_io(pin))
        pin = self.inputs['hopperLidSensorDown']
        SensorDownState = bool(self.rapid.get_digital_io(pin))

        if (SensorUpState) & (SensorDownState):
            return True
        else:
            return False

    def isPressureGood(self):
        pin = self.inputs['hopperPressureSensor']
        return bool(self.rapid.get_digital_io(pin))

        