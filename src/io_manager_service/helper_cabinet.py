class Cabinet(object):
    def __init__(self, rapid,inputs,outputs):
        self.rapid = rapid
        self.inputs = inputs
        self.outputs = outputs

    def openRoboDoor(self):
        pin = self.outputs['roboDoor1']
        self.rapid.set_digital_io(pin, 1)
        pin = self.outputs['roboDoor2'] 
        self.rapid.set_digital_io(pin, 0) 

    def isRoboDoorOpen(self):
        pin = self.inputs['roboDoorSensorUp']
        SensorUpState = bool(self.rapid.get_digital_io(pin))
        pin = self.inputs['roboDoorSensorDown']
        SensorDownState = bool(self.rapid.get_digital_io(pin))

        if (SensorUpState) & (not SensorDownState):
            return True
        else:
            return False

    def closeRoboDoor(self):
        pin = self.outputs['roboDoor1']
        self.rapid.set_digital_io(pin, 0)
        pin = self.outputs['roboDoor2'] 
        self.rapid.set_digital_io(pin, 1)

    def isRoboDoorClosed(self):
        pin = self.inputs['roboDoorSensorUp']
        SensorUpState = bool(self.rapid.get_digital_io(pin))
        pin = self.inputs['roboDoorSensorDown']
        SensorDownState = bool(self.rapid.get_digital_io(pin))

        if (not SensorUpState) & (SensorDownState):
            return True
        else:
            return False

    def isOpDoorClosed(self):
        pin = self.inputs['opDoorSensor']
        return bool(self.rapid.get_digital_io(pin))
        