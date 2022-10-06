class Gripper(object):
    def __init__(self, rapid,inputs,outputs):
        self.rapid = rapid
        self.inputs = inputs
        self.outputs = outputs

    def open(self):
        pin = self.outputs['gripper1']
        self.rapid.set_digital_io(pin, 1)
        pin = self.outputs['gripper2'] 
        self.rapid.set_digital_io(pin, 0) 

    def isOpen(self):
        pin = self.inputs['gripperSensorOpen']
        SensorOpenState = bool(self.rapid.get_digital_io(pin))
        pin = self.inputs['gripperSensorClosed']
        SensorClosedState = bool(self.rapid.get_digital_io(pin))

        if (SensorOpenState) & (not SensorClosedState):
            return True
        else:
            return False

    def close(self):
        pin = self.outputs['gripper1']
        self.rapid.set_digital_io(pin, 0)
        pin = self.outputs['gripper2'] 
        self.rapid.set_digital_io(pin, 1)

    def isClosed(self):
        pin = self.inputs['gripperSensorOpen']
        SensorOpenState = bool(self.rapid.get_digital_io(pin))
        pin = self.inputs['gripperSensorClosed']
        SensorClosedState = bool(self.rapid.get_digital_io(pin))

        if (not SensorOpenState) & (SensorClosedState):
            return True
        else:
            return False

        