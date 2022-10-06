class Emergency(object):
    def __init__(self, rapid,inputs,outputs):
        self.rapid = rapid
        self.inputs = inputs
        self.outputs = outputs

        self.emergency_state = False

    def isEmergency(self):
        # Check the value of input pin from rapid
        pin = self.inputs['emergency']
        self.emergency_state = bool(self.rapid.get_digital_io(pin))
        return self.emergency_state
        