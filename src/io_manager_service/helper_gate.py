class Gate(object):
    def __init__(self, rapid,inputs,outputs):
        self.rapid = rapid
        self.inputs = inputs
        self.outputs = outputs

        self.lock_state = False

    def lock(self):
        pin = self.outputs['gate']
        self.rapid.set_digital_io(pin, 1)
        
        self.lock_state = True

    def isLocked(self):
        return self.lock_state

    def unlock(self):
        pin = self.outputs['gate']
        self.rapid.set_digital_io(pin, 0)

        self.lock_state = False

    def isClosed(self):
        pin = self.inputs['gateSensor']
        return bool(self.rapid.get_digital_io(pin))

        