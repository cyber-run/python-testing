class PID(object):
    def __init__(self, pGain, iGain, dGain):
        self.pGain = pGain
        self.iGain = iGain
        self.dGain = dGain
        self.prevError = 0
        self.iError = 0
        

    # output        - control value
    # error         - difference between wanted value and actual value
    def update(self, error):
        dError = error - self.prevError
        self.iError = self.iError + error
        control = self.pGain*error  \
                + self.dGain*dError \
                + self.iGain*self.iError
        self.prevError = error
        return control        