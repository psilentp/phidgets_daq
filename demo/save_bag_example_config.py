class Config:
    def __init__(self):
        self.basename = 'led_and_odor_control'
        self.directory = '~/orchard/data'
        self.topics = ['/phidgets_daq/digital_output', '/ledpanels/command']
