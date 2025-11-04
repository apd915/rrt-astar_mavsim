"""
msg_delta
    - messages type for input to the aircraft
    
part of mavsim
    - Beard & McLain, PUP, 2012
    - Last update:
        2/27/2020 - RWB
        4/6/2022 - RWB
"""
import numpy as np


class MsgDelta:
    '''
        Message class for control commands to the aircraft
    '''
    def __init__(self,
                 elevator=0.0,
                 aileron=0.0,
                 rudder=0.0,
                 throttle=0.5,
                 azimuth_cmd=0.0,
                 elevation_cmd=0.0):
        self.elevator = float(elevator)  # elevator command
        self.aileron = float(aileron)  # aileron command
        self.rudder = float(rudder)  # rudder command
        self.throttle = float(throttle)  # throttle command
        self.gimbal_az = float(azimuth_cmd)  # azimuth command for gimbal
        self.gimbal_el = float(elevation_cmd)  # elevation command for gimbal

    def to_array(self)->np.ndarray:
        return np.array([[self.elevator],
                         [self.aileron],
                         [self.rudder],
                         [self.throttle],
                         [self.gimbal_az],
                         [self.gimbal_el],
                         ])

    def from_array(self, u:np.ndarray):
        self.elevator = u.item(0)
        self.aileron = u.item(1)
        self.rudder = u.item(2)
        self.throttle = u.item(3)
        self.gimbal_az = u.item(4)
        self.gimbal_el = u.item(5)

    def print(self):
        print('elevator=', self.elevator,
              'aileron=', self.aileron,
              'rudder=', self.rudder,
              'throttle=', self.throttle,
              'azimuth_cmd=', self.gimbal_az,
              'elevation_cmd=', self.gimbal_el)


