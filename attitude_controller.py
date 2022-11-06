#!/usr/bin/env python3
class attitude_controller():

    '''
    The system used to control the attitutde of the drone 
    - TODO: Add publisher and subscriber lines
    
    '''

    def __init__(self):
        super().__init__()
        
        # inputs
        # self.roll = 0
        # self.pitch = 0
        # self.yaw = 0

        # output [prop1, prop2, prop3, prop4]
        self.pwm = [0,0,0,0]
        
        # errors
        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0 ,0.0]
        self.integral_error = [0,0,0]
        self.derivative_error = [0,0,0]
        
        # PID constants [roll, pitch, yaw]
        self.Kp = [0,0,0]
        self.Ki = [0,0,0]
        self.Kd = [0,0,0]

        # IMU values for feedback 
        self.drone_orientation = [0.0, 0.0, 0.0]

        # IMU values we need
        self.setpoint = [0.0, 0.0, 0.0]

        # sampling rate or sampling time in seconds
        self.sample_time = 0.160

        # Add ROS publishers and subscribers
    

    def imu_callback(self):
        self.drone_orientation[0] = self.setpoint[0]
        self.drone_orientation[1] = self.setpoint[1]
        self.drone_orientation[2] = self.setpoint[2]


    # def drone_command_callback(self):
    #     self.setpoint[0] = roll
    #     self.setpoint[1] = pitch
    #     self.setpoint[2] = yaw


    # Define callback function to tune roll, pitch, yaw
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 0.3


    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06
        self.Ki[1] = pitch.Ki * 0.008
        self.Kd[1] = pitch.Kd * 0.3


    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp * 0.06
        self.Ki[2] = yaw.Ki * 0.008
        self.Kd[2] = yaw.Kd * 0.3


    def pid(self):

        # P controller
        self.error[0] = self.setpoint[0] - self.drone_orientation[0]
        self.error[1] = self.setpoint[1] - self.drone_orientation[1]
        self.error[2] = self.setpoint[2] - self.drone_orientation[2]

        # I controller
        self.integral_error[0] = self.integral_error[0] + (self.error[0] * self.sample_time)
        self.integral_error[1] = self.integral_error[1] + (self.error[1] * self.sample_time)
        self.integral_error[2] = self.integral_error[2] + (self.error[2] * self.sample_time)

        # D controller
        self.derivative_error[0] = (self.error[0] - self.prev_error[0])/self.sample_time
        self.derivative_error[1] = (self.error[1] - self.prev_error[1])/self.sample_time
        self.derivative_error[2] = (self.error[2] - self.prev_error[2])/self.sample_time


    def update_roll(self):
        self.roll += float(self.error[0]) * self.Kp[0] + float(self.integral_error[0]) * self.Ki[0] + float(self.derivative_error[0]) * self.Kd[0]


    def update_pitch(self):
        self.pitch += float(self.error[1]) * self.Kp[1] + float(self.integral_error[1]) * self.Ki[1] + float(self.derivative_error[1]) * self.Kd[1]


    def update_yaw(self):
        self.yaw += float(self.error[2]) * self.Kp[2] + float(self.integral_error[2]) * self.Ki[2] + float(self.derivative_error[2]) * self.Kd[2]

        
    def set_pwm(self):
        self.pwm[0] = self.throttle + self.yaw + self.pitch + self.roll
        self.pwm[1] = self.throttle - self.yaw - self.pitch + self.roll
        self.pwm[2] = self.throttle + self.yaw - self.pitch - self.roll
        self.pwm[3] = self.throttle - self.yaw + self.pitch - self.roll


    def update_error(self):
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]
        self.integral_error = self.integral_error + self.error

        # rospy.Rate(1/self.sample_time)
        # self.yaw_error.publish(self.out_yaw)
        # self.pitch_error.publish(self.out_pitch)
        # self.roll_error.publish(self.out_roll)

        
if __name__ == '__main__':
    ac = attitude_controller()
    ac.setpoint = [10,0,0]
    for i in range(0,100):
        ac.pid()
        ac.update_roll()
        print(ac.drone_orientation)
