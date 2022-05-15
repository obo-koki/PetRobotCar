class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.first_flag = True
        self.pre_error = 0
        self.i_error = 0
    
    def get_control_value(self, error):
        if (self.first_flag):
            self.first_flag = False
            d_error = error
            self.pre_error = error
        else:
            d_error = (error - self.pre_error) / self.dt
            self.pre_error = error
        p_error = error
        self.i_error += error * self.dt

        return -self.Kp * p_error - self.Ki * self.i_error - self.Kd * d_error

    def reset(self):
        self.first_flag = True
        self.pre_error = 0
        self.i_error = 0