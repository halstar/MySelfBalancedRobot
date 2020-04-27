import utils


class Pid:

    def __init__(self, kp, ki, kd, target_value, min_value, max_value, anti_wind_up_factor):

        self.kp                 = kp
        self.ki                 = ki
        self.kd                 = kd
        self.target             = target_value
        self.min_value          = min_value
        self.max_value          = max_value
        self.anti_wind_up_value = anti_wind_up_factor * max([abs(min_value), abs(max_value)])

        self.p_value        = 0
        self.i_value        = 0
        self.d_value        = 0
        self.computed_value = 0
        self.last_error     = 0

    def reset(self):

        self.p_value        = 0
        self.i_value        = 0
        self.d_value        = 0
        self.computed_value = 0
        self.last_error     = 0

    def set_kp(self, kp):
        self.kp = kp

    def set_ki(self, ki):
        self.ki = ki

    def set_kd(self, kd):
        self.kd = kd

    def set_target(self, target_value):
        self.target = target_value

    def set_min_value(self, min_value):
        self.min_value = min_value

    def set_max_value(self, max_value):
        self.max_value = max_value

    def set_anti_wind_up(self, anti_wind_up_factor):
        self.anti_wind_up_value = anti_wind_up_factor * max([abs(self.min_value), abs(self.max_value)])

    def get_kp(self):
        return self.kp

    def get_ki(self):
        return self.ki

    def get_kd(self):
        return self.kd

    def get_target(self):
        return self.target

    def get_min_value(self, min_value):
        return min_value

    def get_max_value(self, max_value):
        return max_value

    def get_anti_wind_up(self):
        return self.anti_wind_up_value

    def update(self, current_value, time_delta):

        current_error = self.target - current_value

        self.p_value    =  current_error
        self.i_value   +=  current_error * time_delta
        self.i_value    =  utils.clamp(self.i_value, -self.anti_wind_up_value, self.anti_wind_up_value)
        self.d_value    = (current_error - self.last_error) / time_delta
        self.last_error =  current_error

        self.computed_value = self.kp * self.p_value + self.ki * self.i_value + self.kd * self.d_value
        self.computed_value = utils.clamp(self.computed_value, self.min_value, self.max_value)

        return self.computed_value

    def print_pid_info(self):

        print(" kp = {:6.2f} -  ki = {:6.2f} -     kd = {:6.2f}".format(self.kp       , self.ki       , self.kd                ))
        print("  p = {:6.2f} -   i = {:6.2f} -      d = {:6.2f}".format(self.p_value  , self.i_value  , self.d_value           ))
        print("min = {:6.1f} - max = {:6.1f} - a.w.up = {:6.1f}".format(self.min_value, self.max_value, self.anti_wind_up_value))
        print("  t = {:6.2f} - val = {:6.2f}                   ".format(self.target   , self.computed_value                    ))
