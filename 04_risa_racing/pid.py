import time
    

class SteeringController:
    def __init__(self, kp, ki, kd, min_output=-0.7, max_output=0.7):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output

        self.error = 0.0
        self.integral = 0.0
        self.previous_error = 0.0

        self.previous_time = time.time()

    def steer_controler(self, target, current):
        self.error = target - current
        
        current_time = time.time()
        dt = current_time - self.previous_time
        
        self.integral += self.error * dt
        derivative = (self.error - self.previous_error) / dt

        output = (self.kp * self.error) + (self.ki * self.integral) + (self.kd * derivative)

        output = max(self.min_output, min(output, self.max_output))

        self.previous_error = self.error
        self.previous_time = current_time

        return output
    