import numpy as np

class PIDController:
    def __init__(self, Kp=0.5, Ki=0.0095, Kd=0.03, i_limit=1.0):
        """PID Controller 생성자"""
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.i_limit = i_limit  # 적분항 anti-windup을 위한 클램프
        
        self.prev_error = 0.0
        self.integral = 0.0
        self._initialized = False

    def reset(self):
        """상태 초기화"""
        self.prev_error = 0.0
        self.integral = 0.0
        self._initialized = False

    def compute(self, error, dt=1.0):
        """PID 제어 계산"""
        dt = max(1e-3, float(dt)) # dt가 0이 되는 것을 방지
        
        if not self._initialized:
            # 첫 호출 시 미분항이 폭주하는 것을 방지
            self.prev_error = float(error)
            self._initialized = True

        self.integral += float(error) * dt
        
        # Anti-windup: 적분항 제한
        if self.i_limit is not None:
            self.integral = np.clip(self.integral, -self.i_limit, self.i_limit)

        derivative = (float(error) - self.prev_error) / dt
        output = (self.Kp * float(error)) + (self.Ki * self.integral) + (self.Kd * derivative)
        
        self.prev_error = float(error)
        return float(output)