class Motor:
    """Класс моторов.

    Аргументы:

    name -- название мотора;

    minAngPos -- минимальный угол положения;

    maxAngPos -- максимальный угол положения;

    torq -- максимальное значение тока;

    vel -- максимальное значение скорости оборотов;

    kP -- П коэффициент для ПИД-регулятора;

    kI -- И коэффициент для ПИД-регулятора;

    kD -- Д коэффициент для ПИД-регулятора;

    t -- период;

    description -- описание мотора.
    """

    def __init__(self, name, minAngPos, maxAngPos, torq, vel, kP, kI, kD, t, description) -> None:
        self.name = name
        self.minAngPos = minAngPos
        self.maxAngPos = maxAngPos
        self.torq = torq
        self.vel = vel
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.t = t
        self.setpoint = 0
        self.error = 0
        self.integral_error = 0
        self.error_last = 0
        self.derivative_error = 0
        self.output = 0
        self.current_position = 0.0
        self.description = description

    def pid_compute(self, setpoint) -> float:
        """Работа ПИД-регулятора.

        Аргументы:

        setpoint -- требуемое значение.
        """
        self.setpoint = float(setpoint)
        # Проверяем корректность данных
        if self.setpoint < self.minAngPos:
            self.setpoint = self.minAngPos

        if self.setpoint > self.maxAngPos:
            self.setpoint = self.maxAngPos

        self.error = self.setpoint - self.current_position
        self.integral_error += self.error * self.t
        self.derivative_error = (self.error - self.error_last) / self.t
        self.error_last = self.error
        self.output = self.kP * self.error +\
                      self.kI * self.integral_error +\
                      self.kD * self.derivative_error

        if self.output > self.torq:
            self.output = self.torq

        if self.output < (-1 * self.torq):
            self.output = (-1 * self.torq)

        return self.output
