class StopProcessContent:
    def __init__(self,
                 robot_type: str,
                 operation_code: int,
                 status_code: int):
        self.robot_type = robot_type
        self.operation = operation_code
        self.status = status_code
