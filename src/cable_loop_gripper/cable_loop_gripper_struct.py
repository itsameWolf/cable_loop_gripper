from dataclasses import dataclass

@dataclass
class CLG_status_struct:
    current_loop_length: float = 0
    requested_loop_length: float = 0
    loop_length_at_setpoint: bool = 1
    current_force: float = 0
    requested_force: float = 0
    force_at_setpoint: bool = 0
    control_mode: bool = 1
