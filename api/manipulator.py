from enum import Enum
from typing import Tuple, Union, Sequence

import serial

_USTEPS_PER_UM_ = 25


class Mode(Enum):
    ABSOLUTE = b'a'
    RELATIVE = b'b'


class Resolution(Enum):
    LOW = 0  # 10 uSteps/step
    HIGH = 1  # 50 uSteps/step


_Num = Union[int, float]


class Manipulator:

    def __init__(self, comm_port: str):

        self.serial_conn = serial.Serial(comm_port, baudrate=9600,
                                         bytesize=serial.EIGHTBITS,
                                         parity=serial.PARITY_NONE,
                                         stopbits=serial.STOPBITS_ONE,
                                         timeout=10,
                                         write_timeout=10)

    def __del__(self):
        self.serial_conn.close()

    def get_current_position(self):
        """
        Get the micromanipulator position
        :return: A tuple of floats (x,y,z) of the position in um accurate
        to 0.04 um
        """
        self.serial_conn.write(b'c\r')

        # returns 'xxxxyyyyzzzzCR' in uSteps
        pos_bytes = self.serial_conn.read(13)

        x = int.from_bytes(pos_bytes[0:4], byteorder='little', signed=True)
        y = int.from_bytes(pos_bytes[4:8], byteorder='little', signed=True)
        z = int.from_bytes(pos_bytes[8:12], byteorder='little', signed=True)

        x_pos = float(x / _USTEPS_PER_UM_)
        y_pos = float(y / _USTEPS_PER_UM_)
        z_pos = float(z / _USTEPS_PER_UM_)

        return x_pos, y_pos, z_pos

    def go_to_position(self, x: _Num, y: _Num, z: _Num):
        """
        Direct the micromanipulator to a position within an accuracy of 0.04 um

        :param x: X coordinate in um
        :param y: Y coordinate in um
        :param z: Z coordinate in um
        """

        x_bytes = int(x * _USTEPS_PER_UM_).to_bytes(4,
                                                    byteorder='little',
                                                    signed=True)
        y_bytes = int(y * _USTEPS_PER_UM_).to_bytes(4,
                                                    byteorder='little',
                                                    signed=True)
        z_bytes = int(z * _USTEPS_PER_UM_).to_bytes(4,
                                                    byteorder='little',
                                                    signed=True)

        self.serial_conn.write(b'm' + x_bytes + y_bytes + z_bytes + b'\r')

        # Wait for response
        self.serial_conn.read()

    def send_and_execute_moves(self, moves: Sequence[Tuple[_Num, _Num, _Num]],
                               program_num: int = 1):
        """
        Sends and executes a sequence of moves on the manipulator.
        (Max 99 moves at a time)

        :param moves: List of (x, y, z) coordinates in um
        :param program_num: Optional program number between 1 and 10
        """

        if len(moves) > 99:
            raise ValueError('Maximum number of moves exceed. Limit is 99')

        if not (1 <= program_num <= 10):
            raise ValueError('Program number must be between 1 and 10')

        # Header information:
        # Command is 'd' followed by the program number followed by the
        # number of moves
        mv_len_bytes = len(moves).to_bytes(1, byteorder='little', signed=False)
        byte_str = b'd' + program_num.to_bytes(1, byteorder='little',
                                               signed=False) + mv_len_bytes

        for mv in moves:
            x_int = int(mv[0] * _USTEPS_PER_UM_)
            x_bytes = x_int.to_bytes(4, byteorder='little', signed=True)
            y_int = int(mv[1] * _USTEPS_PER_UM_)
            y_bytes = y_int.to_bytes(4, byteorder='little', signed=True)
            z_int = int(mv[2] * _USTEPS_PER_UM_)
            z_bytes = z_int.to_bytes(4, byteorder='little', signed=True)
            byte_str += x_bytes + y_bytes + z_bytes

        byte_str += b'\r'

        # Send program
        self.serial_conn.write(byte_str)

        # Wait for confirmation
        self.serial_conn.read()

        # Execute program (Command 'k')
        self.serial_conn.write(b'k' +
                               program_num.to_bytes(1, byteorder='little',
                                                    signed=False) + b'\r')

        # Wait for completion
        self.serial_conn.read()

    def set_velocity(self, velocity: _Num, resolution: Resolution):
        """
        Set the velocity of the manipulator. Two resolutions are available:

        Resolution.HIGH allows for increments of 0.4um/second with max
        velocity of 1310 um/s

        Resolution.LOW allows for increments of 2um/second with max
        velocity of 6500 um/s(recommended maximum 3000 um/s)

        :param velocity: velocity value in um/second
        :param resolution: Resolution either Resolution.HIGH
        (0.4um/second) or Resolution.LOW (2um/second)
        """

        if velocity <= 0:
            raise ValueError('Velocity must be positive')

        if resolution == Resolution.HIGH:
            steps = int((velocity * _USTEPS_PER_UM_) / 50)
        elif resolution == Resolution.LOW:
            steps = int((velocity * _USTEPS_PER_UM_) / 10)
        else:
            raise ValueError('Use either HIGH_RESOLUTION or LOW_RESOLUTION')

        steps = (resolution.value << 15) | steps

        self.serial_conn.write(b'V' + steps.to_bytes(2, 'little') + b'\r')

        # Wait for response
        self.serial_conn.read()

    def set_origin(self):
        """
        Sets the origin of the manipulator
        """
        self.serial_conn.write(b'o\r')

        # Wait for response
        self.serial_conn.read()

    def refresh_display(self):
        """
        Refreshes the display on the manipulator
        """
        self.serial_conn.write(b'n\r')

        # Wait for response
        self.serial_conn.read()

    def set_mode(self, mode: Mode):
        """
        Sets the mode of the manipulator
        :param mode: options are ABSOLUTE or RELATIVE
        """

        self.serial_conn.write(mode.value + b'\r')

        # Wait for response
        self.serial_conn.read()

    def interrupt(self):
        """
        Interrupts the manipulator
        """
        self.serial_conn.write(hex(3))

        # Wait for response
        self.serial_conn.read()

    def continue_operation(self):
        """
        Resumes an operation on the manipulator
        """
        self.serial_conn.write(b'e\r')

        # Wait for response
        self.serial_conn.read()

    def reset(self):
        """
        Resets the manipulator. No value is returned from the manipulator
        """
        self.serial_conn.write(b'r\r')

    def get_status(self):
        """
        Returns a dict containing all status information from the mainpulator

        :return: Dict of all status information
        """
        self.serial_conn.write(b's\r')

        status_bytes = self.serial_conn.read(33)

        flag_byte = status_bytes[0]
        flag_2_byte = status_bytes[15]

        # Define ROE_DIR
        if (flag_byte & (1 << 4)) == (1 << 4):
            roe_dir = 'Negative'
        else:
            roe_dir = 'Positive'

        # Define REL_ABS_F
        if (flag_byte & (1 << 5)) == (1 << 5):
            rel_abs_f = 'Absolute'
        else:
            rel_abs_f = 'Relative'

        # Define MODE_F
        if (flag_byte & (1 << 6)) == (1 << 6):
            mode_f = 'Continuous'
        else:
            mode_f = 'Pulse'

        # Define STORE_F
        if (flag_byte & (1 << 7)) == (1 << 7):
            store_f = 'Stored'
        else:
            store_f = 'Erased'

        # Define LOOP_MODE
        if (flag_2_byte & (1 << 0)) == (1 << 0):
            loop_mode = 'Loop'
        else:
            loop_mode = 'Execute Once'

        # Define LEARN_MODE
        if (flag_2_byte & (1 << 1)) == (1 << 1):
            learn_mode = 'Learning'
        else:
            learn_mode = 'Not Learning'

        # Define STEP_MODE
        if (flag_2_byte & (1 << 2)) == (1 << 2):
            step_mode = '50 usteps/step'
        else:
            step_mode = '10 usteps/step'

        # Define JOYSTICK_SIDE
        if (flag_2_byte & (1 << 3)) == (1 << 3):
            joystick_side = 'Enabled'
        else:
            joystick_side = 'Disabled'

        # Define ENABLE_JOYSTICK
        if (flag_2_byte & (1 << 4)) == (1 << 4):
            enable_joystick = 'Enabled'
        else:
            enable_joystick = 'Keypad'
        # Define ENABLE_ROE_SWITCH
        if (flag_2_byte & (1 << 5)) == (1 << 5):
            enable_roe_switch = 'Enabled'
        else:
            enable_roe_switch = 'Disabled'

        # Define SWITCHES_4_AND_5
        if (flag_2_byte & (1 << 6)) == (1 << 6):
            switches_4_and_5 = 'Enabled'
        else:
            switches_4_and_5 = 'Disabled'

        # Define REVERSE_IT
        if (flag_2_byte & (1 << 7)) == (1 << 7):
            reverse_it = 'Reversed'
        else:
            reverse_it = 'Normal Sequence'

        # Define int to feed to JUMPSPD
        jumpspeed = int.from_bytes(status_bytes[16:18], byteorder='little')

        # Define int to feed to HIGHSPD
        highspeed = int.from_bytes(status_bytes[18:20], byteorder='little')

        # Define int to feed to DEAD
        dead = int.from_bytes(status_bytes[20:22], byteorder='little')

        # Define int to feed to WATCHDOG
        watchdog = int.from_bytes(status_bytes[22:24], byteorder='little')

        # Define int to feed to STEP_DIV
        step_div = int.from_bytes(status_bytes[24:26], byteorder='little')

        # Define int to feed to STEP_MUL
        step_mul = int.from_bytes(status_bytes[26:28], byteorder='little')

        # Define x speed resolution
        xspd_res_byte = int.from_bytes(status_bytes[28:30], byteorder='little')
        if (xspd_res_byte & (1 << 15)) == (1 << 15):
            xspeed_res = 'High Resolution'
        else:
            xspeed_res = 'Low Resolution'

        # Define x speed
        xspeed_bytes = int.from_bytes(status_bytes[28:30], byteorder='little')
        xspeed = xspeed_bytes & ~(1 << 15)
        status = {
            'FLAGS': {
                'SETUP': flag_byte & 0b00001111,
                'ROE_DIR': roe_dir,
                'REL_ABS_F': rel_abs_f,
                'MODE_F': mode_f,
                'STORE_F': store_f
            },
            'UDIRX': status_bytes[1],
            'UDIRY': status_bytes[2],
            'UDIRZ': status_bytes[3],
            'ROE_VARI': int.from_bytes(status_bytes[4:6], byteorder='little'),
            'UOFFSET': int.from_bytes(status_bytes[6:8], byteorder='little'),
            'URANGE': int.from_bytes(status_bytes[8:10], byteorder='little'),
            'PULSE': int.from_bytes(status_bytes[10:12], byteorder='little'),
            'USPEED': int.from_bytes(status_bytes[12:14], byteorder='little'),
            'INDEVICE': status_bytes[14],
            'FLAGS_2': {
                'LOOP_MODE': loop_mode,
                'LEARN_MODE': learn_mode,
                'STEP_MODE': step_mode,
                'JOYSTICK_SIDE': joystick_side,  # SW2_MODE
                'ENABLE_JOYSTICK': enable_joystick,  # SW1_MODE
                'ENABLE_ROE_SWITCH': enable_roe_switch,  # SW3_MODE
                '4_AND_5_SWITCHES': switches_4_and_5,  # SW4_MODE
                'REVERSE_IT': reverse_it
            },
            'JUMPSPD': jumpspeed,
            'HIGHSPD': highspeed,
            'DEAD': dead,
            'WATCHDOG': watchdog,
            'STEP_DIV': step_div,
            'STEP_MUL': step_mul,
            'XSPEED_RES': xspeed_res,
            'XSPEED': xspeed,
            # TODO Bytes 31 and 32 Could be integer or Binary Coded decimal
            'VERSION': status_bytes[30:32]
        }

        # Convert the XSPEED back to an actual velocity value

        if status['XSPEED_RES'] == 'Low Resolution':
            status['XSPEED'] *= 10 / _USTEPS_PER_UM_
        else:
            status['XSPEED'] *= 50 / _USTEPS_PER_UM_

        return status
