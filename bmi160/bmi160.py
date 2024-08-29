import struct
from typing import Literal

from .access import Bitfield, UCharRegister, ShortRegister, SplitRegister
from .constants import Register, Field, Def


class BMI160:
    """
    BMI160 MicroPython driver

    .. todo:: Consider moving advanced processing to a different class/subclass
    """
    chip_id = UCharRegister(Register.CHIP_ID)
    fatal_err = Bitfield(Register.ERR_REG, Field.fatal_err)
    error_code = Bitfield(Register.ERR_REG, Field.err_code)

    mag_pmu_status = Bitfield(Register.PMU_STATUS, Field.mag_pmu_status)
    gyr_pmu_status = Bitfield(Register.PMU_STATUS, Field.gyr_pmu_status)
    acc_pmu_status = Bitfield(Register.PMU_STATUS, Field.acc_pmu_status)

    gyro_x = ShortRegister(Register.GYR_X)
    gyro_y = ShortRegister(Register.GYR_Y)
    gyro_z = ShortRegister(Register.GYR_Z)

    acc_x = ShortRegister(Register.ACC_X)
    acc_y = ShortRegister(Register.ACC_Y)
    acc_z = ShortRegister(Register.ACC_Z)

    @property
    def gyro_raw(self) -> tuple[int, int, int]:
        """
        Raw gyro X, Y, Z readings

        Guaranteed to be from the same measurement. No scaling is performed.
        """
        raw = self._read(Register.GYRO, 6)
        return struct.unpack('<hhh', raw)

    @property
    def acc_raw(self) -> tuple[int, int, int]:
        """
        Raw acceleration X, Y, Z readings

        Guaranteed to be from the same measurement. No scaling is performed.
        """
        raw = self._read(Register.ACC, 6)
        return struct.unpack('<hhh', raw)

    @property
    def motion6_raw(self) -> tuple[int, int, int, int, int, int]:
        """
        Raw 6-axis motion readings

        Gyro X, Y, Z followed by acceleration X, Y, Z. Guaranteed to be from the
        same measurement. No scaling is performed.
        """
        raw = self._read(Register.GYRO, 12)
        return struct.unpack('<hhhhhh', raw)

    gyro_st_ok = Bitfield(Register.STATUS, Field.gyr_self_test_ok)
    calibrated = Bitfield(Register.STATUS, Field.foc_rdy)

    temp_raw = ShortRegister(Register.TEMP)

    @property
    def temperature(self):
        """
        Sensor temperature in °C
        """
        scaled = self.temp_raw / (0.5 ** 9)
        return 23 + scaled

    fifo_length = SplitRegister(
            Register.FIFO_LENGTH_0,
            Register.FIFO_LENGTH_1,
            Field.fifo_byte_counter_10_8,
            )

    def fifo_raw(self) -> bytes:
        """
        Raw read of FIFO queue

        Reads all complete frames from the FIFO queue as bytes (according to
        `fifo_length`). Does not perform any unpacking or processing.
        """
        return self._read(Register.FIFO, self.fifo_length)

    def fifo_packets(self):
        """
        Read FIFO packets

        Do some reading of the FIFO queue and post-processing for ease of
        interpretation. Also figure out exactly how much work to do here.
        """
        raise NotImplementedError

    def fifo_scaled(self):
        """
        Read FIFO data

        Do some reading of the FIFO queue.
        """
        raise NotImplementedError

    acc_datarate = Bitfield(Register.ACC_CONF, Field.acc_odr)
    acc_bandwidth = Bitfield(Register.ACC_CONF, Field.acc_bwp)
    acc_undersample = Bitfield(Register.ACC_CONF, Field.acc_us)
    acc_range = Bitfield(Register.ACC_RANGE, Field.acc_range)

    gyro_datarate = Bitfield(Register.GYR_CONF, Field.gyr_odr)
    gyro_bandwidth = Bitfield(Register.GYR_CONF, Field.gyr_bwp)
    gyro_range = Bitfield(Register.GYR_RANGE, Field.gyr_range)

    gyro_fifo_ds = Bitfield(Register.FIFO_DOWNS, Field.gyr_fifo_downs)
    gyro_fifo_use_filter = Bitfield(Register.FIFO_DOWNS, Field.gyr_fifo_filt_data)
    acc_fifo_ds = Bitfield(Register.FIFO_DOWNS, Field.acc_fifo_downs)
    acc_fifo_use_filter = Bitfield(Register.FIFO_DOWNS, Field.acc_fifo_filt_data)

    fifo_use_headers = Bitfield(Register.FIFO_CONFIG_1, Field.fifo_header_en)
    fifo_enable_gyro = Bitfield(Register.FIFO_CONFIG_1, Field.fifo_gyr_en)
    fifo_enable_acc  = Bitfield(Register.FIFO_CONFIG_1, Field.fifo_acc_en)

    cal_enable_gyro = Bitfield(Register.FOC_CONF, Field.foc_gyr_en)
    _cea_x = Bitfield(Register.FOC_CONF, Field.foc_acc_x)
    _cea_y = Bitfield(Register.FOC_CONF, Field.foc_acc_y)
    _cea_z = Bitfield(Register.FOC_CONF, Field.foc_acc_z)

    @property
    def cal_enable_accel(self) -> str:
        val = (self._cea_x << 4) | (self._cea_x << 2) | self._cea_z
        return {
                0b010000: '+x',
                0b100000: '-x',
                0b000100: '+y',
                0b001000: '-y',
                0b000001: '+z',
                0b000010: '-z',
                }.get(val, '')

    @cal_enable_accel.setter
    def cal_enable_accel(self, g_dir: Literal['+x','-x','+y','-y','+z','-z','']):
        val = {
                '+x': 0b010000,
                '-x': 0b100000,
                '+y': 0b000100,
                '-y': 0b001000,
                '+z': 0b000001,
                '-z': 0b000010,
                }.get(g_dir, 0)
        self._cea_x = val >> 4
        self._cea_y = val >> 2
        self._cea_z = val

    autocal_gyro = Bitfield(Register.OFFSET_6, Field.gyr_off_en)
    autocal_acc = Bitfield(Register.OFFSET_6, Field.acc_off_en)

    def command(self, cmd: Def.cmd):
        self._writereg(Register.CMD, cmd)

    def _read(self, register: Register, size: int) -> bytes:
        raise NotImplementedError

    def _write(self, register: Register, data: bytes):
        raise NotImplementedError

    def _readreg(self, register: Register) -> int:
        return self._read(register, 1)[0]

    def _writereg(self, register: Register, value: int):
        return self._write(register, bytes([value]))

