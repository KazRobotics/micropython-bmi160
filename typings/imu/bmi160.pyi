import machine
from enum import Enum, IntEnum
from imu.access import Bitfield as Bitfield, Register16 as Register16, Register8U as Register8U, SplitRegister as SplitRegister
from imu.imu import IMU as IMU
from typing import TypeAlias, Literal, NamedTuple

Axis = Literal['+x','+y','+z','-x','-y','-z']
Axes = Literal['x', 'y', 'z', 'xy', 'xz', 'yz', 'xyz']
Sensor: TypeAlias = Literal['gyro', 'accel', 'both']

class AxisTuple(NamedTuple):
    x: float
    y: float
    z: float


class Reg(IntEnum):
    """
    Register addresses

    Enumeration of all available registers and their addresses. Registers are
    named per the datasheet; most 2-byte registers have an alias with a more
    readable name - e.g. DATA_8 is aliased to GYR_X for the gyro x value.
    """
    CHIP_ID = 0
    ERR_REG = 2
    PMU_STATUS = 3
    DATA = 4
    MAG_X = 4
    DATA_0 = 4
    DATA_1 = 5
    MAG_Y = 6
    DATA_2 = 6
    DATA_3 = 7
    MAG_Z = 8
    DATA_4 = 8
    DATA_5 = 9
    HALL = 10
    DATA_6 = 10
    DATA_7 = 11
    GYRO = 12
    GYR_X = 12
    DATA_8 = 12
    DATA_9 = 13
    GYR_Y = 14
    DATA_10 = 14
    DATA_11 = 15
    GYR_Z = 16
    DATA_12 = 16
    DATA_13 = 17
    ACC = 18
    ACC_X = 18
    DATA_14 = 18
    DATA_15 = 19
    ACC_Y = 20
    DATA_16 = 20
    DATA_17 = 20
    ACC_Z = 22
    DATA_18 = 22
    DATA_19 = 23
    SENSORTIME = 24
    SENSORTIME_0 = 24
    SENSORTIME_1 = 25
    SENSORTIME_2 = 32
    STATUS = 27
    INT_STATUS_0 = 28
    INT_STATUS_1 = 29
    INT_STATUS_2 = 30
    INT_STATUS_3 = 31
    TEMP = 32
    TEMPERATURE_0 = 32
    TEMPERATURE_1 = 33
    FIFO_LENGTH = 34
    FIFO_LENGTH_0 = 34
    FIFO_LENGTH_1 = 35
    FIFO = 36
    ACC_CONF = 64
    ACC_RANGE = 65
    GYR_CONF = 66
    GYR_RANGE = 67
    MAG_CONF = 68
    FIFO_DOWNS = 69
    FIFO_CONFIG_0 = 70
    FIFO_CONFIG_1 = 71
    MAG_IF_0 = 75
    MAG_IF_1 = 76
    MAG_IF_2 = 77
    MAG_IF_3 = 78
    MAG_IF_4 = 79
    INT_EN_0 = 80
    INT_EN_1 = 81
    INT_EN_2 = 82
    INT_OUT_CTRL = 83
    INT_LATCH = 84
    INT_MAP_0 = 85
    INT_MAP_1 = 86
    INT_MAP_2 = 87
    INT_DATA_0 = 88
    INT_DATA_1 = 89
    INT_LOWHIGH_0 = 90
    INT_LOWHIGH_1 = 91
    INT_LOWHIGH_2 = 92
    INT_LOWHIGH_3 = 93
    INT_LOWHIGH_4 = 94
    INT_MOTION_0 = 95
    INT_MOTION_1 = 96
    INT_MOTION_2 = 97
    INT_MOTION_3 = 98
    INT_TAP_0 = 99
    INT_TAP_1 = 100
    INT_ORIENT_0 = 101
    INT_ORIENT_1 = 102
    INT_FLAT_0 = 103
    INT_FLAT_1 = 104
    FOC_CONF = 105
    CONF = 106
    IF_CONF = 107
    PMU_TRIGGER = 108
    SELF_TEST = 109
    NV_CONF = 112
    OFFSET_0 = 113
    OFF_ACC_X = 113
    OFFSET_1 = 114
    OFF_ACC_Y = 114
    OFFSET_2 = 115
    OFF_ACC_Z = 115
    OFFSET_3 = 116
    OFF_GYR_X_L = 116
    OFFSET_4 = 117
    OFF_GYR_Y_L = 117
    OFFSET_5 = 118
    OFF_GYR_Z_L = 118
    OFFSET_6 = 119
    STEP_CNT = 120
    STEP_CNT_0 = 120
    STEP_CNT_1 = 121
    STEP_CONF_0 = 122
    STEP_CONF_1 = 123
    CMD = 126

class Bit(Enum):
    """
    Bit fields within registers

    Fields are named as per the datasheet. Single-bit values are stored as the
    bit position; multi-bit values are stored as a 2-tuple with the start and
    end bits (inclusive).
    """
    fatal_err = 0
    err_code = (1, 4)
    i2c_fail_err = 5
    drop_cmd_err = 6
    mag_drdy_err = 7
    mag_pmu_status = (0, 1)
    gyr_pmu_status = (2, 3)
    acc_pmu_status = (4, 5)
    gyr_self_test_ok = 1
    mag_man_op = 2
    foc_rdy = 3
    nvm_rdy = 4
    drdy_mag = 5
    drdy_gyr = 6
    drdy_acc = 7
    step_int = 0
    signmot_int = 1
    anym_int = 2
    pmu_trigger_int = 3
    d_tap_int = 4
    s_tap_int = 5
    orient_int = 6
    flat_int = 7
    highg_int = 2
    lowg_int = 3
    drdy_int = 4
    ffull_int = 5
    fwm_int = 6
    nomo_int = 7
    anym_first_x = 0
    anym_first_y = 1
    anym_first_z = 2
    anym_sign = 3
    tap_first_x = 4
    tap_first_y = 5
    tap_first_z = 6
    tap_sign = 7
    high_first_x = 0
    high_first_y = 1
    high_first_z = 2
    high_sign = 3
    orient_1_0 = (4, 5)
    orient_2 = 6
    flat = 7
    fifo_byte_counter_10_8 = (0, 2)
    acc_odr = (0, 3)
    acc_bwp = (4, 6)
    acc_us = 7
    acc_range = (0, 3)
    gyr_odr = (0, 3)
    gyr_bwp = (4, 5)
    gyr_range = (0, 2)
    mag_odr = (0, 3)
    gyr_fifo_downs = (0, 2)
    gyr_fifo_filt_data = 3
    acc_fifo_downs = (4, 6)
    acc_fifo_filt_data = 7
    fifo_time_en = 1
    fifo_tag_int2_en = 2
    fifo_tag_int1_en = 3
    fifo_header_en = 4
    fifo_mag_en = 5
    fifo_acc_en = 6
    fifo_gyr_en = 7
    i2c_device_addr = (1, 7)
    mag_rd_burst = (0, 1)
    mag_offset = (3, 5)
    mag_manual_en = 7
    foc_acc_z = (0, 1)
    foc_acc_y = (2, 3)
    foc_acc_x = (4, 5)
    foc_gyr_en = 6
    nvm_prog_en = 1
    spi3 = 0
    if_mode = (4, 5)
    gyr_sleep_trigger = (0, 2)
    gyr_wakeup_trigger = (3, 4)
    gyr_sleep_state = 5
    wakeup_int = 6
    acc_self_test_enable = (0, 1)
    acc_self_test_sign = 2
    acc_self_test_amp = 3
    gyr_self_test_enable = 4
    spi_en = 0
    i2c_wdt_sel = 1
    i2c_wdt_en = 2
    u_spare_0 = 3
    off_gyr_x_9_8 = (0, 1)
    off_gyr_y_9_8 = (2, 3)
    off_gyr_z_9_8 = (4, 5)
    acc_off_en = 6
    gyr_off_en = 7
    step_conf_10_8 = (0, 2)
    step_cnt_en = 3

class Def:
    """
    Definitions used in the registers

    Provides some constant values used by various registers. Values are
    organised with the field name as per the datasheet. Note that not all
    values may be present.

    Some values are non-const as micropython throws errors with identical
    names.
    """
    class error_code(IntEnum):
        NoError = 0
        Error1 = 1
        Error2 = 2
        LPIFilter = 3
        ODRMismatch = 6
        LPFilter = 7
    class pmu_status(IntEnum):
        Suspend = 0
        Normal = 1
        LowPower = 2
        FastStart = 3
    class acc_odr(IntEnum):
        hz25d32 = 1
        hz25d16 = 2
        hz25d8 = 3
        hz25d4 = 4
        hz25d2 = 5
        hz25 = 6
        hz50 = 7
        hz100 = 8
        hz200 = 9
        hz400 = 10
        hz800 = 11
        hz1600 = 12
    class acc_bwp(IntEnum):
        Normal = 2
        OSR2 = 1
        OSR4 = 0
    class acc_range(IntEnum):
        pm2 = 3
        pm4 = 5
        pm8 = 8
        pm16 = 12
    class gyr_odr(IntEnum):
        hz25 = 6
        hz50 = 7
        hz100 = 8
        hz200 = 9
        hz400 = 10
        hz800 = 11
        hz1600 = 12
        hz3200 = 13
    class gyr_bwp(IntEnum):
        Normal = 2
        OSR2 = 1
        OSR4 = 0
    class gyr_range(IntEnum):
        pm2k = 0
        pm1k = 1
        pm500 = 2
        pm250 = 3
        pm125 = 4
    class cmd(IntEnum):
        start_foc = 3
        acc_set_pmu_normal = 17
        acc_set_pmu_low = 18
        gyr_set_pmu_normal = 21
        gyr_set_pmu_fast = 23
        prog_nvm = 160
        fifo_flush = 176
        int_reset = 177
        softreset = 182
        step_cnt_clr = 178

class Map:
    """
    Useful chip-specific mappings of values
    """
    gyro_range_map: Incomplete
    gyro_odr_map: Incomplete
    acc_range_map: Incomplete

class _BMI160:
    """
    Low-level BMI160 register access

    Provides low-level access to the BMI160's registers; does not perform
    higher-level processing.

    Registers with specific types, e.g. bools or definitions, are typed
    appropriately.

    This class does not implement data transmission directly; see
    :class:`_BMI160_I2C` and :class:`_BMI160_SPI`.

    .. todo:: See if we can fix the type checking errors because of this
    """
    chip_id: int
    fatal_err: bool
    error_code: Def.error_code
    mag_status: Def.pmu_status
    gyr_status: Def.pmu_status
    acc_status: Def.pmu_status
    gyro_x: int
    gyro_y: int
    gyro_z: int
    acc_x: int
    acc_y: int
    acc_z: int
    @property
    def gyro(self) -> AxisTuple:
        """
        Raw gyro X, Y, Z readings

        Guaranteed to be from the same measurement. No scaling is performed.
        """
    @property
    def acc(self) -> AxisTuple:
        """
        Raw acceleration X, Y, Z readings

        Guaranteed to be from the same measurement. No scaling is performed.
        """
    @property
    def motion6(self) -> tuple[int, int, int, int, int, int]:
        """
        Raw 6-axis motion readings

        Gyro X, Y, Z followed by acceleration X, Y, Z. Guaranteed to be from the
        same measurement. No scaling is performed.
        """
    gyro_st_ok: bool
    calibrated: bool
    temp: int
    fifo_length: int
    fifo_buffer: bytearray
    def fifo(self) -> memoryview:
        """
        Raw read of FIFO queue

        Reads all complete frames from the FIFO queue as bytes (according to
        `fifo_length`). Does not perform any unpacking or processing.

        If no frames are available, returns an empty bytes object.
        """
    acc_datarate: Def.acc_odr
    acc_bandwidth: Def.acc_bwp
    acc_undersample: Def.acc_us
    acc_range: Def.acc_range
    gyro_datarate: Def.gyr_odr
    gyro_bandwidth: Def.gyr_bwp
    gyro_range: Def.gyr_range
    gyro_fifo_ds: int
    gyro_fifo_use_filter: bool
    acc_fifo_ds: int
    acc_fifo_use_filter: bool
    fifo_use_headers: bool
    fifo_enable_gyro: bool
    fifo_enable_acc: bool
    cal_enable_gyro: bool
    @property
    def cal_enable_accel(self) -> str: ...
    @cal_enable_accel.setter
    def cal_enable_accel(self, g_dir: Axis | None) -> None: ...
    autocal_gyro: bool
    autocal_acc: bool
    def command(self, cmd: Def.cmd) -> None: ...

class _BMI160_I2C(_BMI160):
    """
    I2C interface to the BMI160
    """
    i2c: machine.I2C
    addr: int
    def __init__(self, i2c: machine.I2C, addr: int = 105) -> None: ...

class _BMI160_SPI(_BMI160):
    """
    SPI interface to the BMI160

    Untested. Use at your own risk.
    """
    spi: machine.SPI
    cs: machine.Pin
    def __init__(self, spi: machine.SPI, cs: machine.Pin) -> None: ...

class BMI160(IMU):
    """
    Interface to the BMI160 IMU

    If required, the low-level register interface is available on the ``bmi``
    property.

    .. todo:: Implement accessors for accelerometer data
    """
    bmi: _BMI160
    def __init__(self, gyro: bool = True, accel: bool = True, i2c: machine.I2C | None = None, addr: int = 105, spi: machine.SPI | None = None, cs: machine.Pin | None = None) -> None:
        """
        Initialise a BMI160 IMU

        The BMI160 may be connected over either I2C or SPI. If I2C is connected,
        an i2c bus instance should be passed, and the address if different to
        the default. If connected via spi, an SPI bus instance should be passed
        with a separate chip-select pin.

        The gyroscope and accelerometer can be individually enabled or disabled
        with the *gyro* or *accel* parameters.

        By default, hardware buffering is enabled for all enabled sensors.
        Sensors are configured to a default ODR of 200 Hz.
        """
    def calibrate(self, gyro: bool = True, accel: Axis | None = '-z') -> None:
        """
        Calibrate the BMI160

        Performs calibration using the FOC feature of the BMI160, and enables
        FOC compensation at a hardware level.
        """
    @property
    def gyro_ranges(self) -> tuple[int, ...]: ...
    @property
    def gyro_range(self) -> int: ...
    @gyro_range.setter
    def gyro_range(self, value: int) -> None: ...
    @property
    def accel_ranges(self) -> tuple[int, ...]: ...
    @property
    def accel_range(self) -> int: ...
    @accel_range.setter
    def accel_range(self, value: int) -> None: ...
    def track_angles(self, axes: Axes | None = None) -> None:
        """
        Configure gyroscope angle tracking

        Enables gyroscope data in the hardware FIFO buffer. If the buffer
        overflows, angles will not be accurate.

        Header mode in the FIFO queue is currently not supported; if headers
        are enabled they will be disabled.
        """
    def update(self) -> None:
        """
        Update the IMU state

        This method only has meaning if the FIFO buffer is enabled for at least
        one sensor.
        """
    @property
    def angles(self) -> AxisTuple: ...
    @property
    def gyro(self) -> AxisTuple: ...
    @property
    def gyro_inst(self) -> AxisTuple: ...
    @property
    def accel(self) -> AxisTuple: ...
    @property
    def accel_inst(self) -> AxisTuple: ...
    @property
    def motion6(self) -> tuple[float, float, float, float, float, float]: ...
    @property
    def temperature(self) -> float:
        """
        Sensor temperature in °C
        """
    @property
    def use_hw_buffer(self) -> Sensor | None: ...
    @use_hw_buffer.setter
    def use_hw_buffer(self, sensor: Sensor | None) -> None: ...
