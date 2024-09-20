import machine
import micropython
import struct
import time
from typing import TYPE_CHECKING, Literal

from imu.access import Bitfield, Register8U, Register16, Contiguous10
from imu.imu import IMU


Axis = Literal['+x','+y','+z','-x','-y','-z']
Axes = Literal['x', 'y', 'z', 'xy', 'xz', 'yz', 'xyz']
Sensor = Literal['gyro', 'accel', 'both']


if TYPE_CHECKING:
    from enum import IntEnum, Enum
    from typing import override, NamedTuple

    class AxisTuple(NamedTuple):
        x: float
        y: float
        z: float
else:
    # micropython's typing stubs don't support the override decorator yet
    override = lambda f: f
    from collections import namedtuple
    AxisTuple = namedtuple('AxisTuple', ('x', 'y', 'z'))
    # micropython doesn't have enum support
    Enum = object
    IntEnum = object


_print = False
if not _print:
    print = lambda *_: None


class Reg(IntEnum):
    """
    Register addresses

    Enumeration of all available registers and their addresses. Registers are
    named per the datasheet; most 2-byte registers have an alias with a more
    readable name - e.g. DATA_8 is aliased to GYR_X for the gyro x value.
    """
    CHIP_ID = 0x00
    ERR_REG = 0x02
    PMU_STATUS = 0x03

    DATA = 0x04

    MAG_X = 0x04
    DATA_0 = 0x04
    DATA_1 = 0x05
    MAG_Y = 0x06
    DATA_2 = 0x06
    DATA_3 = 0x07
    MAG_Z = 0x08
    DATA_4 = 0x08
    DATA_5 = 0x09
    HALL = 0x0A
    DATA_6 = 0x0A
    DATA_7 = 0x0B
    GYRO = 0x0C
    GYR_X = 0x0C
    DATA_8 = 0x0C
    DATA_9 = 0x0D
    GYR_Y = 0x0E
    DATA_10 = 0x0E
    DATA_11 = 0x0F
    GYR_Z = 0x10
    DATA_12 = 0x10
    DATA_13 = 0x11
    ACC = 0x12
    ACC_X = 0x12
    DATA_14 = 0x12
    DATA_15 = 0x13
    ACC_Y = 0x14
    DATA_16 = 0x14
    DATA_17 = 0x14
    ACC_Z = 0x16
    DATA_18 = 0x16
    DATA_19 = 0x17

    SENSORTIME = 0x18
    SENSORTIME_0 = 0x18
    SENSORTIME_1 = 0x19
    SENSORTIME_2 = 0x20

    STATUS = 0x1B
    INT_STATUS_0 = 0x1C
    INT_STATUS_1 = 0x1D
    INT_STATUS_2 = 0x1E
    INT_STATUS_3 = 0x1F

    TEMP = 0x20
    TEMPERATURE_0 = 0x20
    TEMPERATURE_1 = 0x21

    FIFO_LENGTH = 0x22
    FIFO_LENGTH_0 = 0x22
    FIFO_LENGTH_1 = 0x23
    FIFO = 0x24

    ACC_CONF = 0x40
    ACC_RANGE = 0x41
    GYR_CONF = 0x42
    GYR_RANGE = 0x43
    MAG_CONF = 0x44

    FIFO_DOWNS = 0x45
    FIFO_CONFIG_0 = 0x46
    FIFO_CONFIG_1 = 0x47

    MAG_IF_0 = 0x4B
    MAG_IF_1 = 0x4C
    MAG_IF_2 = 0x4D
    MAG_IF_3 = 0x4E
    MAG_IF_4 = 0x4F

    INT_EN_0 = 0x50
    INT_EN_1 = 0x51
    INT_EN_2 = 0x52
    INT_OUT_CTRL = 0x53
    INT_LATCH = 0x54
    INT_MAP_0 = 0x55
    INT_MAP_1 = 0x56
    INT_MAP_2 = 0x57
    INT_DATA_0 = 0x58
    INT_DATA_1 = 0x59
    INT_LOWHIGH_0 = 0x5A
    INT_LOWHIGH_1 = 0x5B
    INT_LOWHIGH_2 = 0x5C
    INT_LOWHIGH_3 = 0x5D
    INT_LOWHIGH_4 = 0x5E
    INT_MOTION_0 = 0x5F
    INT_MOTION_1 = 0x60
    INT_MOTION_2 = 0x61
    INT_MOTION_3 = 0x62
    INT_TAP_0 = 0x63
    INT_TAP_1 = 0x64
    INT_ORIENT_0 = 0x65
    INT_ORIENT_1 = 0x66
    INT_FLAT_0 = 0x67
    INT_FLAT_1 = 0x68

    FOC_CONF = 0x69
    CONF = 0x6A
    IF_CONF = 0x6B
    PMU_TRIGGER = 0x6C
    SELF_TEST = 0x6D
    NV_CONF = 0x70

    OFFSET_0 = 0x71
    OFF_ACC_X = 0x71
    OFFSET_1 = 0x72
    OFF_ACC_Y = 0x72
    OFFSET_2 = 0x73
    OFF_ACC_Z = 0x73
    OFFSET_3 = 0x74
    OFF_GYR_X_L = 0x74
    OFFSET_4 = 0x75
    OFF_GYR_Y_L = 0x75
    OFFSET_5 = 0x76
    OFF_GYR_Z_L = 0x76
    OFFSET_6 = 0x77

    STEP_CNT = 0x78
    STEP_CNT_0 = 0x78
    STEP_CNT_1 = 0x79
    STEP_CONF_0 = 0x7A
    STEP_CONF_1 = 0x7B

    CMD = 0x7E


class Bit(Enum):
    """
    Bit fields within registers

    Fields are named as per the datasheet. Single-bit values are stored as the
    bit position; multi-bit values are stored as a 2-tuple with the start and
    end bits (inclusive).
    """
    ## ERR_REG
    fatal_err = 0
    err_code = (1, 4)
    i2c_fail_err = 5
    drop_cmd_err = 6
    mag_drdy_err = 7

    ## PMU_STATUS
    mag_pmu_status = (0, 1)
    gyr_pmu_status = (2, 3)
    acc_pmu_status = (4, 5)

    ## STATUS
    gyr_self_test_ok = 1
    mag_man_op = 2
    foc_rdy = 3
    nvm_rdy = 4
    drdy_mag = 5
    drdy_gyr = 6
    drdy_acc = 7

    # INT_STATUS_0
    step_int = 0
    signmot_int = 1
    anym_int = 2
    pmu_trigger_int = 3
    d_tap_int = 4
    s_tap_int = 5
    orient_int = 6
    flat_int = 7

    ## INT_STATUS_1
    highg_int = 2
    lowg_int = 3
    drdy_int = 4
    ffull_int = 5
    fwm_int = 6
    nomo_int = 7

    ## INT_STATUS_2
    anym_first_x = 0
    anym_first_y = 1
    anym_first_z = 2
    anym_sign = 3
    tap_first_x = 4
    tap_first_y = 5
    tap_first_z = 6
    tap_sign = 7

    ## INT_STATUS_3
    high_first_x = 0
    high_first_y = 1
    high_first_z = 2
    high_sign = 3
    orient_1_0 = (4, 5)
    orient_2 = 6
    flat = 7

    ## FIFO_LENGTH_1
    fifo_byte_counter_10_8 = (0, 2)

    ## Range & Config
    acc_odr = (0, 3)
    acc_bwp = (4, 6)
    acc_us = 7
    acc_range = (0, 3)
    gyr_odr = (0, 3)
    gyr_bwp = (4, 5)
    gyr_range = (0, 2)
    mag_odr = (0, 3)

    ## FIFO_DOWNS
    gyr_fifo_downs = (0, 2)
    gyr_fifo_filt_data = 3
    acc_fifo_downs = (4, 6)
    acc_fifo_filt_data = 7

    ## FIFO_CONFIG_1
    fifo_time_en = 1
    fifo_tag_int2_en = 2
    fifo_tag_int1_en = 3
    fifo_header_en = 4
    fifo_mag_en = 5
    fifo_acc_en = 6
    fifo_gyr_en = 7

    ## MAG_IF_0, MAG_IF_1
    i2c_device_addr = (1, 7)
    mag_rd_burst = (0, 1)
    mag_offset = (3, 5)
    mag_manual_en = 7

    ## Interrupts
    # There's heaps and I don't want to add them all. Feel free to copy them
    # in yourself if you need them.

    ## Config (FOC_CONF, CONF, IF_CONF)
    foc_acc_z = (0, 1)
    foc_acc_y = (2, 3)
    foc_acc_x = (4, 5)
    foc_gyr_en = 6
    nvm_prog_en = 1
    spi3 = 0
    if_mode = (4, 5)

    ## PMU_TRIGGER
    gyr_sleep_trigger = (0, 2)
    gyr_wakeup_trigger = (3, 4)
    gyr_sleep_state = 5
    wakeup_int = 6

    ## SELF_TEST
    acc_self_test_enable = (0, 1)
    acc_self_test_sign = 2
    acc_self_test_amp = 3
    gyr_self_test_enable = 4

    ## NV_CONF
    spi_en = 0
    i2c_wdt_sel = 1
    i2c_wdt_en = 2
    u_spare_0 = 3

    ## OFFSET_6
    off_gyr_x_9_8 = (0, 1)
    off_gyr_y_9_8 = (2, 3)
    off_gyr_z_9_8 = (4, 5)
    acc_off_en = 6
    gyr_off_en = 7

    ## STEP_CONF_1
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
        NoError = 0b0000
        Error1 = 0b0001
        Error2 = 0b0010
        LPIFilter = 0b0011
        ODRMismatch = 0b0110
        LPFilter = 0b0111

    class pmu_status(IntEnum):
        Suspend = 0b00
        Normal = 0b01
        # Accel and magnet only
        LowPower = 0b10
        # Gyro only
        FastStart = 0b11

    class acc_odr(IntEnum):
        hz25d32 = 0b0001
        hz25d16 = 0b0010
        hz25d8 = 0b0011
        hz25d4 = 0b0100
        hz25d2 = 0b0101
        hz25 = 0b0110
        hz50 = 0b0111
        hz100 = 0b1000
        hz200 = 0b1001
        hz400 = 0b1010
        hz800 = 0b1011
        hz1600 = 0b1100

    class acc_bwp(IntEnum):
        Normal = 0b010
        OSR2 = 0b001
        OSR4 = 0b000

    class acc_range(IntEnum):
        pm2 = 0b0011
        pm4 = 0b0101
        pm8 = 0b1000
        pm16 = 0b1100

    class gyr_odr(IntEnum):
        hz25 = 0b0110
        hz50 = 0b0111
        hz100 = 0b1000
        hz200 = 0b1001
        hz400 = 0b1010
        hz800 = 0b1011
        hz1600 = 0b1100
        hz3200 = 0b1101

    class gyr_bwp(IntEnum):
        Normal = 0b010
        OSR2 = 0b001
        OSR4 = 0b000

    class gyr_range(IntEnum):
        pm2k = 0b000
        pm1k = 0b001
        pm500 = 0b010
        pm250 = 0b011
        pm125 = 0b100

    class cmd(IntEnum):
        start_foc = 0x03
        acc_set_pmu_normal = 0x11
        acc_set_pmu_low = 0x12
        gyr_set_pmu_normal = 0x15
        gyr_set_pmu_fast = 0x17
        prog_nvm = 0xA0
        fifo_flush = 0xB0
        int_reset = 0xB1
        softreset = 0xB6
        step_cnt_clr = 0xB2


class Map:
    """
    Useful chip-specific mappings of values
    """
    # Gyro range to scaler values
    gyro_range_map = {
        Def.gyr_range.pm2k: 16.4,
        Def.gyr_range.pm1k: 32.8,
        Def.gyr_range.pm500: 65.6,
        Def.gyr_range.pm250: 131.2,
        Def.gyr_range.pm125: 262.4,
        }

    # Bit-value to frequency for calculations
    gyro_odr_map = {
        Def.gyr_odr.hz25: 25,
        Def.gyr_odr.hz50: 50,
        Def.gyr_odr.hz100: 100,
        Def.gyr_odr.hz200: 200,
        Def.gyr_odr.hz400: 400,
        Def.gyr_odr.hz800: 800,
        Def.gyr_odr.hz1600: 1600,
        Def.gyr_odr.hz3200: 3200,
        }

    acc_range_map = {
        Def.acc_range.pm2: 32768 / 2,
        Def.acc_range.pm4: 32768 / 4,
        Def.acc_range.pm8: 32768 / 8,
        Def.acc_range.pm16: 32768 / 16,
        }


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
    chip_id: int = Register8U(Reg.CHIP_ID)
    fatal_err: bool = Bitfield(Reg.ERR_REG, Bit.fatal_err)
    error_code: Def.error_code = Bitfield(Reg.ERR_REG, Bit.err_code)

    mag_status: Def.pmu_status = Bitfield(Reg.PMU_STATUS, Bit.mag_pmu_status)
    gyr_status: Def.pmu_status = Bitfield(Reg.PMU_STATUS, Bit.gyr_pmu_status)
    acc_status: Def.pmu_status = Bitfield(Reg.PMU_STATUS, Bit.acc_pmu_status)

    gyro_x: int = Register16(Reg.GYR_X)
    gyro_y: int = Register16(Reg.GYR_Y)
    gyro_z: int = Register16(Reg.GYR_Z)

    acc_x: int = Register16(Reg.ACC_X)
    acc_y: int = Register16(Reg.ACC_Y)
    acc_z: int = Register16(Reg.ACC_Z)

    @property
    def gyro(self) -> AxisTuple:
        """
        Raw gyro X, Y, Z readings

        Guaranteed to be from the same measurement. No scaling is performed.
        """
        raw = self._read(Reg.GYRO, 6)
        return AxisTuple(*struct.unpack('<hhh', raw))

    @property
    def acc(self) -> AxisTuple:
        """
        Raw acceleration X, Y, Z readings

        Guaranteed to be from the same measurement. No scaling is performed.
        """
        raw = self._read(Reg.ACC, 6)
        return AxisTuple(*struct.unpack('<hhh', raw))

    @property
    def motion6(self) -> tuple[int, int, int, int, int, int]:
        """
        Raw 6-axis motion readings

        Gyro X, Y, Z followed by acceleration X, Y, Z. Guaranteed to be from the
        same measurement. No scaling is performed.
        """
        raw = self._read(Reg.GYRO, 12)
        return struct.unpack('<hhhhhh', raw)

    gyro_st_ok: bool = Bitfield(Reg.STATUS, Bit.gyr_self_test_ok)
    calibrated: bool = Bitfield(Reg.STATUS, Bit.foc_rdy)

    temp: int = Register16(Reg.TEMP)

    fifo_length: int = Contiguous10(Reg.FIFO_LENGTH_0)

    fifo_buffer: bytearray = bytearray(1024)

    #@micropython.native
    def fifo(self) -> memoryview:
        """
        Raw read of FIFO queue

        Reads all complete frames from the FIFO queue as bytes (according to
        `fifo_length`). Does not perform any unpacking or processing.

        If no frames are available, returns an empty bytes object.
        """
        #ts = time.ticks_us
        #t0 = ts()
        count = self.fifo_length
        #t1 = ts()
        view = memoryview(self.fifo_buffer)[:count]
        #t2 = ts()
        if count != 0:
            self._read_into(Reg.FIFO, view)
        #t3 = ts()
        #print('FIFO:{},{},{},{}'.format(t3-t0, t1-t0, t2-t1, t3-t2))
        return view

    acc_datarate: Def.acc_odr = Bitfield(Reg.ACC_CONF, Bit.acc_odr)
    acc_bandwidth: Def.acc_bwp = Bitfield(Reg.ACC_CONF, Bit.acc_bwp)
    acc_undersample: Def.acc_us = Bitfield(Reg.ACC_CONF, Bit.acc_us)
    acc_range: Def.acc_range = Bitfield(Reg.ACC_RANGE, Bit.acc_range, cache=True)

    gyro_datarate: Def.gyr_odr = Bitfield(Reg.GYR_CONF, Bit.gyr_odr, cache=True)
    gyro_bandwidth: Def.gyr_bwp = Bitfield(Reg.GYR_CONF, Bit.gyr_bwp)
    gyro_range: Def.gyr_range = Bitfield(Reg.GYR_RANGE, Bit.gyr_range, cache=True)

    gyro_fifo_ds: int = Bitfield(Reg.FIFO_DOWNS, Bit.gyr_fifo_downs)
    gyro_fifo_use_filter: bool = Bitfield(Reg.FIFO_DOWNS, Bit.gyr_fifo_filt_data)
    acc_fifo_ds: int = Bitfield(Reg.FIFO_DOWNS, Bit.acc_fifo_downs)
    acc_fifo_use_filter: bool = Bitfield(Reg.FIFO_DOWNS, Bit.acc_fifo_filt_data)

    fifo_use_headers: bool = Bitfield(Reg.FIFO_CONFIG_1, Bit.fifo_header_en, cache=True)
    fifo_enable_gyro: bool = Bitfield(Reg.FIFO_CONFIG_1, Bit.fifo_gyr_en, cache=True)
    fifo_enable_acc: bool  = Bitfield(Reg.FIFO_CONFIG_1, Bit.fifo_acc_en, cache=True)

    cal_enable_gyro: bool = Bitfield(Reg.FOC_CONF, Bit.foc_gyr_en)
    _cea_x: int = Bitfield(Reg.FOC_CONF, Bit.foc_acc_x)
    _cea_y: int = Bitfield(Reg.FOC_CONF, Bit.foc_acc_y)
    _cea_z: int = Bitfield(Reg.FOC_CONF, Bit.foc_acc_z)

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
    def cal_enable_accel(self, g_dir: Axis|None) -> None:
        val = {
                '+x': 0b010000,
                '-x': 0b100000,
                '+y': 0b000100,
                '-y': 0b001000,
                '+z': 0b000001,
                '-z': 0b000010,
                None: 0b000000,
                }.get(g_dir, 0)
        self._cea_x = val >> 4
        self._cea_y = val >> 2
        self._cea_z = val

    autocal_gyro: bool = Bitfield(Reg.OFFSET_6, Bit.gyr_off_en)
    autocal_acc: bool = Bitfield(Reg.OFFSET_6, Bit.acc_off_en)

    def command(self, cmd: Def.cmd) -> None:
        self._writereg(Reg.CMD, cmd)

    def _read(self, register: Reg|int, size: int) -> bytes:
        raise NotImplementedError

    def _write(self, register: Reg|int, data: bytes) -> None:
        raise NotImplementedError

    #@micropython.native
    def _readreg(self, register: Reg) -> int:
        return self._read(register, 1)[0]

    #@micropython.native
    def _writereg(self, register: Reg, value: int) -> None:
        return self._write(register, bytes([value]))


class _BMI160_I2C(_BMI160):
    """
    I2C interface to the BMI160
    """
    i2c: machine.I2C
    addr: int

    def __init__(self, i2c: machine.I2C, addr=0x69) -> None:
        self.i2c = i2c
        self.addr = addr
        # Should be in I2C mode by default

    #@micropython.native
    def _read(self, register: Reg|int, size: int) -> bytes:
        return self.i2c.readfrom_mem(self.addr, register, size)

    #@micropython.native
    def _read_into(self, register: Reg|int, buf: bytes|memoryview) -> None:
        self.i2c.readfrom_mem_into(self.addr, register, buf)

    #@micropython.native
    def _write(self, register: Reg|int, data: bytes) -> None:
        self.i2c.writeto_mem(self.addr, register, data)


class _BMI160_SPI(_BMI160):
    """
    SPI interface to the BMI160

    Untested. Use at your own risk.
    """
    spi: machine.SPI
    cs: machine.Pin

    def __init__(self, spi: machine.SPI, cs: machine.Pin) -> None:
        self.spi = spi
        self.cs = cs
        # Dummy-read to trigger SPI mode
        self._read(0x7F, 1)

    def _read(self, register: Reg|int, size: int) -> bytes:
        try:
            self.cs(0)
            self.spi.write(bytes([register]))
            return self.spi.read(size)
        finally:
            self.cs(1)

    def _write(self, register: Reg|int, data: bytes) -> None:
        try:
            self.cs(0)
            self.spi.write(bytes([register]))
            self.spi.write(data)
        finally:
            self.cs(1)


class BMI160(IMU):
    """
    Interface to the BMI160 IMU

    If required, the low-level register interface is available on the ``bmi``
    property.

    .. todo:: Implement accessors for accelerometer data
    """
    bmi: _BMI160

    def __init__(
            self,
            gyro=True,
            accel=True,
            i2c: machine.I2C|None = None,
            addr=0x69,
            spi: machine.SPI|None = None,
            cs: machine.Pin|None = None,
            ) -> None:
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
        if i2c is None and spi is None:
            raise ValueError('No bus specified')
        elif i2c is not None:
            print('Initializing BMI160 over I2C...')
            self.bmi = _BMI160_I2C(i2c, addr=addr)
        elif spi is not None and cs is None:
            raise ValueError('SPI bus specified with no CS pin')
        elif spi is not None and cs is not None:
            print('Initializing BMI160 over SPI...')
            self.bmi = _BMI160_SPI(spi, cs)
        else:
            raise ValueError('Invalid bus configuration')
        print('Finished initializing')

        if gyro:
            print('Waiting for gyro powerup')
            self.bmi.command(Def.cmd.gyr_set_pmu_normal)
            while self.bmi.gyr_status != Def.pmu_status.Normal:
                # Expected 55-80ms
                time.sleep_ms(10)
            self.bmi.fifo_enable_gyro = True
            self.bmi.gyro_datarate = Def.gyr_odr.hz200
        if accel:
            print('Waiting for accel powerup')
            self.bmi.command(Def.cmd.acc_set_pmu_normal)
            while self.bmi.acc_status != Def.pmu_status.Normal:
                # Excepted 3.2-3.8ms
                time.sleep_ms(1)
            self.bmi.fifo_enable_acc = True
            self.bmi.acc_datarate = Def.acc_odr.hz200
            print(f'Accel range: {self.bmi.acc_range:04b}, scaler: {Map.acc_range_map[self.bmi.acc_range]}')
        print('Sensors started')

        self._angles = 0, 0, 0
        self._accu_gyr = 0, 0, 0, 0
        self._accu_acc = 0, 0, 0, 0
        self._gyr_trk = False, False, False
        self._acc_trk = False, False, False

    @override
    def calibrate(self, gyro=True, accel: Axis|None = '-z') -> None:
        """
        Calibrate the BMI160

        Performs calibration using the FOC feature of the BMI160, and enables
        FOC compensation at a hardware level.
        """
        if not gyro and accel is None:
            return
        self.bmi.cal_enable_gyro = gyro
        self.bmi.cal_enable_accel = accel
        self.bmi.command(Def.cmd.start_foc)
        while not self.bmi.calibrated:
            time.sleep_ms(2)
        self.bmi.autocal_gyro = gyro
        self.bmi.autocal_acc = accel is not None
        self._angles = 0, 0, 0

    @property
    @override
    def gyro_ranges(self) -> tuple[int, ...]:
        return (4000, 2000, 1000, 500, 250)

    @property
    @override
    def gyro_range(self) -> int:
        R = Def.gyr_range
        return {
                R.pm2k: 4000,
                R.pm1k: 2000,
                R.pm500: 1000,
                R.pm250: 500,
                R.pm125: 250,
                }[self.bmi.gyro_range]

    @gyro_range.setter
    @override
    def gyro_range(self, value: int) -> None:
        R = Def.gyr_range
        bits = {
                4000: R.pm2k,
                2000: R.pm1k,
                1000: R.pm500,
                500: R.pm250,
                250: R.pm125,
                }.get(value, None)
        if bits is None:
            raise ValueError('Unknown range {}'.format(value))
        self.bmi.gyro_range = bits
        # Datasheet recommends a single read for data ready clear
        _ = self.bmi.gyro

    @property
    @override
    def accel_ranges(self) -> tuple[int, ...]:
        return (32, 16, 8, 4)

    @property
    @override
    def accel_range(self) -> int:
        R = Def.acc_range
        return {
                R.pm2: 4,
                R.pm4: 8,
                R.pm8: 16,
                R.pm16: 32,
                }[self.bmi.acc_range]

    @accel_range.setter
    @override
    def accel_range(self, value: int) -> None:
        R = Def.acc_range
        self.bmi.acc_range = {
                4: R.pm2,
                8: R.pm4,
                16: R.pm8,
                32: R.pm16,
                }.get(value, R.pm16)
        # Datasheet recommends a single read for data ready clear
        _ = self.bmi.acc

    @override
    def track_angles(self, axes: Axes|None = None) -> None:
        """
        Configure gyroscope angle tracking

        Enables gyroscope data in the hardware FIFO buffer. If the buffer
        overflows, angles will not be accurate.

        Header mode in the FIFO queue is currently not supported; if headers
        are enabled they will be disabled.
        """
        x = 'x' in axes if axes else False
        y = 'y' in axes if axes else False
        z = 'z' in axes if axes else False
        self._gyr_trk = x, y, z

        if not any((x, y, z)):
            self.bmi.fifo_enable_gyro = False
            return
        # TODO: don't force this
        self.bmi.fifo_use_headers = False

        heads = self.bmi.fifo_use_headers
        acc_on = self.bmi.fifo_enable_acc
        acc_odr = self.bmi.acc_datarate
        gyr_odr = self.bmi.gyro_datarate
        if acc_on and acc_odr != gyr_odr and not heads:
            raise ValueError('Sensor data rate mismatch in FIFO headless mode')
        self.bmi.fifo_enable_gyro = True

    @override
    #@micropython.native
    def update(self) -> None:
        """
        Update the IMU state

        This method only has meaning if the FIFO buffer is enabled for at least
        one sensor.
        """
        heads = self.bmi.fifo_use_headers
        fifo = self.bmi.fifo()

        if heads:
            raise NotImplementedError('Header mode not yet supported')

        raw = (struct.unpack_from('<hhh', fifo, i) for i in range(0, len(fifo), 6))
        gyro = self.bmi.fifo_enable_gyro
        acc = self.bmi.fifo_enable_acc
        gdx, gdy, gdz, gc = 0, 0, 0, 0
        adx, ady, adz, ac = 0, 0, 0, 0

        if gyro and acc:
            while True:
                try:
                    x, y, z = next(raw)
                    gdx += x
                    gdy += y
                    gdz += z
                    gc += 1
                    x, y, z = next(raw)
                    adx += x
                    ady += y
                    adz += z
                    ac += 1
                except StopIteration:
                    break
        elif gyro:
            for x, y, z in raw:
                gdx += x
                gdy += y
                gdz += z
                gc += 1
        elif acc:
            for x, y, z in raw:
                adx += x
                ady += y
                adz += z
                ac += 1
        else:
            return

        if gyro:
            scaler = Map.gyro_range_map[self.bmi.gyro_range]
            odr = Map.gyro_odr_map[self.bmi.gyro_datarate]
            dx = gdx / scaler
            dy = gdy / scaler
            dz = gdz / scaler
            x, y, z, c = self._accu_gyr
            self._accu_gyr = x+dx, y+dy, z+dz, c+gc
            x, y, z = self._angles
            tx, ty, tz = self._gyr_trk
            x += dx/odr if tx else 0
            y += dy/odr if ty else 0
            z += dz/odr if tz else 0
            self._angles = x, y, z

        if acc:
            scaler = Map.acc_range_map[self.bmi.acc_range]
            dx = adx / scaler
            dy = ady / scaler
            dz = adz / scaler
            x, y, z, c = self._accu_acc
            self._accu_acc = x+dx, y+dy, z+dz, c+ac

    @property
    @override
    #@micropython.native
    def angles(self) -> AxisTuple:
        return AxisTuple(*self._angles)

    @property
    @override
    #@micropython.native
    def gyro(self) -> AxisTuple:
        if self.bmi.fifo_enable_gyro:
            x, y, z, c = self._accu_gyr
            if c != 0:
                self._accu_gyr = 0, 0, 0, 0
                return AxisTuple(x/c, y/c, z/c)
        return self.gyro_inst

    @property
    @override
    def gyro_inst(self) -> AxisTuple:
        s = Map.gyro_range_map[self.bmi.gyro_range]
        x, y, z = self.bmi.gyro
        return AxisTuple(x/s, y/s, z/s)

    @property
    @override
    #@micropython.native
    def accel(self) -> AxisTuple:
        if self.bmi.fifo_enable_acc:
            x, y, z, c = self._accu_acc
            if c != 0:
                self._accu_acc = 0, 0, 0, 0
                return AxisTuple(x/c, y/c, z/c)
        return self.accel_inst

    @property
    @override
    def accel_inst(self) -> AxisTuple:
        s = Map.acc_range_map[self.bmi.acc_range]
        x, y, z = self.bmi.acc
        return AxisTuple(x/s, y/s, z/s)

    @property
    @override
    def motion6(self) -> tuple[float, float, float, float, float, float]:
        g_s = Map.gyro_range_map[self.bmi.gyro_range]
        a_s = Map.acc_range_map[self.bmi.acc_range]
        gx, gy, gz, ax, ay, az = self.bmi.motion6
        return (gx/g_s, gy/g_s, gz/g_s, ax/a_s, ay/a_s, az/a_s)

    @property
    def temperature(self) -> float:
        """
        Sensor temperature in °C
        """
        scaled = self.bmi.temp / (0.5 ** 9)
        return 23 + scaled

    @property
    def use_hw_buffer(self) -> Sensor|None:
        gyro = self.bmi.fifo_enable_gyro
        acc = self.bmi.fifo_enable_acc
        return {
                (True, True): 'both',
                (True, False): 'gyro',
                (False, True): 'accel',
                (False, False): None,
                }.get((gyro, acc))

    @use_hw_buffer.setter
    def use_hw_buffer(self, sensor: Sensor|None) -> None:
        gyro, acc = {
                'both': (True, True),
                'gyro': (True, False),
                'accel': (False, True),
                None: (False, False),
                }.get(sensor, (False, False))
        if (sensor == 'both'
            and self.bmi.acc_datarate != self.bmi.gyro_datarate
            and not self.bmi.fifo_use_headers):
            raise ValueError('Cannot use headerless FIFO with ODR mismatch')
        self.bmi.fifo_enable_gyro = gyro
        self.bmi.fifo_enable_acc = acc

