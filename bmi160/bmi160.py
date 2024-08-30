import machine
import struct
import time
from typing import TYPE_CHECKING

from bmi160.access import Bitfield, Register8U, Register16, SplitRegister
from bmi160.constants import Reg, Bit, Def, Axis, Map
from bmi160.imu import IMU


if TYPE_CHECKING:
    from typing import override
else:
    # micropython's typing stubs don't support the override decorator yet
    override = lambda f: f


_print = False
if not _print:
    print = lambda *_: None


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
    chip_id = Register8U(Reg.CHIP_ID)
    fatal_err: bool = Bitfield(Reg.ERR_REG, Bit.fatal_err)
    error_code: Def.error_code = Bitfield(Reg.ERR_REG, Bit.err_code)

    mag_status: Def.pmu_status = Bitfield(Reg.PMU_STATUS, Bit.mag_pmu_status)
    gyr_status: Def.pmu_status = Bitfield(Reg.PMU_STATUS, Bit.gyr_pmu_status)
    acc_status: Def.pmu_status = Bitfield(Reg.PMU_STATUS, Bit.acc_pmu_status)

    gyro_x = Register16(Reg.GYR_X)
    gyro_y = Register16(Reg.GYR_Y)
    gyro_z = Register16(Reg.GYR_Z)

    acc_x = Register16(Reg.ACC_X)
    acc_y = Register16(Reg.ACC_Y)
    acc_z = Register16(Reg.ACC_Z)

    @property
    def gyro(self) -> tuple[int, int, int]:
        """
        Raw gyro X, Y, Z readings

        Guaranteed to be from the same measurement. No scaling is performed.
        """
        raw = self._read(Reg.GYRO, 6)
        return struct.unpack('<hhh', raw)

    @property
    def acc(self) -> tuple[int, int, int]:
        """
        Raw acceleration X, Y, Z readings

        Guaranteed to be from the same measurement. No scaling is performed.
        """
        raw = self._read(Reg.ACC, 6)
        return struct.unpack('<hhh', raw)

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

    temp = Register16(Reg.TEMP)

    fifo_length = SplitRegister(
            Reg.FIFO_LENGTH_0,
            Reg.FIFO_LENGTH_1,
            Bit.fifo_byte_counter_10_8,
            )

    def fifo(self) -> bytes:
        """
        Raw read of FIFO queue

        Reads all complete frames from the FIFO queue as bytes (according to
        `fifo_length`). Does not perform any unpacking or processing.

        If no frames are available, returns an empty bytes object.
        """
        count = self.fifo_length
        if count == 0:
            return bytes()
        return self._read(Reg.FIFO, count)

    acc_datarate: Def.acc_odr = Bitfield(Reg.ACC_CONF, Bit.acc_odr)
    acc_bandwidth: Def.acc_bwp = Bitfield(Reg.ACC_CONF, Bit.acc_bwp)
    acc_undersample: Def.acc_us = Bitfield(Reg.ACC_CONF, Bit.acc_us)
    acc_range: Def.acc_range = Bitfield(Reg.ACC_RANGE, Bit.acc_range)

    gyro_datarate: Def.gyr_odr = Bitfield(Reg.GYR_CONF, Bit.gyr_odr)
    gyro_bandwidth: Def.gyr_bwp = Bitfield(Reg.GYR_CONF, Bit.gyr_bwp)
    gyro_range: Def.gyr_range = Bitfield(Reg.GYR_RANGE, Bit.gyr_range)

    gyro_fifo_ds = Bitfield(Reg.FIFO_DOWNS, Bit.gyr_fifo_downs)
    gyro_fifo_use_filter: bool = Bitfield(Reg.FIFO_DOWNS, Bit.gyr_fifo_filt_data)
    acc_fifo_ds = Bitfield(Reg.FIFO_DOWNS, Bit.acc_fifo_downs)
    acc_fifo_use_filter: bool = Bitfield(Reg.FIFO_DOWNS, Bit.acc_fifo_filt_data)

    fifo_use_headers: bool = Bitfield(Reg.FIFO_CONFIG_1, Bit.fifo_header_en)
    fifo_enable_gyro: bool = Bitfield(Reg.FIFO_CONFIG_1, Bit.fifo_gyr_en)
    fifo_enable_acc: bool  = Bitfield(Reg.FIFO_CONFIG_1, Bit.fifo_acc_en)

    cal_enable_gyro: bool = Bitfield(Reg.FOC_CONF, Bit.foc_gyr_en)
    _cea_x = Bitfield(Reg.FOC_CONF, Bit.foc_acc_x)
    _cea_y = Bitfield(Reg.FOC_CONF, Bit.foc_acc_y)
    _cea_z = Bitfield(Reg.FOC_CONF, Bit.foc_acc_z)

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
    def cal_enable_accel(self, g_dir: Axis|None):
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

    def command(self, cmd: Def.cmd):
        self._writereg(Reg.CMD, cmd)

    def _read(self, register: Reg|int, size: int) -> bytes:
        raise NotImplementedError

    def _write(self, register: Reg|int, data: bytes):
        raise NotImplementedError

    def _readreg(self, register: Reg) -> int:
        return self._read(register, 1)[0]

    def _writereg(self, register: Reg, value: int):
        return self._write(register, bytes([value]))


class _BMI160_I2C(_BMI160):
    """
    I2C interface to the BMI160
    """
    def __init__(self, i2c: machine.I2C, addr=0x69):
        self.i2c = i2c
        self.addr = addr
        # Should be in I2C mode by default

    def _read(self, register: Reg|int, size: int) -> bytes:
        return self.i2c.readfrom_mem(self.addr, register, size)

    def _write(self, register: Reg|int, data: bytes):
        self.i2c.writeto_mem(self.addr, register, data)


class _BMI160_SPI(_BMI160):
    """
    SPI interface to the BMI160

    Untested. Use at your own risk.
    """
    def __init__(self, spi: machine.SPI, cs: machine.Pin):
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

    def _write(self, register: Reg|int, data: bytes):
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
    def __init__(
            self,
            gyro=True,
            accel=True,
            i2c: machine.I2C|None = None,
            addr=0x69,
            spi: machine.SPI|None = None,
            cs: machine.Pin|None = None,
            ):
        """
        Initialise a BMI160 IMU

        The BMI160 may be connected over either I2C or SPI. If I2C is connected,
        an i2c bus instance should be passed, and the address if different to
        the default. If connected via spi, an SPI bus instance should be passed
        with a separate chip-select pin.

        The gyroscope and accelerometer can be individually enabled or disabled
        with the *gyro* or *accel* parameters.
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
        if accel:
            print('Waiting for accel powerup')
            self.bmi.command(Def.cmd.acc_set_pmu_normal)
            while self.bmi.acc_status != Def.pmu_status.Normal:
                # Excepted 3.2-3.8ms
                time.sleep_ms(1)
        print('Sensors started')

        self._angles = 0, 0, 0
        self.g_track_x = False
        self.g_track_y = False
        self.g_track_z = False

    @override
    def calibrate(self, gyro=True, accel: Axis|None = '-z'):
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
    def gyro_ranges(self):
        return (4000, 2000, 1000, 500, 250)

    @property
    @override
    def gyro_range(self):
        R = Def.gyr_range
        return {
                R.pm2k: 4000,
                R.pm1k: 2000,
                R.pm500: 1000,
                R.pm250: 500,
                R.pm125: 250,
                }.get(self.bmi.gyro_range)

    @gyro_range.setter
    @override
    def gyro_range(self, value: int):
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
    def accel_ranges(self):
        return (32, 16, 8, 4)

    @property
    @override
    def accel_range(self):
        R = Def.acc_range
        return {
                R.pm2: 4,
                R.pm4: 8,
                R.pm8: 16,
                R.pm16: 32,
                }.get(self.bmi.acc_range)

    @accel_range.setter
    @override
    def accel_range(self, value: int):
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
    def gyro_track(self, x=False, y=False, z=False):
        """
        Configure gyroscope angle tracking

        Enables gyroscope data in the hardware FIFO buffer. If the buffer
        overflows, angles will not be accurate.

        Header mode in the FIFO queue is currently not supported; if headers
        are enabled they will be disabled.
        """
        self.g_track_x = x
        self.g_track_y = y
        self.g_track_z = z

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
    def update_angles(self) -> None:
        """
        Update the tracked angle values

        This is only supported when the FIFO buffer has been enabled via
        :meth:`gyro_track()`.
        """
        scaler = Map.gyro_range_map[self.bmi.gyro_range]
        dx, dy, dz = 0, 0, 0
        if not self.bmi.fifo_enable_gyro:
            raise NotImplementedError
            # buffer disabled, using current values
            # this implementation is incorrect. We need to keep track of the
            # time between readings for at least vaguely accurate updates.
            drs = self.bmi.gyro
            dx = drs[0] / scaler if self.g_track_x else 0
            dy = drs[1] / scaler if self.g_track_y else 0
            dz = drs[2] / scaler if self.g_track_z else 0
        else:
            heads = self.bmi.fifo_use_headers
            odr = Map.gyro_odr_map[self.bmi.gyro_datarate]
            fifo = self.bmi.fifo()
            acc_present = self.bmi.fifo_enable_acc

            g_data = []
            if heads:
                raise NotImplementedError('Cannot read gyro data in header mode')
            else:
                g_data = fifo[::2] if acc_present else fifo

            # TODO: are we better off unpacking 6 bytes at once? Should be guaranteed?
            g_data = [struct.unpack('<h', g_data[i:i+2])[0] for i in range(0, len(g_data), 2)]

            if self.g_track_x:
                dx = sum(g_data[::3]) / odr / scaler
            if self.g_track_y:
                dy = sum(g_data[1::3]) / odr / scaler
            if self.g_track_z:
                dz = sum(g_data[2::3]) / odr / scaler

        x, y, z = self._angles
        self._angles = x + dx, y + dy, z + dz

    @property
    @override
    def angles(self) -> tuple[float, float, float]:
        return self._angles

    @property
    @override
    def gyro(self):
        s = Map.gyro_range_map[self.bmi.gyro_range]
        x, y, z = self.bmi.gyro
        return (x/s, y/s, z/s)

    @property
    def temperature(self):
        """
        Sensor temperature in °C
        """
        scaled = self.bmi.temp / (0.5 ** 9)
        return 23 + scaled

