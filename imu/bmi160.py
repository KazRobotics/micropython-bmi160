import builtins
import machine
import micropython
import struct
import time
from typing import TYPE_CHECKING, Literal

from imu.access import Bitfield, Register8U, Register16, SplitRegister, Contiguous10
from imu.constants import Reg, Bit, Def, Axis, Axes, Map
from imu.imu import IMU


if TYPE_CHECKING:
    from typing import override

    from typing import NamedTuple
    class AxisTuple(NamedTuple):
        x: float
        y: float
        z: float
else:
    # micropython's typing stubs don't support the override decorator yet
    override = lambda f: f
    from collections import namedtuple
    AxisTuple = namedtuple('AxisTuple', ('x', 'y', 'z'))


_print = False
if not _print:
    print = lambda *_: None


Sensor = Literal['gyro', 'accel', 'both']


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

    @micropython.native
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

    @micropython.native
    def _readreg(self, register: Reg) -> int:
        return self._read(register, 1)[0]

    @micropython.native
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

    @micropython.native
    def _read(self, register: Reg|int, size: int) -> bytes:
        return self.i2c.readfrom_mem(self.addr, register, size)

    @micropython.native
    def _read_into(self, register: Reg|int, buf: bytes|memoryview) -> None:
        self.i2c.readfrom_mem_into(self.addr, register, buf)

    @micropython.native
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
    @micropython.native
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
    @micropython.native
    def angles(self) -> AxisTuple:
        return AxisTuple(*self._angles)

    @property
    @override
    @micropython.native
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
    @micropython.native
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

