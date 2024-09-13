import machine
from imu.access import Bitfield as Bitfield, Register16 as Register16, Register8U as Register8U, SplitRegister as SplitRegister
from imu.constants import Axes as Axes, Axis as Axis, Bit as Bit, Def as Def, Map as Map, Reg as Reg
from imu.imu import IMU as IMU
from typing import TypeAlias, Literal, NamedTuple

Sensor: TypeAlias = Literal['gyro', 'accel', 'both']
class AxisTuple(NamedTuple):
    x: float
    y: float
    z: float


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
    def fifo(self) -> bytes:
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
