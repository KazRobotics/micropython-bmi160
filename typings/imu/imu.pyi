from typing import Literal

Axis = Literal['+x','+y','+z','-x','-y','-z']
Axes = Literal['x', 'y', 'z', 'xy', 'xz', 'yz', 'xyz']

class IMU:
    """
    Generic API to a 6-axis IMU

    Concrete implementations of IMU interfaces should inherit from this class
    and implement methods if possible.

    .. todo:: Consider implementing more positioning algorithms? In the meantime
    a secondary library such as micropython-fusion may be used.
    """
    def __init__(self, gyro: bool = True, accel: bool = True) -> None:
        """
        Create a new 6-axis IMU

        If supported by the IMU, the gyroscope and accelerometer may be
        individually enabled and disabled using the *gyro* and *accel*
        parameters.
        """
    def calibrate(self, gyro: bool = True, accel: Axis | None = '-z') -> None:
        """
        Calibrate the IMU

        The IMU should be in a resting position with one axis level during
        calibration. This method blocks until calibration is complete.

        If *gyro* is True, the gyro is calibrated to 0°/s in all axes.

        If *accel* is not None, the accelerometer is calibrated to 1g on the
        specified axis. This may be specified as +/- x, y, or z, depending on
        the orientation of the accelerometer.
        """
    @property
    def gyro_ranges(self) -> tuple[int]:
        """
        Valid gyroscope sensor ranges, in °/s
        """
    @property
    def gyro_range(self) -> int:
        """
        Current gyroscope sensor range, in °/s
        """
    @gyro_range.setter
    def gyro_range(self, range: int) -> None:
        """
        Set the gyroscope sensor range

        Measured in °/s. Must be a value from `gyro_ranges`.
        """
    @property
    def accel_ranges(self) -> tuple[int]:
        """
        Valid accelerometer sensor ranges, in m/s2
        """
    @property
    def accel_range(self) -> int:
        """
        Current accelerometer sensor range, in m/s2
        """
    @accel_range.setter
    def accel_range(self, range: int) -> None:
        """
        Set the accelerometer sensor range

        Measured in m/s2. Must be a value from `accel_ranges`.
        """
    def track_angles(self, axes: Axes | None = None) -> None:
        """
        Enable or disable gyroscope angle tracking

        Enables or disables gyroscope-based angle tracking for each axis. Note
        that angle tracking does not provide absolute pitch/roll/yaw
        measurements; tracking only monitors relative in-plane rotation for
        each axis.

        Angles are reset to zero during calibration.
        """
    def update(self) -> None:
        """
        Update the IMU state

        Makes a best-effort attempt to update the angle states since the last
        call. If buffering is available at either a hardware or software level,
        this method will attempt to use the cumulative angle change according
        to the buffer since the last call to this method. Note that if the
        buffer has overflown since the last call this will not be accurate.

        If buffering is not available, this method will use the current
        gyroscope reading.

        This method should be polled frequently to ensure data accuracy.
        """
    def update_async(self, hz: int = 50) -> None:
        """
        Start an asyncio task to update the IMU state

        If asyncio is available and in use, this method may be used to start a
        background task to update the IMU state via asyncio. If this is used,
        :meth:`update()` does not need to be explicitly polled.
        """
    @property
    def angles(self) -> tuple[float, float, float]:
        """
        Cumulative angular movement since last calibration

        Provides the total angular movement for each axis since the previous
        calibration, in degrees. Only axes where tracking is enabled via
        :meth:`gyro_track()` will return non-zero values.

        This method relies on :meth:`update_angles()` being called sufficiently
        frequently for the angle to be accurate.

        Returns x, y, z in degrees.
        """
    @property
    def gyro(self) -> tuple[float, float, float]:
        """
        Accurate x, y, z gyroscope readings, in °/s

        If data buffering is available, provides accumulated gyro readings
        since the last read; this is typically more accurate than the
        `gyro_inst` property. This depends on :meth:`update()` being polled
        frequently, or the asyncio task running.

        If data buffering is not available, this is identical to `gyro_inst`.
        """
    @property
    def gyro_inst(self) -> tuple[float, float, float]:
        """
        Instantaneous x, y, z gyroscope readings, in °/s

        Guaranteed to be from the same sensor reading.
        """
    @property
    def accel(self) -> tuple[float, float, float]:
        """
        Accurate x, y, z accelerometer readings, in m/s2

        If data buffering is available, provides accumulated gyro readings
        since the last read; this is typically more accurate than the
        `gyro_inst` property. This depends on :meth:`update()` being polled
        frequently, or the asyncio task running.

        Guaranteed to be from the same sensor reading.

        If data buffering is not available, this is identical to `accel_inst`.
        """
    @property
    def accel_inst(self) -> tuple[float, float, float]:
        """
        Instantaneous x, y, z accelerometer readings, in m/s2

        Guaranteed to be from the same sensor reading.
        """
    @property
    def motion6(self) -> tuple[float, float, float, float, float, float]:
        """
        Instantaneous combined gyroscope and accelerometer readings

        Returns the gyroscope x, y, z readings (°/s) followed by the
        accelerometer x, y, z readings (m/s2). Guaranteed to be from the same
        sensor reading.

        Note that this is implemented to allow the guarantee to hold; this is
        less relevant for accumulated data, hence an accumulated accessor is
        not implemented.
        """
