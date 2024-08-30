from bmi160.constants import Axis


class IMU:
    def __init__(self, gyro=True, accel=True):
        raise NotImplementedError

    def calibrate(self, gyro=True, accel: Axis|None = '-z'):
        raise NotImplementedError

    @property
    def gyro_ranges(self) -> tuple[int]:
        raise NotImplementedError

    @property
    def gyro_range(self) -> int:
        raise NotImplementedError

    @gyro_range.setter
    def gyro_range(self, range: int):
        raise NotImplementedError

    @property
    def accel_ranges(self) -> tuple[int]:
        raise NotImplementedError

    @property
    def accel_range(self) -> int:
        raise NotImplementedError

    @accel_range.setter
    def accel_range(self, range: int):
        raise NotImplementedError

    def gyro_track(self, x=False, y=False, z=False) -> None:
        raise NotImplementedError

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
        """
        raise NotImplementedError

    @property
    def angles(self) -> tuple[float, float, float]:
        raise NotImplementedError

    @property
    def gyro(self) -> tuple[float, float, float]:
        raise NotImplementedError

    @property
    def accel(self) -> tuple[float, float, float]:
        raise NotImplementedError

    @property
    def motion6(self) -> tuple[float, float, float]:
        raise NotImplementedError

