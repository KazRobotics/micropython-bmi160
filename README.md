# MicroPython BMI160 Driver

MicroPython driver for the Bosch BMI160 6-axis IMU.

Inspired by Jose D. Montoya's [BMI160](https://github.com/jposada202020/MicroPython_BMI160)
library.

## Development

To generate typing stubs, `stubgen` from `mypy` may be used:

```sh
stubgen --include-docstrings imu
```

Be sure to check the stub files. Any type aliases (`Axis`, `Axes`, etc) will
need to be manually replaced.
