from typing import TYPE_CHECKING, Literal


if TYPE_CHECKING:
    from enum import IntEnum, Enum
else:
    # micropython doesn't have enum support
    IntEnum = object
    Enum = object


Axis = Literal['+x','+y','+z','-x','-y','-z']


class Reg(IntEnum):
    """
    Register addresses

    Enumeration of all available registers and their addresses. Registers are
    named per the datasheet; most 2-byte registers have an alias with a more
    readable name - e.g. DATA_8 is aliased to GYR_X for the gyro x value.
    """
    CHIP_ID = const(0x00)
    ERR_REG = const(0x02)
    PMU_STATUS = const(0x03)

    DATA = const(0x04)

    MAG_X = const(0x04)
    DATA_0 = const(0x04)
    DATA_1 = const(0x05)
    MAG_Y = const(0x06)
    DATA_2 = const(0x06)
    DATA_3 = const(0x07)
    MAG_Z = const(0x08)
    DATA_4 = const(0x08)
    DATA_5 = const(0x09)
    HALL = const(0x0A)
    DATA_6 = const(0x0A)
    DATA_7 = const(0x0B)
    GYRO = const(0x0C)
    GYR_X = const(0x0C)
    DATA_8 = const(0x0C)
    DATA_9 = const(0x0D)
    GYR_Y = const(0x0E)
    DATA_10 = const(0x0E)
    DATA_11 = const(0x0F)
    GYR_Z = const(0x10)
    DATA_12 = const(0x10)
    DATA_13 = const(0x11)
    ACC = const(0x12)
    ACC_X = const(0x12)
    DATA_14 = const(0x12)
    DATA_15 = const(0x13)
    ACC_Y = const(0x14)
    DATA_16 = const(0x14)
    DATA_17 = const(0x14)
    ACC_Z = const(0x16)
    DATA_18 = const(0x16)
    DATA_19 = const(0x17)

    SENSORTIME = const(0x18)
    SENSORTIME_0 = const(0x18)
    SENSORTIME_1 = const(0x19)
    SENSORTIME_2 = const(0x20)

    STATUS = const(0x1B)
    INT_STATUS_0 = const(0x1C)
    INT_STATUS_1 = const(0x1D)
    INT_STATUS_2 = const(0x1E)
    INT_STATUS_3 = const(0x1F)

    TEMP = const(0x20)
    TEMPERATURE_0 = const(0x20)
    TEMPERATURE_1 = const(0x21)

    FIFO_LENGTH = const(0x22)
    FIFO_LENGTH_0 = const(0x22)
    FIFO_LENGTH_1 = const(0x23)
    FIFO = const(0x24)

    ACC_CONF = const(0x40)
    ACC_RANGE = const(0x41)
    GYR_CONF = const(0x42)
    GYR_RANGE = const(0x43)
    MAG_CONF = const(0x44)

    FIFO_DOWNS = const(0x45)
    FIFO_CONFIG_0 = const(0x46)
    FIFO_CONFIG_1 = const(0x47)

    MAG_IF_0 = const(0x4B)
    MAG_IF_1 = const(0x4C)
    MAG_IF_2 = const(0x4D)
    MAG_IF_3 = const(0x4E)
    MAG_IF_4 = const(0x4F)

    INT_EN_0 = const(0x50)
    INT_EN_1 = const(0x51)
    INT_EN_2 = const(0x52)
    INT_OUT_CTRL = const(0x53)
    INT_LATCH = const(0x54)
    INT_MAP_0 = const(0x55)
    INT_MAP_1 = const(0x56)
    INT_MAP_2 = const(0x57)
    INT_DATA_0 = const(0x58)
    INT_DATA_1 = const(0x59)
    INT_LOWHIGH_0 = const(0x5A)
    INT_LOWHIGH_1 = const(0x5B)
    INT_LOWHIGH_2 = const(0x5C)
    INT_LOWHIGH_3 = const(0x5D)
    INT_LOWHIGH_4 = const(0x5E)
    INT_MOTION_0 = const(0x5F)
    INT_MOTION_1 = const(0x60)
    INT_MOTION_2 = const(0x61)
    INT_MOTION_3 = const(0x62)
    INT_TAP_0 = const(0x63)
    INT_TAP_1 = const(0x64)
    INT_ORIENT_0 = const(0x65)
    INT_ORIENT_1 = const(0x66)
    INT_FLAT_0 = const(0x67)
    INT_FLAT_1 = const(0x68)

    FOC_CONF = const(0x69)
    CONF = const(0x6A)
    IF_CONF = const(0x6B)
    PMU_TRIGGER = const(0x6C)
    SELF_TEST = const(0x6D)
    NV_CONF = const(0x70)

    OFFSET_0 = const(0x71)
    OFF_ACC_X = const(0x71)
    OFFSET_1 = const(0x72)
    OFF_ACC_Y = const(0x72)
    OFFSET_2 = const(0x73)
    OFF_ACC_Z = const(0x73)
    OFFSET_3 = const(0x74)
    OFF_GYR_X_L = const(0x74)
    OFFSET_4 = const(0x75)
    OFF_GYR_Y_L = const(0x75)
    OFFSET_5 = const(0x76)
    OFF_GYR_Z_L = const(0x76)
    OFFSET_6 = const(0x77)

    STEP_CNT = const(0x78)
    STEP_CNT_0 = const(0x78)
    STEP_CNT_1 = const(0x79)
    STEP_CONF_0 = const(0x7A)
    STEP_CONF_1 = const(0x7B)

    CMD = const(0x7E)


class Bit(Enum):
    """
    Bit fields within registers

    Fields are named as per the datasheet. Single-bit values are stored as the
    bit position; multi-bit values are stored as a 2-tuple with the start and
    end bits (inclusive).
    """
    ## ERR_REG
    fatal_err = const(0)
    err_code = const((1, 4))
    i2c_fail_err = const(5)
    drop_cmd_err = const(6)
    mag_drdy_err = const(7)

    ## PMU_STATUS
    mag_pmu_status = const((0, 1))
    gyr_pmu_status = const((2, 3))
    acc_pmu_status = const((4, 5))

    ## STATUS
    gyr_self_test_ok = const(1)
    mag_man_op = const(2)
    foc_rdy = const(3)
    nvm_rdy = const(4)
    drdy_mag = const(5)
    drdy_gyr = const(6)
    drdy_acc = const(7)

    # INT_STATUS_0
    step_int = const(0)
    signmot_int = const(1)
    anym_int = const(2)
    pmu_trigger_int = const(3)
    d_tap_int = const(4)
    s_tap_int = const(5)
    orient_int = const(6)
    flat_int = const(7)

    ## INT_STATUS_1
    highg_int = const(2)
    lowg_int = const(3)
    drdy_int = const(4)
    ffull_int = const(5)
    fwm_int = const(6)
    nomo_int = const(7)

    ## INT_STATUS_2
    anym_first_x = const(0)
    anym_first_y = const(1)
    anym_first_z = const(2)
    anym_sign = const(3)
    tap_first_x = const(4)
    tap_first_y = const(5)
    tap_first_z = const(6)
    tap_sign = const(7)

    ## INT_STATUS_3
    high_first_x = const(0)
    high_first_y = const(1)
    high_first_z = const(2)
    high_sign = const(3)
    orient_1_0 = const((4, 5))
    orient_2 = const(6)
    flat = const(7)

    ## FIFO_LENGTH_1
    fifo_byte_counter_10_8 = const((0, 2))

    ## Range & Config
    acc_odr = const((0, 3))
    acc_bwp = const((4, 6))
    acc_us = const(7)
    acc_range = const((0, 3))
    gyr_odr = const((0, 3))
    gyr_bwp = const((4, 5))
    gyr_range = const((0, 2))
    mag_odr = const((0, 3))

    ## FIFO_DOWNS
    gyr_fifo_downs = const((0, 2))
    gyr_fifo_filt_data = const(3)
    acc_fifo_downs = const((4, 6))
    acc_fifo_filt_data = const(7)

    ## FIFO_CONFIG_1
    fifo_time_en = const(1)
    fifo_tag_int2_en = const(2)
    fifo_tag_int1_en = const(3)
    fifo_header_en = const(4)
    fifo_mag_en = const(5)
    fifo_acc_en = const(6)
    fifo_gyr_en = const(7)

    ## MAG_IF_0, MAG_IF_1
    i2c_device_addr = const((1, 7))
    mag_rd_burst = const((0, 1))
    mag_offset = const((3, 5))
    mag_manual_en = const(7)

    ## Interrupts
    # There's heaps and I don't want to add them all. Feel free to copy them
    # in yourself if you need them.

    ## Config (FOC_CONF, CONF, IF_CONF)
    foc_acc_z = const((0, 1))
    foc_acc_y = const((2, 3))
    foc_acc_x = const((4, 5))
    foc_gyr_en = const(6)
    nvm_prog_en = const(1)
    spi3 = const(0)
    if_mode = const((4, 5))

    ## PMU_TRIGGER
    gyr_sleep_trigger = const((0, 2))
    gyr_wakeup_trigger = const((3, 4))
    gyr_sleep_state = const(5)
    wakeup_int = const(6)

    ## SELF_TEST
    acc_self_test_enable = const((0, 1))
    acc_self_test_sign = const(2)
    acc_self_test_amp = const(3)
    gyr_self_test_enable = const(4)

    ## NV_CONF
    spi_en = const(0)
    i2c_wdt_sel = const(1)
    i2c_wdt_en = const(2)
    u_spare_0 = const(3)

    ## OFFSET_6
    off_gyr_x_9_8 = const((0, 1))
    off_gyr_y_9_8 = const((2, 3))
    off_gyr_z_9_8 = const((4, 5))
    acc_off_en = const(6)
    gyr_off_en = const(7)

    ## STEP_CONF_1
    step_conf_10_8 = const((0, 2))
    step_cnt_en = const(3)


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
        NoError = const(0b0000)
        Error1 = const(0b0001)
        Error2 = const(0b0010)
        LPIFilter = const(0b0011)
        ODRMismatch = const(0b0110)
        LPFilter = const(0b0111)

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
        pm2 = const(0b0011)
        pm4 = const(0b0101)
        pm8 = const(0b1000)
        pm16 = const(0b1100)

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
        pm2k = const(0b000)
        pm1k = const(0b001)
        pm500 = const(0b010)
        pm250 = const(0b011)
        pm125 = const(0b100)

    class cmd(IntEnum):
        start_foc = const(0x03)
        acc_set_pmu_normal = const(0x11)
        acc_set_pmu_low = const(0x12)
        gyr_set_pmu_normal = const(0x15)
        gyr_set_pmu_fast = const(0x17)
        prog_nvm = const(0xA0)
        fifo_flush = const(0xB0)
        int_reset = const(0xB1)
        softreset = const(0xB6)
        step_cnt_clr = const(0xB2)


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

