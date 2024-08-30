import struct

from bmi160.constants import Reg, Bit


class Bitfield:
    """
    Bit-field access descriptor

    Provides access to one or more bits within a one-byte register. Expects the
    owner object to implement `_readreg` and `_writereg` to read and write a
    single register respectively.
    """
    def __init__(self, register: Reg, field: Bit):
        """
        Create a new bitfield descriptor

        *field* should be either an integer from 0 to 7, for single bit, or a
        2-tuple providing [start, end] of the bit field (inclusive). *register*
        is the address of the memory register containing the relevant data.
        """
        if isinstance(field, int):
            start = field
            width = 1
        elif isinstance(field, tuple):
            start = field[0]
            # inclusive, so add 1
            width = field[1] - start + 1
        else:
            raise TypeError('Unknown field! Got {}'.format(field))

        self.mask = ((1 << width) - 1) << start
        self.register = register
        self.start = start
        self.width = width

    def __get__(self, obj, objtype=None) -> int:
        val: int = obj._readreg(self.register)
        # mask out irrelevant data and shift to 0
        val = (val & self.mask) >> self.start
        return val

    def __set__(self, obj, value: int):
        stale: int = obj._readreg(self.register)
        stale &= ~self.mask # remove any data in destination

        value <<= self.start # offset
        # remove any data in value outside our field, to make sure incorrect
        # values don't interfere with other fields
        value &= self.mask
        value |= stale

        obj._writereg(self.register, value)


class BytesRegister:
    """
    Bulk memory register access descriptor

    Provides access to one or more complete registers. Given a struct format
    string *fmt*, provides a getter and a setter to unpack and pack a tuple into
    the register at *reg*.
    """
    def __init__(self, reg: Reg, fmt: str):
        self.fmt = fmt
        self.register = reg
        self.width = struct.calcsize(fmt)

    def __get__(self, obj, objtype=None) -> tuple:
        val: bytes = obj._read(self.register, self.width)
        return struct.unpack(self.fmt, val)

    def __set__(self, obj, value: tuple):
        data = struct.pack(self.fmt, *value)
        obj._write(self.register, data)


class Register8U(BytesRegister):
    """
    Unsigned byte register descriptor

    Provides a getter and setter to a 1-byte register with an unsigned value.
    """
    def __init__(self, reg: Reg):
        super().__init__(reg, '<B')

    def __get__(self, obj, objtype=None) -> int:
        return super().__get__(obj, objtype)[0]

    def __set__(self, obj, value: int):
        return super().__set__(obj, (value,))


class Register16(BytesRegister):
    """
    Signed 16-bit register descriptor

    Provides a getter and setter to a 2-byte register with a single signed
    value.
    """
    def __init__(self, reg: Reg):
        super().__init__(reg, '<h')

    def __get__(self, obj, objtype=None) -> int:
        return super().__get__(obj, objtype)[0]

    def __set__(self, obj, value: int):
        return super().__set__(obj, (value,))


class SplitRegister:
    """
    9-15 bit register descriptor

    Some devices may store a value between 9 and 15 bits with the lower 8 bits
    in one register, and the upper bits in a section of a second register. This
    descriptor provides a getter and setter access these values.
    """
    def __init__(self, reg_a: Reg, reg_b: Reg, field: Bit):
        """
        Create a new split-value descriptor

        *reg_a* should contain the lower 8 bits of the value, and *reg_b* should
        contain the upper bits as described by *field* (see :class:`Bitfield`).
        """
        # This is somewhat annoying to do with direct access, so just delegate
        # the desciptors we already have
        self.lower = Register8U(reg_a)
        self.upper = Bitfield(reg_b, field)

    def __get__(self, obj, objtype=None) -> int:
        return (self.lower.__get__(obj, objtype)
                | (self.upper.__get__(obj, objtype) << 8))

    def __set__(self, obj, value: int):
        self.lower.__set__(obj, value & 0xFF)
        self.upper.__set__(obj, value >> 8)

