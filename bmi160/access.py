import struct

from .constants import Register, Field


class Bitfield:
    def __init__(self, register: Register, field: Field):
        if isinstance(field, int):
            start = field
            width = 1
        elif isinstance(field, tuple):
            start, width = field
        else:
            raise TypeError('Unknown field! Got {}'.format(field))

        self.mask = ((1 << width) - 1) << start
        self.register = register
        self.start = start
        self.width = width

    def __get__(self, obj, objtype=None) -> int:
        val: int = obj._readreg(self.register)
        val = (val & self.mask) >> self.start
        return val

    def __set__(self, obj, value: int):
        stale: int = obj._readreg(self.register)
        stale &= ~self.mask

        value <<= self.start
        value &= self.mask
        value |= stale

        obj._writereg(self.register, value)


class BytesRegister:
    def __init__(self, reg: Register, fmt: str):
        self.fmt = fmt
        self.register = reg
        self.width = struct.calcsize(fmt)

    def __get__(self, obj, objtype=None) -> tuple:
        val: bytes = obj._read(self.register, self.width)
        return struct.unpack(self.fmt, val)

    def __set__(self, obj, value: tuple):
        data = struct.pack(self.fmt, *value)
        obj._write(self.register, data)


class UCharRegister(BytesRegister):
    def __init__(self, reg: Register):
        super().__init__(reg, '<C')

    def __get__(self, obj, objtype=None) -> int:
        return super().__get__(obj, objtype)[0]

    def __set__(self, obj, value: int):
        return super().__set__(obj, (value,))


class ShortRegister(BytesRegister):
    def __init__(self, reg: Register):
        super().__init__(reg, '<h')

    def __get__(self, obj, objtype=None) -> int:
        return super().__get__(obj, objtype)[0]

    def __set__(self, obj, value: int):
        return super().__set__(obj, (value,))


class SplitRegister:
    def __init__(self, reg_a: Register, reg_b: Register, field: Field):
        self.lower = UCharRegister(reg_a)
        self.upper = Bitfield(reg_b, field)

    def __get__(self, obj, objtype=None) -> int:
        return (self.lower.__get__(obj, objtype)
                | (self.upper.__get__(obj, objtype) << 8))

    def __set__(self, obj, value: int):
        self.lower.__set__(obj, value & 0xFF)
        self.upper.__set__(obj, value >> 8)

