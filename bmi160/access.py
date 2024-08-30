import struct

from bmi160.constants import Reg, Bit


class Bitfield:
    def __init__(self, register: Reg, field: Bit):
        if isinstance(field, int):
            start = field
            width = 1
        elif isinstance(field, tuple):
            start = field[0]
            width = field[1] - start + 1
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
    def __init__(self, reg: Reg):
        super().__init__(reg, '<B')

    def __get__(self, obj, objtype=None) -> int:
        return super().__get__(obj, objtype)[0]

    def __set__(self, obj, value: int):
        return super().__set__(obj, (value,))


class Register16(BytesRegister):
    def __init__(self, reg: Reg):
        super().__init__(reg, '<h')

    def __get__(self, obj, objtype=None) -> int:
        return super().__get__(obj, objtype)[0]

    def __set__(self, obj, value: int):
        return super().__set__(obj, (value,))


class SplitRegister:
    def __init__(self, reg_a: Reg, reg_b: Reg, field: Bit):
        self.lower = Register8U(reg_a)
        self.upper = Bitfield(reg_b, field)

    def __get__(self, obj, objtype=None) -> int:
        return (self.lower.__get__(obj, objtype)
                | (self.upper.__get__(obj, objtype) << 8))

    def __set__(self, obj, value: int):
        self.lower.__set__(obj, value & 0xFF)
        self.upper.__set__(obj, value >> 8)

