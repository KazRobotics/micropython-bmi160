from _typeshed import Incomplete
from imu.constants import Bit as Bit, Reg as Reg

class Bitfield:
    """
    Bit-field access descriptor

    Provides access to one or more bits within a one-byte register. Expects the
    owner object to implement `_readreg` and `_writereg` to read and write a
    single register respectively.
    """
    mask: Incomplete
    register: Incomplete
    start: Incomplete
    width: Incomplete
    def __init__(self, register: Reg, field: Bit, cache: bool = False) -> None:
        """
        Create a new bitfield descriptor

        *field* should be either an integer from 0 to 7, for single bit, or a
        2-tuple providing [start, end] of the bit field (inclusive). *register*
        is the address of the memory register containing the relevant data.
        """
    def __get__(self, obj, objtype: Incomplete | None = None) -> int: ...
    def __set__(self, obj, value: int): ...

class BytesRegister:
    """
    Bulk memory register access descriptor

    Provides access to one or more complete registers. Given a struct format
    string *fmt*, provides a getter and a setter to unpack and pack a tuple into
    the register at *reg*.
    """
    fmt: Incomplete
    register: Incomplete
    width: Incomplete
    def __init__(self, reg: Reg, fmt: str) -> None: ...
    def __get__(self, obj, objtype: Incomplete | None = None) -> tuple: ...
    def __set__(self, obj, value: tuple): ...

class Register8U(BytesRegister):
    """
    Unsigned byte register descriptor

    Provides a getter and setter to a 1-byte register with an unsigned value.
    """
    def __init__(self, reg: Reg) -> None: ...
    def __get__(self, obj, objtype: Incomplete | None = None) -> int: ...
    def __set__(self, obj, value: int): ...

class Register16(BytesRegister):
    """
    Signed 16-bit register descriptor

    Provides a getter and setter to a 2-byte register with a single signed
    value.
    """
    def __init__(self, reg: Reg) -> None: ...
    def __get__(self, obj, objtype: Incomplete | None = None) -> int: ...
    def __set__(self, obj, value: int): ...

class Contiguous10:
    def __init__(self, low_reg: Reg) -> None: ...
    def __get__(self, obj, objtype: Incomplete | None = None) -> int: ...

class SplitRegister:
    """
    9-15 bit register descriptor

    Some devices may store a value between 9 and 15 bits with the lower 8 bits
    in one register, and the upper bits in a section of a second register. This
    descriptor provides a getter and setter access these values.
    """
    lower: Incomplete
    upper: Incomplete
    def __init__(self, reg_a: Reg, reg_b: Reg, field: Bit) -> None:
        """
        Create a new split-value descriptor

        *reg_a* should contain the lower 8 bits of the value, and *reg_b* should
        contain the upper bits as described by *field* (see :class:`Bitfield`).
        """
    def __get__(self, obj, objtype: Incomplete | None = None) -> int: ...
    def __set__(self, obj, value: int): ...
