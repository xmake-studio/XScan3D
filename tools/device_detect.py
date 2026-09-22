#!/usr/bin/env python3
"""Find the scanner among the serial ports without opening any of them.

Several RP2040 boards may be plugged in at once and they all enumerate as the
same 2E8A:000A "Pico". Opening a port to ask "are you the scanner?" is not an
option: asserting DTR wakes the other board's firmware, a stray byte may be a
command to it, and a 1200 baud open reboots it into the bootloader. So the
scanner is recognised purely from its USB descriptors:

  * the firmware sets its USB product string to PRODUCT (see platformio.ini,
    board_build.arduino.earlephilhower.usb_product). Stock boards say "Pico".
  * as a fallback, the chip serial number of a board that has already proven
    itself (sent a config record over a live link) is remembered by the UI, so
    a scanner still running older firmware is found too.

On Windows pyserial does not report the product string (the port node is the
CDC interface, whose name is "Pico Serial"), so it is read from the parent USB
device node through cfgmgr32. Linux and macOS get it from pyserial directly.
"""

import sys

from serial.tools import list_ports

PRODUCT = "XScan3D"
RP2040_VID = 0x2E8A


class PortInfo:
    __slots__ = ("device", "description", "vid", "pid", "serial", "product")

    def __init__(self, p, product):
        self.device = p.device
        self.description = p.description
        self.vid = p.vid
        self.pid = p.pid
        self.serial = p.serial_number
        self.product = product

    @property
    def is_rp2040(self):
        return self.vid == RP2040_VID

    @property
    def is_scanner_product(self):
        return bool(self.product) and self.product.strip().lower() \
            == PRODUCT.lower()


# --- Windows: product string from the USB device node -----------------------

_win_product = None

if sys.platform == "win32":
    import ctypes
    from ctypes import wintypes

    class _GUID(ctypes.Structure):
        _fields_ = [("Data1", wintypes.DWORD), ("Data2", wintypes.WORD),
                    ("Data3", wintypes.WORD), ("Data4", ctypes.c_ubyte * 8)]

    class _DEVPROPKEY(ctypes.Structure):
        _fields_ = [("fmtid", _GUID), ("pid", wintypes.ULONG)]

    # DEVPKEY_Device_BusReportedDeviceDesc: the iProduct string as the device
    # itself reported it, not the driver's friendly name.
    _BUS_DESC = _DEVPROPKEY(
        _GUID(0x540B947E, 0x8B40, 0x45BC,
              (ctypes.c_ubyte * 8)(0xA8, 0xA2, 0x6A, 0x0B,
                                   0x89, 0x4C, 0xBD, 0xA2)), 4)
    _DEVPROP_TYPE_STRING = 0x12
    _CR_SUCCESS = 0

    try:
        _cfg = ctypes.WinDLL("cfgmgr32")
        _locate = _cfg.CM_Locate_DevNodeW
        _locate.argtypes = [ctypes.POINTER(wintypes.DWORD), wintypes.LPCWSTR,
                            wintypes.ULONG]
        _locate.restype = wintypes.DWORD
        _getprop = _cfg.CM_Get_DevNode_PropertyW
        _getprop.argtypes = [wintypes.DWORD, ctypes.POINTER(_DEVPROPKEY),
                             ctypes.POINTER(wintypes.ULONG), ctypes.c_void_p,
                             ctypes.POINTER(wintypes.ULONG), wintypes.ULONG]
        _getprop.restype = wintypes.DWORD
    except (OSError, AttributeError):
        _cfg = None

    def _win_product(p):
        # A USB device with a serial number is instanced by it, so the parent
        # (composite) device's id can be built without walking the tree. For a
        # non-composite device this is the port node itself -- same answer.
        if _cfg is None or p.vid is None or not p.serial_number:
            return None
        inst = f"USB\\VID_{p.vid:04X}&PID_{p.pid:04X}\\{p.serial_number}"
        node = wintypes.DWORD()
        if _locate(ctypes.byref(node), inst, 0) != _CR_SUCCESS:
            return None
        ptype = wintypes.ULONG()
        buf = ctypes.create_unicode_buffer(256)
        size = wintypes.ULONG(ctypes.sizeof(buf))
        if _getprop(node, ctypes.byref(_BUS_DESC), ctypes.byref(ptype), buf,
                    ctypes.byref(size), 0) != _CR_SUCCESS:
            return None
        if ptype.value != _DEVPROP_TYPE_STRING:
            return None
        return buf.value or None


def _product(p):
    if p.vid != RP2040_VID:
        return p.product
    if _win_product is not None:
        try:
            return _win_product(p) or p.product
        except Exception:
            return p.product
    return p.product


def scan_ports():
    """Every serial port, with the USB product string where it can be read."""
    return [PortInfo(p, _product(p)) for p in list_ports.comports()]


def find_scanner(ports, known_serials=()):
    """The ports that are this scanner, best match first.

    A matching product string wins. A remembered serial number counts only
    when the product string says nothing either way ("Pico" is what both an
    old scanner firmware and any other board report, so it cannot rule the
    board out -- the UI drops the link if it never sends a config record).
    """
    known = set(known_serials or ())
    by_product = [p for p in ports if p.is_rp2040 and p.is_scanner_product]
    by_serial = [p for p in ports if p.is_rp2040 and not p.is_scanner_product
                 and p.serial and p.serial in known]
    return by_product + by_serial


if __name__ == "__main__":
    ports = scan_ports()
    hits = {p.device for p in find_scanner(ports)}
    for p in ports:
        vidpid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "----:----"
        mark = " <- scanner" if p.device in hits else ""
        print(f"{p.device:8} {vidpid} ser={p.serial} product={p.product!r}"
              f"{mark}")
