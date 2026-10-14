# SEN0776 Gravity 九轴光学里程计，I2C 主机库。寄存器语义与 Arduino 库一致。

import struct

PID = 0x0776
DEFAULT_ADDR = 0x11


class DFRobot_9DofODO_I2C(object):
    def __init__(self, bus, addr=DEFAULT_ADDR):
        self._bus = bus
        self._addr = addr
        self._ready = False

    def begin(self):
        pid = self._read_u16(0x0000)
        self._ready = pid == PID
        return self._ready

    def get_paa_id(self):
        return self._read_u16(0x0006)

    def get_paa_revision_id(self):
        return self._read_u16(0x0007)

    def get_total_displacement(self):
        return self._read_f32(0x000A), self._read_f32(0x000C)

    def reset_total_displacement(self):
        self._write_u16(0x0100, 1)

    def set_calibration_scale(self, scale_x, scale_y):
        self._write_f32(0x0101, scale_x)
        self._write_f32(0x0103, scale_y)

    def set_cpi(self, cpi):
        if cpi < 1 or cpi > 200:
            raise ValueError("cpi 1..200")
        self._write_u16(0x0105, cpi)

    def read_squal(self):
        return self._read_u16(0x000E) & 0xFF

    def set_orientation(self, swap_xy, invert_x, invert_y):
        value = (1 if swap_xy else 0) | (2 if invert_x else 0) | (4 if invert_y else 0)
        self._write_u16(0x0106, value)

    def get_6dof_data(self):
        acc = [self._read_s16(0x0012 + i) for i in range(3)]
        gyr = [self._read_s16(0x0015 + i) for i in range(3)]
        return acc, gyr

    def get_geomagnetic_data(self):
        return [self._read_s16(0x0018 + i) for i in range(3)]

    def get_euler(self):
        return self._read_f32(0x001B), self._read_f32(0x001D), self._read_f32(0x001F)

    def get_compass_degree(self):
        return self._read_f32(0x0021)

    def set_bmi_mode(self, mode):
        self._write_u16(0x0107, mode)

    def reset_bmi(self):
        self._write_u16(0x010E, 1)

    def set_accel_range(self, range_code):
        self._write_u16(0x0108, range_code)

    def set_gyro_range(self, range_code):
        self._write_u16(0x0109, range_code)

    def set_bmm_mode(self, op_mode):
        self._write_u16(0x010A, op_mode)

    def set_bmm_preset_mode(self, preset, rate):
        self._write_u16(0x010C, rate)
        self._write_u16(0x010B, preset)

    def set_bmm_rate(self, rate):
        self._write_u16(0x010C, rate)

    def set_bmm_measurement_xyz(self, en_x, en_y, en_z):
        axes = (1 if en_x else 0) | (2 if en_y else 0) | (4 if en_z else 0)
        self._write_u16(0x010D, axes)

    def get_data_ready_state(self):
        return self._read_u16(0x003D) != 0

    def fusion_ahrs_get_quaternion(self):
        return [self._read_f32(0x0023 + i * 2) for i in range(4)]

    def fusion_ahrs_get_gravity(self):
        return [self._read_f32(0x002B + i * 2) for i in range(3)]

    def fusion_ahrs_get_linear_acceleration(self):
        return [self._read_f32(0x0031 + i * 2) for i in range(3)]

    def fusion_ahrs_get_earth_acceleration(self):
        return [self._read_f32(0x0037 + i * 2) for i in range(3)]

    def fusion_bias_get_offset(self):
        return [self._read_f32(0x0041 + i * 2) for i in range(3)]

    def set_mag_calibration(self, hard_iron, soft_iron):
        for i in range(3):
            self._write_f32(0x0110 + i * 2, hard_iron[i])
        for i in range(9):
            self._write_f32(0x0116 + i * 2, soft_iron[i // 3][i % 3])
        self._write_u16(0x0128, 1)

    def _read_u16(self, reg):
        from smbus2 import i2c_msg

        w = i2c_msg.write(self._addr, [(reg >> 8) & 0xFF, reg & 0xFF])
        r = i2c_msg.read(self._addr, 2)
        self._bus.i2c_rdwr(w, r)
        data = list(r)
        return (data[0] << 8) | data[1]

    def _read_s16(self, reg):
        v = self._read_u16(reg)
        return v - 0x10000 if v & 0x8000 else v

    def _read_f32(self, reg_hi):
        hi = self._read_u16(reg_hi)
        lo = self._read_u16(reg_hi + 1)
        return struct.unpack(">f", struct.pack(">I", (hi << 16) | lo))[0]

    def _write_u16(self, reg, value):
        from smbus2 import i2c_msg
        payload = [(reg >> 8) & 0xFF, reg & 0xFF, (value >> 8) & 0xFF, value & 0xFF]
        if hasattr(self._bus, "i2c_rdwr"):
            self._bus.i2c_rdwr(i2c_msg.write(self._addr, payload))
        else:
            self._bus.write_i2c_block_data(self._addr, payload[0], payload[1:])

    def _write_f32(self, reg_hi, value):
        raw = struct.unpack(">I", struct.pack(">f", float(value)))[0]
        self._write_u16(reg_hi, (raw >> 16) & 0xFFFF)
        self._write_u16(reg_hi + 1, raw & 0xFFFF)
