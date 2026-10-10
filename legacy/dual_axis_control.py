# -*- coding: utf-8 -*-
"""
瓴控 MG V2.36 + SMC60 + PEAK PCAN USB 双轴摇杆速度控制（V9）

功能：
  - 启动后直接控制：X左右=横滚0x141，Y前后=俯仰0x142；推得越多速度越快。
  - 回中：发送A2目标速度0，利用9C确认停稳、9C/92反馈新鲜后发送A4在当前位置保持。
  - 无S键、无角度示教、无固定角度软件限位；忽略以前的mg_taught_limits.json。
  - 运动过程中依据【转矩电流突增+实际转速下降】或【高电流持续无运动】检测疑似干涉。
  - 疑似干涉：两轴同时停车（防止耦合机构另一轴继续挤压），程序不退出；
    只锁定出现异常的那一轴的危险方向，回中后允许反向退出。
  - 真正的CAN反馈故障/驱动故障仍会全局停机，不能仅靠本程序防止人身伤害。

⚠️ 该检测不能保证发现所有碰撞/夹挤，尤其是低摩擦、柔性干涉或高速度接触。
    未设置角度边界时，更必须有实体急停、机械保护、可靠承载支撑，并从低速开始实测。
    本程序不改电机ROM的最大扭矩/速度/零点设置，不提升电流上限。

环境：Windows + python-can + PCAN-USB，运行前关闭PCAN-View。
命令：python dual_axis_collision_control_v9.py
按键：Q退出；I清除电流峰值。无S键/角度示教。
"""
from __future__ import annotations

# =====================================================================
# 【1】可修改参数（集中在前面；确认硬件额定值前不建议提高电流）
# =====================================================================

# 1.1 总线与实际CAN ID
PCAN_CHANNEL = 'PCAN_USBBUS1'
CAN_BITRATE = 250_000
JOYSTICK_CAN_ID = 0x101
PITCH_CAN_ID = 0x142                  # 实际电机ID2，俯仰（前后）
ROLL_CAN_ID = 0x141                   # 实际电机ID1，横滚（左右）
GEAR_RATIO = 24.0                     # 电机轴角速度 / 输出轴角速度
PITCH_MOTOR_SIGN = +1                # 前推为正；方向不对才改 -1
ROLL_MOTOR_SIGN = +1                 # 右推为正；方向不对才改 -1

# 1.2 摇杆速度：默认只开放2°/s作为无位置限位时的低速调试速度
#     确认运动安全后可逐步上调，不能只依靠电流碰撞检测保护机械。
RUN_MAX_SPEED_OUTPUT_DPS = 20.0       # 输出轴最大速度，非电机轴速度
MIN_MOVE_SPEED_OUTPUT_DPS = 0.35     # 摇杆刚离开死区时最低速度
JOY_CENTER = 0x0800
JOY_DEADZONE = 0x0030
JOY_MAX_DELTA = 0x0700
JOY_TIMEOUT_SEC = 0.20               # 手柄失联马上请求零速

# 1.3 电流：官方A2 8字节= A2 00 + iqLimit(int16) + speed(int32/100)
#     1000只表示本次速度指令的转矩电流限制，不是电机真实峰值额定电流。
A2_IQ_LIMIT_RAW = 1000               # 沿用V8，未增加扭矩电流上限
HOLD_A4_MAX_SPEED_MOTOR_DPS = 80.0   # A4在当前角度保持时的最大电机轴速度

# 1.4 【疑似干涉】检测 —— 运动中才判定，静止A4保持不判定为碰撞。
#     这不是经标定的碰撞阈值；只能试验调节，不能当作功能安全认证。
COLLISION_START_GRACE_SEC = 0.35     # 推杆开始后的启动缓冲
COLLISION_MIN_COMMAND_OUT_DPS = 0.8  # 过小的运动目标不参与碰撞判断
# 策略 A：相对近期正常运动的电流基线，出现明显增长且速度掉得很低
SURGE_MIN_IQ_RAW = 300               # 只有Iq大于此值才考虑突增
SURGE_ABOVE_BASE_RAW = 170           # 绝对Iq相对正常运动基线至少增加170
SURGE_SLOW_RATIO = 0.45             # 实际电机转速 < 指令绝对速度的45%
SURGE_CONFIRM_SEC = 0.16            # 条件持续满足才触发
# 策略 B：即使基线高，也检测电流很大并几乎停止
HIGH_IQ_RAW = 760
HIGH_IQ_LOW_MOTOR_DPS = 6.0          # 电机轴速度，输出轴对应6/24=0.25°/s
HIGH_IQ_CONFIRM_SEC = 0.28
# 策略 C：较长时间完全不动（避免持续带电堵转）
NO_PROGRESS_IQ_RAW = 450
NO_PROGRESS_MOTOR_DPS = 4.0
NO_PROGRESS_CONFIRM_SEC = 0.75
# 策略 D：已逼近【本次指令的电流限制】，且速度明显低于命令
NEAR_LIMIT_IQ_FRACTION = 0.93
NEAR_LIMIT_CONFIRM_SEC = 0.18
# 检测事件后：原方向禁止，必须先回中、再往反方向退出足够距离才解除原方向锁定
REVERSE_CLEAR_OUTPUT_DEG = 1.5

# 1.5 周期与异常保护
LOOP_SEC = 0.010
SPEED_TX_PERIOD_SEC = 0.040
ANGLE_QUERY_PERIOD_SEC = 0.050
CURRENT_QUERY_PERIOD_SEC = 0.050
STATE_QUERY_PERIOD_SEC = 0.70
FEEDBACK_TIMEOUT_SEC = 0.45
STOP_STABLE_SEC = 0.12
STOP_SETTLE_TIMEOUT_SEC = 2.5
STOP_SPEED_MOTOR_DPS = 7.0
STARTUP_RETRIES = 5
STARTUP_REPLY_TIMEOUT = 0.3
PRINT_PERIOD_SEC = 0.5

# 1.6 9C转矩电流显示比例：MG CAN V2.36写66/4096 A/LSB。
#     MG使用手册另一处写2048→16.5A，两份资料冲突，故安培值仅供参考。
IQ_AMP_PER_RAW_EST = 66.0 / 4096.0

# =====================================================================
# 【2】以下为功能实现，一般不需要修改
# =====================================================================
import os
import time
from dataclasses import dataclass, field
from typing import Optional

import can

if os.name == 'nt':
    import msvcrt
else:
    msvcrt = None


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(v, hi))


def sign(v: float) -> int:
    return 1 if v > 0 else (-1 if v < 0 else 0)


def send_frame(bus, can_id: int, data: bytes) -> None:
    if len(data) != 8:
        raise ValueError('MG V2.36必须使用8字节标准数据帧')
    bus.send(can.Message(arbitration_id=can_id, is_extended_id=False,
                         data=bytes(data)), timeout=0.1)


def send_cmd(bus, can_id: int, opcode: int, data1: int = 0) -> None:
    send_frame(bus, can_id, bytes([opcode, data1, 0, 0, 0, 0, 0, 0]))


def send_a2(bus, can_id: int, motor_dps: float) -> None:
    """0xA2: iqControl signed16小端+速度 signed32小端(0.01电机dps/LSB)。"""
    if not 1 <= A2_IQ_LIMIT_RAW <= 2048:
        raise ValueError('A2_IQ_LIMIT_RAW超出MG协议允许范围')
    raw = round(motor_dps * 100)
    if not -(1 << 31) <= raw < (1 << 31):
        raise ValueError('A2速度越界')
    data = (b'\xA2\x00' + A2_IQ_LIMIT_RAW.to_bytes(2, 'little', signed=True)
            + raw.to_bytes(4, 'little', signed=True))
    send_frame(bus, can_id, data)


def send_a4(bus, can_id: int, motor_deg: float) -> None:
    """0xA4: 停稳后把当前位置作为位置保持目标，不返回0°。"""
    raw = round(motor_deg * 100)
    if not -(1 << 31) <= raw < (1 << 31):
        raise ValueError('A4多圈角度越界')
    speed = round(HOLD_A4_MAX_SPEED_MOTOR_DPS)
    if not 1 <= speed <= 65535:
        raise ValueError('A4速度越界')
    send_frame(bus, can_id, b'\xA4\x00' + speed.to_bytes(2, 'little')
               + raw.to_bytes(4, 'little', signed=True))


def parse_92(d: bytes) -> float:
    if len(d) != 8 or d[0] != 0x92:
        raise ValueError('非法92回复')
    return int.from_bytes(d[1:8], 'little', signed=True) / 100.0


def parse_9c(d: bytes):
    if len(d) != 8 or d[0] not in (0x9C, 0xA2):
        raise ValueError('非法9C/A2回复')
    temp = int.from_bytes(d[1:2], 'little', signed=True)
    iq = int.from_bytes(d[2:4], 'little', signed=True)
    speed = int.from_bytes(d[4:6], 'little', signed=True)
    return temp, iq, speed


def parse_joy(d: bytes):
    if len(d) != 8 or d[-1] != 0xA5:
        return None
    x = int.from_bytes(d[0:2], 'little')
    y = int.from_bytes(d[2:4], 'little')
    return (x, y) if 0 <= x <= 4095 and 0 <= y <= 4095 else None


def joy_ratio(value: int) -> float:
    delta = value - JOY_CENTER
    if abs(delta) <= JOY_DEADZONE:
        return 0.
    mag = clamp((abs(delta) - JOY_DEADZONE) / max(1, JOY_MAX_DELTA - JOY_DEADZONE), 0, 1)
    return mag if delta > 0 else -mag


@dataclass
class Axis:
    name: str
    can_id: int
    stick: str
    motor_sign: int
    motor_deg: Optional[float] = None
    angle_at: float = 0.
    temp_c: int = 0
    iq_raw: Optional[int] = None
    speed_motor_dps: Optional[int] = None
    feedback_at: float = 0.
    motor_state: Optional[int] = None
    motor_error: int = 0
    voltage_v: float = 0.
    bus_i_a: float = 0.
    status_at: float = 0.
    brake: Optional[int] = None
    brake_at: float = 0.
    iq_peak: int = 0
    mode: str = 'HOLD'                 # HOLD/MOVE/SETTLING
    cmd_output_dps: float = 0.
    move_at: float = 0.
    last_tx_at: float = 0.
    stop_at: float = 0.
    stopped_stable_at: float = 0.
    need_neutral: bool = True
    blocked: dict = field(default_factory=dict)  # {+1:位置,-1:位置}，以输出轴角度计
    baseline_iq: Optional[float] = None
    surge_at: float = 0.
    high_at: float = 0.
    frozen_at: float = 0.
    near_at: float = 0.

    def output_deg(self):
        return None if self.motor_deg is None else self.motor_deg / (GEAR_RATIO * self.motor_sign)

    def output_speed(self):
        return None if self.speed_motor_dps is None else self.speed_motor_dps / (GEAR_RATIO * self.motor_sign)

    def reset_detection(self):
        self.surge_at = self.high_at = self.frozen_at = self.near_at = 0.


def persists(a: Axis, attr: str, cond: bool, now: float, duration: float) -> bool:
    if not cond:
        setattr(a, attr, 0.)
        return False
    start = getattr(a, attr)
    if start <= 0.:
        setattr(a, attr, now)
        return False
    return now - start >= duration


class Controller:
    def __init__(self):
        self.pitch = Axis('俯仰', PITCH_CAN_ID, 'y', PITCH_MOTOR_SIGN)
        self.roll = Axis('横滚', ROLL_CAN_ID, 'x', ROLL_MOTOR_SIGN)
        self.axes = (self.pitch, self.roll)
        self.by_id = {a.can_id: a for a in self.axes}
        self.bus = None
        self.running = True
        self.has_joy = False
        self.x, self.y = JOY_CENTER, JOY_CENTER
        self.joy_at = 0.
        self.last_joy_ok = False
        self.start_at = 0.
        self.last_angle_q = self.last_current_q = self.last_status_q = 0.
        self.last_print = 0.

    def recv_msg(self, msg):
        now = time.monotonic()
        if msg is None or msg.is_extended_id or msg.is_remote_frame:
            return
        can_id = msg.arbitration_id
        if can_id == JOYSTICK_CAN_ID:
            j = parse_joy(bytes(msg.data))
            if j:
                self.x, self.y = j
                self.joy_at = now
                self.has_joy = True
            return
        a = self.by_id.get(can_id)
        if not a or len(msg.data) != 8:
            return
        d = bytes(msg.data)
        op = d[0]
        if op == 0x92:
            a.motor_deg = parse_92(d)
            a.angle_at = now
        elif op == 0x9A:
            a.temp_c = int.from_bytes(d[1:2], 'little', signed=True)
            a.voltage_v = int.from_bytes(d[2:4], 'little', signed=True) * 0.01
            a.bus_i_a = int.from_bytes(d[4:6], 'little', signed=True) * 0.01
            a.motor_state, a.motor_error = d[6], d[7]
            a.status_at = now
            if a.motor_error:
                self.global_fault(f'{a.name} 驱动错误位=0x{a.motor_error:02X}')
        elif op == 0x8C:
            if d[1] in (0, 1):
                a.brake = d[1]
                a.brake_at = now
        elif op in (0x9C, 0xA2):
            a.temp_c, a.iq_raw, a.speed_motor_dps = parse_9c(d)
            a.feedback_at = now
            a.iq_peak = max(a.iq_peak, abs(a.iq_raw))

    def drain(self):
        for _ in range(150):
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                return
            self.recv_msg(msg)
            if not self.running:
                return

    def query_one(self, a: Axis, opcode: int):
        for i in range(STARTUP_RETRIES):
            before = time.monotonic()
            send_cmd(self.bus, a.can_id, opcode)
            until = before + STARTUP_REPLY_TIMEOUT
            while time.monotonic() < until:
                msg = self.bus.recv(timeout=.02)
                if msg:
                    self.recv_msg(msg)
                ts = {0x92: a.angle_at, 0x9C: a.feedback_at, 0x9A: a.status_at}[opcode]
                if ts >= before:
                    return
            print(f'[启动重试] {a.name} 0x{opcode:02X} 第{i+1}/{STARTUP_RETRIES}次')
        raise RuntimeError(f'{a.name} 0x{opcode:02X} 无有效回复；请核对PCAN-View中的同类报文')

    def brake_confirm(self, a: Axis):
        for _ in range(3):
            before = time.monotonic()
            send_cmd(self.bus, a.can_id, 0x8C, 0x10)
            until = before + .3
            while time.monotonic() < until:
                msg = self.bus.recv(timeout=.02)
                if msg:
                    self.recv_msg(msg)
                if a.brake_at >= before:
                    if a.brake == 1:
                        print(f'[抱闸] {a.name} 驱动反馈释放（仍需确认机械实际释放）')
                        return
                    raise RuntimeError(f'{a.name} 抱闸未释放')
        raise RuntimeError(f'{a.name} 读取抱闸状态超时')

    def startup(self):
        print(f'[CAN] {PCAN_CHANNEL}  {CAN_BITRATE} bps  手柄101 / 俯仰142 / 横滚141')
        self.bus = can.Bus(interface='pcan', channel=PCAN_CHANNEL,
                           bitrate=CAN_BITRATE, receive_own_messages=False)
        for a in self.axes:
            for op in (0x9A, 0x92, 0x9C):
                self.query_one(a, op)
            print(f'[状态] {a.name} 角度={a.output_deg():+.2f}° '
                  f'Iq={a.iq_raw:+d}, 电机轴转速={a.speed_motor_dps:+d}dps')
        for a in self.axes:
            send_cmd(self.bus, a.can_id, 0x88)  # Motor ON
            time.sleep(.12)
            self.query_one(a, 0x9A)
            if a.motor_state != 0 or a.motor_error:
                raise RuntimeError(f'{a.name} 未使能或驱动器故障，拒绝解锁抱闸')
        for a in self.axes:
            send_a4(self.bus, a.can_id, a.motor_deg)  # 保持当前位置，不回0点
        for a in self.axes:
            send_cmd(self.bus, a.can_id, 0x8C, 0x01)  # 释放制动器
        time.sleep(.12)
        for a in self.axes:
            self.brake_confirm(a)
        self.start_at = time.monotonic()
        self.last_angle_q = self.last_current_q = 0.
        print('\n[启动完成] 摇杆直接控制，无S键、无角度示教、无固定角度软限位。')
        print(f'[速度] 最大输出轴速度 {RUN_MAX_SPEED_OUTPUT_DPS:.2f}°/s；A2电流限幅原始值={A2_IQ_LIMIT_RAW}（未提高）')
        print('[疑似干涉] 两轴一同减速停车以避免耦合挤压；只锁异常轴当前方向；回中后反向退出。')
        print('[按键] I 清除峰值；Q 请求停止并退出；Ctrl+C退出。')
        print('[警告] 此检测不能保证及时发现所有接触，实体急停、支撑与安全距离必须到位。\n')

    def global_fault(self, reason: str):
        if not self.running:
            return
        self.running = False
        print(f'\n[全局故障] {reason}；正在尝试速度0和上抱闸，程序将退出，请使用实体急停！')
        if self.bus:
            for a in self.axes:
                try:
                    send_a2(self.bus, a.can_id, 0.)
                    send_cmd(self.bus, a.can_id, 0x8C, 0x00)
                except Exception:
                    pass

    def stop_axis(self, a: Axis, reason: Optional[str] = None):
        """运动时立即发速度0；保持驱动ON，停稳再发A4。只拦截该轴。"""
        if a.mode == 'SETTLING':
            return
        if reason is not None:
            direction = sign(a.cmd_output_dps)
            if direction:
                a.blocked[direction] = a.output_deg()
                print(f'\n[疑似干涉/本轴保护] {a.name} 输出角={a.output_deg():+.2f}° '
                      f'Iq={a.iq_raw:+d} 实际速度={a.output_speed():+.2f}°/s；{reason}')
                print(f'[方向锁定] 禁止继续向 {"正" if direction > 0 else "负"}方向运动；'
                      '请摇杆回中、待停稳后向反方向退出。程序保持运行。')
        send_a2(self.bus, a.can_id, 0.)
        a.cmd_output_dps = 0.
        a.last_tx_at = time.monotonic()
        a.mode = 'SETTLING'
        a.stop_at = time.monotonic()
        a.stopped_stable_at = 0.
        a.move_at = 0.
        a.need_neutral = True
        a.reset_detection()

    def on_collision(self, a: Axis, reason: str):
        """二自由度强耦合：任何一个关节疑似碰撞，都立刻要求两轴零速。

        但只锁定真正检测到异常的轴的当前运动方向，不退出程序。
        另一轴可在所有摇杆都回中、两轴停稳后继续运动以解除干涉。
        """
        self.stop_axis(a, reason)
        for other in self.axes:
            if other is a:
                continue
            if other.mode == 'MOVE':
                self.stop_axis(other)
            else:
                other.need_neutral = True
        print('[双轴协同停车] 两轴均已请求速度0；程序保持运行，请先回中再反向退出。')

    def settling(self, a: Axis, now: float):
        if now - a.last_tx_at >= SPEED_TX_PERIOD_SEC:
            send_a2(self.bus, a.can_id, 0.)
            a.last_tx_at = now
        if now - a.stop_at > STOP_SETTLE_TIMEOUT_SEC:
            self.global_fault(f'{a.name} 停车后{STOP_SETTLE_TIMEOUT_SEC}s未停稳')
            return
        fresh = (now - a.angle_at < FEEDBACK_TIMEOUT_SEC
                 and now - a.feedback_at < FEEDBACK_TIMEOUT_SEC)
        slow = (a.speed_motor_dps is not None and abs(a.speed_motor_dps) <= STOP_SPEED_MOTOR_DPS)
        if fresh and slow:
            if a.stopped_stable_at <= 0:
                a.stopped_stable_at = now
            if now - a.stopped_stable_at >= STOP_STABLE_SEC and a.angle_at >= a.stop_at:
                send_a4(self.bus, a.can_id, a.motor_deg)
                a.mode = 'HOLD'
                a.cmd_output_dps = 0.
                a.stopped_stable_at = 0.
        else:
            a.stopped_stable_at = 0.

    def desired_speed(self, ratio: float) -> float:
        if ratio == 0:
            return 0.
        speed = max(MIN_MOVE_SPEED_OUTPUT_DPS, abs(ratio) * RUN_MAX_SPEED_OUTPUT_DPS)
        return speed * sign(ratio)

    def control_axis(self, a: Axis, desired: float, now: float, joy_fresh: bool = False):
        if a.mode == 'SETTLING':
            self.settling(a, now)
            return
        if a.need_neutral:
            # 注意不能用“手柄已失联时模拟的0”解除回中要求！
            # 必须亲自收到手柄实时中心位，才允许新的运动。
            if desired == 0 and joy_fresh:
                a.need_neutral = False
            desired = 0.
        # 方向屏蔽只针对本次发生疑似干涉的方向，保留反向退出。
        if sign(desired) in a.blocked:
            desired = 0.
        if desired == 0:
            if a.mode == 'MOVE':
                self.stop_axis(a)
            return
        if a.mode == 'HOLD':
            a.mode = 'MOVE'
            a.move_at = now
            a.baseline_iq = float(abs(a.iq_raw or 0))
            a.reset_detection()
            a.last_tx_at = 0.
        if (now - a.last_tx_at >= SPEED_TX_PERIOD_SEC
                or abs(desired - a.cmd_output_dps) >= .25):
            send_a2(self.bus, a.can_id, desired * GEAR_RATIO * a.motor_sign)
            a.last_tx_at = now
            a.cmd_output_dps = desired

    def verify_retreat(self, a: Axis):
        """朝原碰撞方向的反方向退出1.5°后解除该方向的软锁。"""
        current = a.output_deg()
        if current is None:
            return
        for d, location in list(a.blocked.items()):
            # 只有主动给出【反向速度指令】且确实退回一定距离才解锁；
            # 重力造成的被动位移不得自动解除碰撞方向禁令。
            retreat_commanded = (a.mode == 'MOVE' and sign(a.cmd_output_dps) == -d)
            if retreat_commanded and (current - location) * d <= -REVERSE_CLEAR_OUTPUT_DEG:
                del a.blocked[d]
                print(f'\n[退出干涉区] {a.name} 已朝反方向退出至少{REVERSE_CLEAR_OUTPUT_DEG}°，'
                      '解除原运动方向禁令；如果仍有阻力请停止操作。')

    def check_collision(self, a: Axis, now: float):
        # 只根据最近的有效9C反馈判断正在运动的轴
        if a.mode != 'MOVE' or a.move_at <= 0 or a.iq_raw is None or a.speed_motor_dps is None:
            a.reset_detection()
            return
        if now - a.feedback_at > FEEDBACK_TIMEOUT_SEC:
            return
        asked = abs(a.cmd_output_dps)
        if asked < COLLISION_MIN_COMMAND_OUT_DPS:
            a.reset_detection()
            return
        iq = abs(a.iq_raw)
        measured = abs(a.speed_motor_dps)
        commanded_motor_dps = asked * GEAR_RATIO
        # 只有实际已经能够跟随速度时，才缓慢更新“正常运动Iq”基线。
        # 避免撞击造成的持续电流上涨把基线带高，掩盖突变。
        if measured >= commanded_motor_dps * 0.7:
            a.baseline_iq = iq if a.baseline_iq is None else (0.97 * a.baseline_iq + 0.03 * iq)
        if now - a.move_at < COLLISION_START_GRACE_SEC:
            a.reset_detection()
            return
        baseline = a.baseline_iq if a.baseline_iq is not None else iq
        surge = (iq >= SURGE_MIN_IQ_RAW
                 and iq - baseline >= SURGE_ABOVE_BASE_RAW
                 and measured <= commanded_motor_dps * SURGE_SLOW_RATIO)
        high = iq >= HIGH_IQ_RAW and measured <= HIGH_IQ_LOW_MOTOR_DPS
        frozen = iq >= NO_PROGRESS_IQ_RAW and measured <= NO_PROGRESS_MOTOR_DPS
        near = iq >= A2_IQ_LIMIT_RAW * NEAR_LIMIT_IQ_FRACTION and measured <= commanded_motor_dps * .35
        if persists(a, 'surge_at', surge, now, SURGE_CONFIRM_SEC):
            self.on_collision(a, f'电流突增：Iq={iq}，运动基线≈{baseline:.0f}，速度只有{measured:.0f}/{commanded_motor_dps:.0f}dps')
        elif persists(a, 'high_at', high, now, HIGH_IQ_CONFIRM_SEC):
            self.on_collision(a, f'高电流近零速：Iq={iq}，电机轴速度={measured:.0f}dps')
        elif persists(a, 'frozen_at', frozen, now, NO_PROGRESS_CONFIRM_SEC):
            self.on_collision(a, f'持续失速：Iq={iq}，电机轴速度={measured:.0f}dps')
        elif persists(a, 'near_at', near, now, NEAR_LIMIT_CONFIRM_SEC):
            self.on_collision(a, f'接近本次电流上限{A2_IQ_LIMIT_RAW}且速度不足')

    def periodic(self, now: float):
        if now - self.last_angle_q >= ANGLE_QUERY_PERIOD_SEC:
            for a in self.axes:
                send_cmd(self.bus, a.can_id, 0x92)
            self.last_angle_q = now
        if now - self.last_current_q >= CURRENT_QUERY_PERIOD_SEC:
            for a in self.axes:
                send_cmd(self.bus, a.can_id, 0x9C)
            self.last_current_q = now
        if now - self.last_status_q >= STATE_QUERY_PERIOD_SEC:
            for a in self.axes:
                send_cmd(self.bus, a.can_id, 0x9A)
            self.last_status_q = now
        if now - self.start_at < FEEDBACK_TIMEOUT_SEC:
            return
        for a in self.axes:
            if now - a.angle_at > FEEDBACK_TIMEOUT_SEC or now - a.feedback_at > FEEDBACK_TIMEOUT_SEC:
                self.global_fault(f'{a.name} CAN角度或转速电流反馈超过{FEEDBACK_TIMEOUT_SEC}s没有更新')
                return
            self.check_collision(a, now)
            if not self.running:
                return
            if a.mode in ('HOLD', 'MOVE'):
                self.verify_retreat(a)

    def read_keyboard(self):
        if msvcrt is None:
            return
        for _ in range(10):
            if not msvcrt.kbhit():
                return
            key = msvcrt.getwch().lower()
            if key in ('\x00', '\xe0'):
                if msvcrt.kbhit():
                    msvcrt.getwch()
                continue
            if key == 'q':
                self.running = False
                return
            if key == 'i':
                for a in self.axes:
                    a.iq_peak = abs(a.iq_raw or 0)
                print('\n[电流] 已重新开始记录本次峰值')

    def print_status(self, now: float):
        if now - self.last_print < PRINT_PERIOD_SEC:
            return
        self.last_print = now
        print(f'\n[NORMAL] 摇杆 X=0x{self.x:04X} Y=0x{self.y:04X}')
        for a in self.axes:
            p = a.output_deg()
            speed = a.output_speed()
            iq = a.iq_raw or 0
            blocked = ','.join('正向' if k == 1 else '反向' for k in sorted(a.blocked)) or '无'
            print(f'  {a.name} 位置={p:+7.2f}° 实际速度={speed:+5.2f}°/s '
                  f'目标速度={a.cmd_output_dps:+5.2f}°/s '
                  f'Iq={iq:+5d} (~{iq*IQ_AMP_PER_RAW_EST:+.2f}A，仅估算) '
                  f'峰值={a.iq_peak} 状态={a.mode} 被禁止方向={blocked} '
                  f'电压={a.voltage_v:.1f}V 温度={a.temp_c}°C')

    def control(self, now: float):
        joy_ok = self.has_joy and now - self.joy_at <= JOY_TIMEOUT_SEC
        if self.last_joy_ok and not joy_ok:
            print('\n[手柄超时] 请求速度0；恢复后必须先回中')
            for a in self.axes:
                a.need_neutral = True
        self.last_joy_ok = joy_ok
        xr = joy_ratio(self.x) if joy_ok else 0.
        yr = joy_ratio(self.y) if joy_ok else 0.
        self.control_axis(self.pitch, self.desired_speed(yr), now, joy_fresh=joy_ok)
        self.control_axis(self.roll, self.desired_speed(xr), now, joy_fresh=joy_ok)

    def shutdown(self):
        if self.bus is None:
            return
        print('\n[程序退出] 请求A2速度0、锁抱闸、0x81停止')
        try:
            for a in self.axes:
                try:
                    send_a2(self.bus, a.can_id, 0.)
                except Exception:
                    pass
            time.sleep(.08)
            for a in self.axes:
                try:
                    send_cmd(self.bus, a.can_id, 0x8C, 0)
                except Exception:
                    pass
            time.sleep(.12)
            for a in self.axes:
                try:
                    send_cmd(self.bus, a.can_id, 0x81)
                except Exception:
                    pass
                print(f'[Iq峰值] {a.name}: {a.iq_peak} raw (估算{a.iq_peak*IQ_AMP_PER_RAW_EST:.2f} A)')
        finally:
            self.bus.shutdown()
            self.bus = None

    def run(self):
        try:
            self.startup()
            tick = time.monotonic()
            while self.running:
                self.drain()
                if not self.running:
                    break
                self.read_keyboard()
                if not self.running:
                    break
                now = time.monotonic()
                self.periodic(now)
                if not self.running:
                    break
                self.control(now)
                self.print_status(now)
                tick += LOOP_SEC
                remaining = tick - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    tick = time.monotonic()
        except KeyboardInterrupt:
            print('\n[Ctrl+C] 用户请求退出')
        finally:
            self.shutdown()


if __name__ == '__main__':
    if os.name != 'nt':
        raise SystemExit('此程序用于Windows，键盘监听使用msvcrt。')
    Controller().run()
