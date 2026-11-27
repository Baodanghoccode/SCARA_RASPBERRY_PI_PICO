# main.py - Firmware SCARA 4 trục cho Raspberry Pi Pico 2 W (MicroPython)
# Khung lệnh:  #LỆNH:DỮ_LIỆU\r\n   ->   #OK:LỆNH | #VALUE:... | #ERR:lý_do | #EVT:sự_kiện
# Chân theo sơ đồ SCARA_PICO2W_V1.kicad_sch. Vị trí tính bằng XUNG (bước), đơn vị do giao diện quy đổi.
#
# Lệnh:  PING | START | STOP | HALT | STAT | EN:1/0
#        CFG:trục,vmax,acc,đảo_chiều | LIM:trục,min,max | ZERO:trục(0=tất cả)
#        MOVA:s1,s2,s3,s4,f   (đích tuyệt đối, 'x' = giữ nguyên trục đó, f = 0..1 tốc độ)
#        MOVR:d1,d2,d3,d4,f   (đi tương đối)
#        JOG:trục,hướng(-1/0/1),f   (gửi lặp ~100 ms/lần, mất lệnh > 300 ms tự dừng)
#        HOME:trục,hướng_về_công_tắc(-1/1),xung_mỗi_giây
import sys, time

from machine import Pin

N = 4
STEP = (2, 5, 8, 11)       # STEP của A4988 trục 1..4
DIR = (3, 6, 9, 12)        # DIR
EN_N = (4, 7, 10, 13)      # ~ENABLE (0 = bật driver; mạch có kéo lên nên mặc định tắt)
HOME = (16, 17, 18, 19)    # HOME1..4
ESTOP = 20                 # STOP_SENSE

HOME_ACTIVE = 1            # công tắc NC: hở khi tác động -> R4k7 kéo lên 3V3 -> đọc 1
ESTOP_ACTIVE = 1           # nút dừng NC: hở -> đọc 1
ESTOP_ENABLED = True       # đặt False nếu J905 chưa nối kín (NC)
VMIN = 60.0                # xung/s: tốc độ bắt đầu / kết thúc mỗi lần chạy
JOG_TIMEOUT = 300000       # us: mất lệnh JOG quá lâu thì tự giảm tốc dừng
SEARCH_STEPS = 200000      # số xung tối đa để tìm công tắc home

ticks_us, ticks_diff, ticks_add = time.ticks_us, time.ticks_diff, time.ticks_add


def out(s):
    sys.stdout.write(s + "\r\n")


class Move:
    pass


class Machine:
    def __init__(self):
        self.step = [Pin(p, Pin.OUT, value=0) for p in STEP]
        self.dir = [Pin(p, Pin.OUT, value=0) for p in DIR]
        self.en = [Pin(p, Pin.OUT, value=1) for p in EN_N]
        self.home = [Pin(p, Pin.IN) for p in HOME]
        self.est = Pin(ESTOP, Pin.IN)
        self.pos = [0] * N
        self.vmax = [1500.0] * N
        self.acc = [3000.0] * N
        self.inv = [0] * N
        self.lo = [-10 ** 9] * N
        self.hi = [10 ** 9] * N
        self.homed = [0] * N
        self.enabled = False
        self.estop = False
        self.mv = None

    # ---------- phần cứng ----------
    def set_en(self, on):
        for p in self.en:
            p.value(0 if on else 1)
        self.enabled = on

    def set_dir(self, i, sg):
        self.dir[i].value(1 if (sg > 0) != bool(self.inv[i]) else 0)

    def kill(self):
        self.mv = None
        self.set_en(False)

    # ---------- bắt đầu chuyển động ----------
    def start(self, d, f, now, jog=False):
        axes = [i for i in range(N) if d[i]]
        if not axes:
            return None
        for i in axes:
            t = self.pos[i] + d[i]
            if t < self.lo[i] or t > self.hi[i]:
                return 'ERR:LIMIT:%d' % (i + 1)
        n = max(abs(d[i]) for i in axes)
        vt = a = 1e12
        for i in axes:                      # trục có nhiều xung nhất là "nhịp"; các trục khác tỉ lệ theo
            k = n / abs(d[i])
            vt = min(vt, f * self.vmax[i] * k)
            a = min(a, self.acc[i] * k)
        m = Move()
        m.axes, m.n, m.done, m.hp = axes, n, 0, 0
        m.ad = [abs(x) for x in d]
        m.sg = [1 if x > 0 else -1 for x in d]
        m.ac = [n // 2] * N
        m.vt, m.a = vt, a
        m.v0 = m.v = min(VMIN, vt)
        m.halt, m.jog = False, jog
        m.deadline = ticks_add(now, JOG_TIMEOUT)
        m.next_t = now
        for i in axes:
            self.set_dir(i, m.sg[i])
        self.mv = m
        return None

    def start_home(self, i, d, sps, now):
        m = Move()
        m.axes, m.n, m.done, m.hp = [i], SEARCH_STEPS, 0, 1
        m.ad = [0] * N
        m.ad[i] = SEARCH_STEPS
        m.sg = [1] * N
        m.sg[i] = 1 if d > 0 else -1
        m.ac = [0] * N
        m.vt = m.v = m.v0 = float(sps)
        m.a, m.halt, m.jog, m.deadline, m.next_t = 0.0, False, False, 0, now
        self.set_dir(i, m.sg[i])
        self.mv = m

    def home_flip(self, m):                 # chạm công tắc -> lùi ra chậm cho tới khi nhả
        i = m.axes[0]
        m.hp, m.n, m.done = 2, SEARCH_STEPS, 0
        m.sg[i] = -m.sg[i]
        m.vt = m.v = max(VMIN, m.v / 4)
        self.set_dir(i, m.sg[i])
        time.sleep_us(10)

    # ---------- vòng chạy chính: gọi liên tục ----------
    def service(self, now):
        if ESTOP_ENABLED and not self.estop and self.est.value() == ESTOP_ACTIVE:
            self.kill()
            self.estop = True
            out('#ERR:ESTOP')
            return
        m = self.mv
        if m is None or ticks_diff(now, m.next_t) < 0:
            return
        if m.hp:
            i = m.axes[0]
            if m.halt:
                self.mv = None
                return
            hit = self.home[i].value() == HOME_ACTIVE
            if m.hp == 1 and hit:
                self.home_flip(m)
            elif m.hp == 2 and not hit:
                self.pos[i] = 0
                self.homed[i] = 1
                self.mv = None
                out('#EVT:HOME:%d' % (i + 1))
                return
            elif m.done >= m.n:
                self.mv = None
                out('#ERR:HOME_FAIL:%d' % (i + 1))
                return
        elif m.jog and ticks_diff(now, m.deadline) > 0:
            m.halt = True
        self.tick(m)
        if not m.hp:
            self.speed(m)
            if m.done >= m.n:
                self.mv = None
                out('#EVT:DONE')
                return
        iv = int(1000000 / m.v)
        t = ticks_add(m.next_t, iv)
        if ticks_diff(now, t) > iv:         # trễ quá thì bỏ qua, không dồn xung
            t = ticks_add(now, iv)
        m.next_t = t

    def tick(self, m):                      # 1 nhịp: các trục chia xung theo Bresenham
        n = m.n
        for i in m.axes:
            m.ac[i] += m.ad[i]
            if m.ac[i] >= n:
                m.ac[i] -= n
                p = self.step[i]
                p.value(1)
                time.sleep_us(2)
                p.value(0)
                self.pos[i] += m.sg[i]
        m.done += 1

    def speed(self, m):                     # hình thang: tăng tốc - đều - giảm tốc
        v = m.v
        if m.halt or (m.n - m.done) * 2 * m.a <= v * v:
            v -= m.a / v
            if v < m.v0:
                v = m.v0
            if m.halt and v <= m.v0:
                m.n = m.done
        elif v < m.vt:
            v = min(m.vt, v + m.a / v)
        elif v > m.vt:
            v = max(m.vt, v - m.a / v)
        m.v = v

    # ---------- xử lý lệnh ----------
    def handle(self, line, now):
        line = line.strip()
        if not line.startswith('#'):
            return
        cmd, _, data = line[1:].partition(':')
        a = data.split(',') if data else []
        try:
            r = self.run(cmd.upper(), a, now)
        except (ValueError, IndexError):
            r = 'ERR:ARG'
        out('#' + r)

    def run(self, c, a, now):
        if c == 'PING':
            return 'OK:PING'
        if c == 'START':
            self.mv = None
            if ESTOP_ENABLED and self.est.value() == ESTOP_ACTIVE:
                return 'ERR:ESTOP'
            self.estop = False
            return 'OK:START'
        if c == 'STOP':
            self.kill()
            return 'OK:STOP'
        if c == 'HALT':
            if self.mv:
                self.mv.halt = True
            return 'OK:HALT'
        if c == 'STAT':
            hm = sum(1 << i for i in range(N) if self.home[i].value() == HOME_ACTIVE)
            hd = sum(1 << i for i in range(N) if self.homed[i])
            return 'VALUE:%d,%d,%d,%d,%d,%d,%d,%d,%d' % (
                self.pos[0], self.pos[1], self.pos[2], self.pos[3],
                1 if self.mv else 0, 1 if self.enabled else 0, hm, 1 if self.estop else 0, hd)
        if c == 'EN':
            if a[0] == '1':
                if self.estop:
                    return 'ERR:ESTOP'
                self.set_en(True)
            else:
                self.kill()
            return 'OK:EN'
        if c == 'CFG':
            i = int(a[0]) - 1
            self.vmax[i], self.acc[i], self.inv[i] = float(a[1]), float(a[2]), int(a[3])
            return 'OK:CFG'
        if c == 'LIM':
            i = int(a[0]) - 1
            self.lo[i], self.hi[i] = int(a[1]), int(a[2])
            return 'OK:LIM'
        if c == 'ZERO':
            if self.mv:
                return 'ERR:BUSY'
            i = int(a[0])
            for k in (range(N) if i == 0 else (i - 1,)):
                self.pos[k] = 0
            return 'OK:ZERO'
        if c in ('MOVA', 'MOVR', 'JOG', 'HOME'):
            if self.estop:
                return 'ERR:ESTOP'
            if not self.enabled:
                return 'ERR:DISABLED'
            return self.motion(c, a, now)
        return 'ERR:CMD'

    def motion(self, c, a, now):
        m = self.mv
        if c == 'JOG':
            i, d, f = int(a[0]) - 1, int(a[1]), min(1.0, max(0.01, float(a[2])))
            if d == 0:
                if m and m.jog:
                    m.halt = True
                return 'OK:JOG'
            if m:
                if m.jog and m.axes[0] == i and m.sg[i] == (1 if d > 0 else -1) and not m.halt:
                    m.vt = f * self.vmax[i]
                    m.deadline = ticks_add(now, JOG_TIMEOUT)
                    return 'OK:JOG'
                return 'ERR:BUSY'
            dist = (self.hi[i] - self.pos[i]) if d > 0 else (self.pos[i] - self.lo[i])
            if dist <= 0:
                return 'ERR:LIMIT:%d' % (i + 1)
            dl = [0] * N
            dl[i] = d * dist
            return self.start(dl, f, now, jog=True) or 'OK:JOG'
        if m:
            return 'ERR:BUSY'
        if c == 'HOME':
            i, d, sps = int(a[0]) - 1, int(a[1]), float(a[2])
            self.start_home(i, d, max(VMIN, sps), now)
            return 'OK:HOME'
        f = min(1.0, max(0.01, float(a[4])))
        dl = [0] * N
        for i in range(N):
            if a[i] not in ('x', 'X', ''):
                v = int(a[i])
                dl[i] = v - self.pos[i] if c == 'MOVA' else v
        return self.start(dl, f, now) or 'OK:' + c


def run():
    import select
    mach = Machine()
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    buf = ''
    while True:
        now = ticks_us()
        mach.service(now)
        if poller.poll(0):
            ch = sys.stdin.read(1)
            if ch == '\n':
                mach.handle(buf, ticks_us())
                buf = ''
            elif ch != '\r' and len(buf) < 120:
                buf += ch


if __name__ == '__main__':
    run()