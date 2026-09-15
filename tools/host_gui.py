#!/usr/bin/env python3
"""STM32F407 双臂下位机图形化串口测试工具。"""
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("缺少 pyserial，请运行: python3 -m pip install pyserial") from exc

HELLO, ENABLE, STOP, GET_STATE, DISABLE, TARGET = 1, 2, 3, 4, 5, 0x10
ACK, STATE = 0x80, 0x81
CMD_NAME = {HELLO:"HELLO", ENABLE:"ENABLE", STOP:"STOP", GET_STATE:"GET_STATE",
            DISABLE:"DISABLE", TARGET:"TARGET"}
RESULT = {0:"成功",1:"CRC错误",2:"格式/版本错误",3:"未知命令",4:"长度错误",
          5:"参数越界",6:"状态不允许",7:"无效机械臂",8:"通信超时",
          9:"控制序列忙",10:"电机控制失败"}
STATE_NAME = {0:"DISABLED",1:"ENABLED",2:"STOPPED",3:"FAULT"}

def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xa001 if crc & 1 else crc >> 1
    return crc

def packet(cmd, seq, payload=b""):
    body = struct.pack("<BBHH", 1, cmd, seq, len(payload)) + payload
    return b"\xaa\x55" + body + struct.pack("<H", crc16(body))

class Link:
    def __init__(self, events):
        self.events, self.ser, self.seq = events, None, 1
        self.running = False
        self.lock = threading.Lock()

    def connect(self, port):
        self.close()
        self.ser = serial.Serial(port, 115200, timeout=.05)
        self.ser.reset_input_buffer()
        self.running = True
        threading.Thread(target=self.reader, daemon=True).start()

    def close(self):
        self.running = False
        old, self.ser = self.ser, None
        if old:
            try: old.close()
            except serial.SerialException: pass

    def send(self, cmd, payload=b""):
        if not self.ser or not self.ser.is_open:
            raise serial.SerialException("串口未连接")
        with self.lock:
            seq = self.seq
            self.seq = self.seq + 1 if self.seq < 0xffff else 1
            self.ser.write(packet(cmd, seq, payload))
            return seq

    def reader(self):
        buf = bytearray()
        while self.running and self.ser:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data: buf.extend(data)
                while True:
                    start = buf.find(b"\xaa\x55")
                    if start < 0:
                        buf[:] = buf[-1:] if buf[-1:] == b"\xaa" else b""
                        break
                    del buf[:start]
                    if len(buf) < 10: break
                    ver, cmd, seq, size = struct.unpack_from("<BBHH", buf, 2)
                    if ver != 1 or size > 256:
                        del buf[0]; continue
                    total = 10 + size
                    if len(buf) < total: break
                    raw = bytes(buf[:total]); del buf[:total]
                    if struct.unpack_from("<H", raw, total-2)[0] == crc16(raw[2:-2]):
                        self.events.put(("frame", (cmd, seq, raw[8:-2])))
                    else:
                        self.events.put(("log", "收到 CRC 错误帧"))
            except (serial.SerialException, OSError) as exc:
                if self.running: self.events.put(("error", str(exc)))
                break

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AIMotor F407 上位机测试")
        self.geometry("980x680")
        self.events, self.link = queue.Queue(), Link(None)
        self.link.events = self.events
        self.port, self.connected = tk.StringVar(), tk.StringVar(value="未连接")
        self.auto = tk.BooleanVar(value=True)
        self.arm = tk.StringVar(value="左臂")
        self.ctrl_state = tk.StringVar(value="未知")
        self.fault, self.valid, self.enabled = (tk.StringVar(value=x) for x in ("0x00","0x0000","0x0000"))
        self.pos = [[tk.StringVar(value="—") for _ in range(6)] for _ in range(2)]
        self.target = [tk.StringVar(value="0") for _ in range(6)]
        self.build_ui(); self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.quit_app)
        self.after(30, self.process_events)
        self.after(100, self.poll)

    def build_ui(self):
        root = ttk.Frame(self, padding=12); root.pack(fill="both", expand=True)
        bar = ttk.LabelFrame(root, text="串口连接", padding=10); bar.pack(fill="x")
        ttk.Label(bar,text="串口").pack(side="left")
        self.combo = ttk.Combobox(bar,textvariable=self.port,width=24,state="readonly")
        self.combo.pack(side="left",padx=6)
        ttk.Button(bar,text="刷新",command=self.refresh_ports).pack(side="left")
        self.conn_btn = ttk.Button(bar,text="连接",command=self.toggle)
        self.conn_btn.pack(side="left",padx=8)
        ttk.Label(bar,text="115200 / 8N1").pack(side="left",padx=8)
        ttk.Label(bar,textvariable=self.connected).pack(side="right")

        ctl = ttk.LabelFrame(root,text="控制",padding=10); ctl.pack(fill="x",pady=8)
        for text, cmd in (("HELLO",HELLO),("读取状态",GET_STATE)):
            ttk.Button(ctl,text=text,command=lambda c=cmd:self.send(c)).pack(side="left",padx=3)
        ttk.Button(ctl,text="使能",command=lambda:self.confirm(ENABLE,"确认执行全轴使能？")).pack(side="left",padx=3)
        ttk.Button(ctl,text="失能",command=lambda:self.confirm(DISABLE,"确认停止并失能全部电机？")).pack(side="left",padx=3)
        tk.Button(ctl,text="全局停止",bg="#c62828",fg="white",
                  command=lambda:self.send(STOP,b"\x00")).pack(side="left",padx=12)
        ttk.Checkbutton(ctl,text="100 ms 自动刷新/保活",variable=self.auto).pack(side="right")

        stat = ttk.LabelFrame(root,text="控制器状态",padding=10); stat.pack(fill="x")
        for name,var in (("状态",self.ctrl_state),("故障",self.fault),
                         ("有效轴",self.valid),("使能轴",self.enabled)):
            ttk.Label(stat,text=name+":").pack(side="left",padx=(10,2))
            ttk.Label(stat,textvariable=var,width=12).pack(side="left")

        mid = ttk.Frame(root); mid.pack(fill="x",pady=8)
        feedback = ttk.LabelFrame(mid,text="12轴反馈",padding=10)
        feedback.pack(side="left",fill="both",expand=True,padx=(0,5))
        for col,text in enumerate(("关节","左臂","右臂")):
            ttk.Label(feedback,text=text).grid(row=0,column=col,padx=14)
        for j in range(6):
            unit = "µm" if j < 3 else "µrad"
            ttk.Label(feedback,text=f"J{j+1} ({unit})").grid(row=j+1,column=0,sticky="w",pady=5)
            ttk.Label(feedback,textvariable=self.pos[0][j],width=15,anchor="e").grid(row=j+1,column=1)
            ttk.Label(feedback,textvariable=self.pos[1][j],width=15,anchor="e").grid(row=j+1,column=2)

        target = ttk.LabelFrame(mid,text="单臂目标",padding=10)
        target.pack(side="left",fill="both",expand=True,padx=(5,0))
        ttk.Combobox(target,textvariable=self.arm,values=("左臂","右臂"),
                     width=9,state="readonly").grid(row=0,column=0)
        ttk.Button(target,text="填入当前位置",command=self.copy_pos).grid(row=0,column=1,padx=5)
        for j in range(6):
            unit = "µm" if j < 3 else "µrad"
            ttk.Label(target,text=f"J{j+1} ({unit})").grid(row=j+1,column=0,sticky="w",pady=4)
            ttk.Entry(target,textvariable=self.target[j],width=18).grid(row=j+1,column=1)
        ttk.Button(target,text="发送 TARGET",command=self.send_target).grid(row=7,column=0,columnspan=2,sticky="ew",pady=10)
        ttk.Label(target,text="首次测试：填入当前位置，只小改一个轴。",foreground="#9a6700").grid(row=8,column=0,columnspan=2)

        box = ttk.LabelFrame(root,text="通信日志",padding=6); box.pack(fill="both",expand=True)
        self.logs = tk.Text(box,height=9,state="disabled"); self.logs.pack(fill="both",expand=True)

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.combo["values"] = ports
        if ports and self.port.get() not in ports: self.port.set(ports[0])

    def toggle(self):
        if self.link.ser:
            self.link.close(); self.connected.set("未连接"); self.conn_btn.config(text="连接")
            self.log("串口已断开"); return
        if not self.port.get(): messagebox.showwarning("提示","请选择串口"); return
        try:
            self.link.connect(self.port.get()); self.connected.set("已连接")
            self.conn_btn.config(text="断开"); self.log("串口已连接"); self.send(HELLO)
        except (serial.SerialException,OSError) as exc: messagebox.showerror("连接失败",str(exc))

    def send(self,cmd,payload=b"",quiet=False):
        try:
            seq=self.link.send(cmd,payload)
            if not quiet:self.log(f"TX #{seq} {CMD_NAME.get(cmd,hex(cmd))}")
        except (serial.SerialException,OSError) as exc:
            if not quiet:messagebox.showerror("发送失败",str(exc))

    def confirm(self,cmd,text):
        if messagebox.askyesno("确认",text):self.send(cmd)

    def copy_pos(self):
        side=0 if self.arm.get()=="左臂" else 1
        for j in range(6):
            if self.pos[side][j].get()!="—":self.target[j].set(self.pos[side][j].get())

    def send_target(self):
        try:
            values=[int(v.get().strip()) for v in self.target]
            if any(not -2**31 <= v < 2**31 for v in values):raise ValueError("超出 int32")
        except ValueError as exc:
            messagebox.showerror("目标无效",f"6个目标必须是整数：{exc}");return
        arm=0 if self.arm.get()=="左臂" else 1
        payload=struct.pack("<BBH6i",arm,0,0,*values)
        if messagebox.askyesno("确认 TARGET",f"向{self.arm.get()}发送目标？"):self.send(TARGET,payload)

    def process_events(self):
        try:
            while True:
                kind,value=self.events.get_nowait()
                if kind=="frame":self.handle(*value)
                elif kind=="error":
                    self.log("串口错误: "+value);self.link.close()
                    self.connected.set("连接异常");self.conn_btn.config(text="连接")
                else:self.log(value)
        except queue.Empty:pass
        self.after(30,self.process_events)

    def handle(self,cmd,seq,payload):
        if cmd==ACK and len(payload)==4:
            ack_seq,origin,result=struct.unpack("<HBB",payload)
            self.log(f"RX ACK #{ack_seq} {CMD_NAME.get(origin,hex(origin))}: {RESULT.get(result,hex(result))}")
        elif cmd==STATE and len(payload)==60:
            _,state,fault,valid,enabled,axis=struct.unpack_from("<HBBHHI",payload)
            joints=struct.unpack_from("<12i",payload,12)
            self.ctrl_state.set(STATE_NAME.get(state,str(state)));self.fault.set(f"0x{fault:02X}")
            self.valid.set(f"0x{valid:04X}");self.enabled.set(f"0x{enabled:04X}")
            for i,val in enumerate(joints):
                self.pos[i//6][i%6].set(str(val) if valid&(1<<i) else "—")
            self.connected.set(f"已连接 · axis 0x{axis:06X}")
        else:self.log(f"RX 未识别帧 cmd=0x{cmd:02X} len={len(payload)}")

    def poll(self):
        if self.auto.get() and self.link.ser:self.send(GET_STATE,quiet=True)
        self.after(100,self.poll)

    def log(self,text):
        self.logs.config(state="normal");self.logs.insert("end",f"[{time.strftime('%H:%M:%S')}] {text}\n")
        self.logs.see("end");self.logs.config(state="disabled")

    def quit_app(self):
        self.link.close();self.destroy()

if __name__=="__main__":
    App().mainloop()
