#!/usr/bin/env python3
from threading import Thread
from pathlib import Path
import subprocess as sp
import socket
import serial
import time
import json
import os
import atexit

REPO_ROOT = Path(__file__).resolve().parents[1]
pts_dir = str(REPO_ROOT / '.runtime')
os.makedirs(pts_dir, exist_ok=True)
ROBOT_HOST = os.environ.get('ROS_CAR_HOST', 'micu-ros-car.local')


def wait_for_ports(paths, processes, timeout=5.0):
    """等待所有PTY创建完成，子进程失败或超时立即报告。"""
    deadline = time.monotonic() + timeout
    while True:
        if any(process.poll() is not None for process in processes):
            raise RuntimeError('socat exited before ports became ready')
        if all(path.exists() for path in paths):
            return
        if time.monotonic() >= deadline:
            raise TimeoutError('Timed out waiting for virtual serial ports')
        time.sleep(0.05)


with \
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s1, \
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s2:

    s1.bind((socket.gethostname(), 0))
    s2.bind((socket.gethostname(), 0))
    s1.settimeout(5)
    s2.settimeout(5)

    serial_ports = {
        "lidar": (f"{pts_dir}/pts0", f"{pts_dir}/pts1"),
        "chassis": (f"{pts_dir}/pts2", f"{pts_dir}/pts3"),
    }

    # 不覆盖其他运行实例或上次遗留的PTY链接。
    if any(os.path.lexists(path) for pair in serial_ports.values() for path in pair):
        raise RuntimeError('PTY paths already exist; stop the previous bridge and clear .runtime/pts0..pts3')

    def cleanup():
        for process in vserial_popens:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except sp.TimeoutExpired:
                    process.kill()
                    process.wait()
        for pair in serial_ports.values():
            for name in pair:
                path = Path(name)
                if path.is_symlink():
                    path.unlink()

    vserial_popens = []
    atexit.register(cleanup)
    #创建虚拟串口
    args = lambda item: ["socat", f"pty,raw,echo=0,link={item[0]}", f"pty,raw,echo=0,link={item[1]}"]
    for item in serial_ports.values():
        vserial_popens.append(sp.Popen(args(item), stdout=sp.DEVNULL, stderr=sp.DEVNULL))

    # launch文件使用固定的仓库运行路径，不在启动时改写源码。
    wait_for_ports([Path(path) for pair in serial_ports.values() for path in pair], vserial_popens)

    def connect_esp32_car(local_port, toaddr,device_type):
        message = json.dumps({
        "port": local_port, 
        "type": device_type  # "chassis" 或 "lidar"
    	})
        with socket.socket() as s:  #创建TCP客户端socket
            try:
                s.connect(toaddr)# TCP连接到ESP32的TCP服务器
                s.send(message.encode("ascii"))#发送JSON消息
                print(f"Sending JSON config: {message}")
                response = s.recv(1024)
                print(f"Received response from ESP32: {response.decode()}")
            except socket.error as e:
                raise RuntimeError(f"cannot found the dns or ip address: {toaddr[0]}") from e

    class Communication:
        def __init__(self, serial_port: str, tcpaddr, device_type: str) -> None:
            self.seri = serial.Serial(serial_port, timeout=0.01)
            self.tcpaddr = tcpaddr
            self.device_type = device_type
            self.last_addr = None

        def start(self):
            self.running = True
            ths = [Thread(target=self.socket_to_serial, daemon=True), Thread(target=self.serial_to_socket, daemon=True)]
            [th.start() for th in ths]
            return ths
        
        def stop(self):
            self.running = False

        def __enter__(self):
            return self.start()

        def __exit__(self, *_):
            self.stop()

        def socket_to_serial(self):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                # 1. 创建UDP套接字并绑定到随机端口
                sock.bind(('0.0.0.0', 0))
                port = sock.getsockname()[1]
                print(f"UDP socket bound to port: {port}")
                sock.settimeout(5)
                # 2. 通过TCP连接告诉ESP32这个UDP端口
                connect_esp32_car(port, self.tcpaddr, self.device_type)
                print(f"TCP connection established with {self.tcpaddr}")
                while self.running:
                    try:
                    	# 接收ESP32发来的数据，并写入串口
                        message, self.last_addr = sock.recvfrom(1024)                        
                        self.seri.write(message)
                        print(self.tcpaddr, self.device_type,len(message), hex(message[0]))
                    except socket.timeout:
                        connect_esp32_car(port, self.tcpaddr, self.device_type)
                        print(f"reconnect smart car: {self.tcpaddr}")

        def serial_to_socket(self):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                message = b""
                while self.running:
                    time.sleep(0.005)
                    #读取虚拟串口的数据，通过 UDP 发送到 ESP32。
                    message += tmp if (tmp := self.seri.read_all()) is not None else b""
                    if self.last_addr is not None and len(message) > 0:
                        print(f"[{self.device_type}] Read from serial: {message.hex()}")
                        sock.sendto(message, self.last_addr)
                        print(f"[{self.device_type}] Sent to ESP32: {message.hex()}")
                        message = b""

    time.sleep(1)

    comms = {
        "lidar": Communication(
            serial_ports["lidar"][0],
            (ROBOT_HOST, 8080),
            "lidar",
        ),
        "chassis": Communication(
            serial_ports["chassis"][0],
            (ROBOT_HOST, 8080),
            "chassis"
        )
    }

    thread_groups = {hostname: c.start() for hostname, c in comms.items()}

    for hostname, c in comms.items():
        print(f"{hostname} serial: {serial_ports[hostname][0]}")
    
    while True:
        time.sleep(0.1)
        for hostname, group in thread_groups.items():
            if not all(map(lambda x: x.is_alive(), group)):
                [c.stop() for c in comms.values()]
                raise RuntimeError(f"{hostname} pipe ended abnormally")
