import argparse
from typing import cast
import ipaddress
import asyncio
import async_timeout
from redis.asyncio import Redis

import customtkinter as ctk

ctk.set_default_color_theme(
    "dark-blue"
)  # Themes: "blue" (standard), "green", "dark-blue"
import tkinter as tk
from customtkinter import CTk, CTkButton, CTkFrame, CTkLabel, CTkFont

from async_tkinter_loop import async_handler
from async_tkinter_loop.mixins import AsyncCTk


class NavFrame(CTkFrame):
    def __init__(self, master):
        super().__init__(master, corner_radius=0, fg_color="gray20")
        self.nav_frame_label = CTkLabel(
            self,
            anchor="center",
            text="네비게이션 프레임",
            font=CTkFont(size=16, weight="bold"),
            text_color="white",
        )
        self.nav_frame_label.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")

        self.grid(row=0, column=1, sticky="nsew")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)


class SettingFrame(CTkFrame):
    def __init__(self, master):
        super().__init__(master, corner_radius=0, fg_color="gray20")

        self.setting_frame_label = CTkLabel(
            self,
            anchor="center",
            text="설정 프레임",
        )
        self.setting_frame_label.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")

        self.grid(row=0, column=1, sticky="nsew")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)


class DevelFrame(CTkFrame):
    def __init__(self, master):
        super().__init__(master, corner_radius=0, fg_color="gray20")
        self.master = master

        self.localization_label = CTkLabel(
            self,
            anchor="w",
            textvariable=master.localization_var,
            text="",
            font=CTkFont(size=16, weight="bold"),
        )
        self.localization_label.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")

        self.status_label = CTkLabel(
            self,
            anchor="w",
            textvariable=master.status_var,
            text="",
            font=CTkFont(size=16, weight="bold"),
        )
        self.status_label.grid(row=1, column=0, padx=20, pady=20, sticky="nsew")

        self.connect_button = CTkButton(self, text="연결", height=40)
        self.connect_button.grid(row=2, column=0, padx=20, pady=20, sticky="w")

        self.align_button = CTkButton(self, text="방향 정렬", height=40)
        self.align_button.grid(row=3, column=0, padx=20, pady=20, sticky="w")

        self.test_run_button = CTkButton(self, text="테스트 런", height=40)
        self.test_run_button.grid(row=4, column=0, padx=20, pady=20, sticky="w")

        self.grid(row=0, column=1, sticky="nsew")
        self.grid_columnconfigure(0, weight=1)
        # self.grid_rowconfigure(0, weight=1)

    def update(self):
        # This method can be called periodically to update the frame's content
        if self.master.connected:
            self.connect_button.configure(state="disabled")
        else:
            self.connect_button.configure(state="normal")


class App(CTk, AsyncCTk):  # add AsyncCTk as a second parent class
    def __init__(self, event_loop, redis_ip, debug=False):
        super().__init__()

        self.localization_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="")

        self.event_loop = event_loop
        self.redis_ip = redis_ip
        self.debug = debug

        # Initialize instance variables
        self.connected: bool = False
        self.ping_fails: int = 0
        self.heading_aligned: bool = False
        self.latitude: float = 0.0
        self.longitude: float = 0.0
        self.heading: float = 0.0
        self.gps_quality: int = 0
        self.num_sats: int = 0

        self.redis = Redis.from_url(f"redis://{self.redis_ip}")
        # self.protocol("WM_DELETE_WINDOW", lambda: self.stop_application())

        # center the window on the screen
        window_width = 1280
        window_height = 1024
        window_x = (self.winfo_screenwidth() - window_width) // 2
        window_y = (self.winfo_screenheight() - window_height) * 1 // 3

        self.geometry(f"{window_width}x{window_height}+{window_x}+{window_y}")
        self.minsize(window_width, window_height)
        self.title("UWTEC USV Control GUI")

    def setup_ui(self):
        # create menu frame
        self.menu_frame = CTkFrame(self, corner_radius=0, fg_color="gray25")
        self.menu_frame.grid(row=0, column=0, sticky="nsew")

        # create menu buttons
        self.nav_button = CTkButton(
            self.menu_frame,
            text="주행",
            height=40,
            command=lambda: self.select_frame_by_name("nav"),
        )
        self.nav_button.grid(row=0, column=0, padx=60, pady=(40, 10), sticky="n")

        self.setting_button = CTkButton(
            self.menu_frame,
            text="설정",
            height=40,
            command=lambda: self.select_frame_by_name("setting"),
        )
        self.setting_button.grid(row=1, column=0, padx=60, pady=(20, 10), sticky="n")

        self.devel_button = CTkButton(
            self.menu_frame,
            text="개발자 모드",
            height=40,
            command=lambda: self.select_frame_by_name("devel"),
        )
        self.devel_button.grid(row=2, column=0, padx=60, pady=(20, 10), sticky="n")

        if not self.connected:
            self.connect_button = CTkButton(self.menu_frame, text="연결", height=40)
            self.connect_button.grid(row=10, column=0, padx=20, pady=10, sticky="s")

        self.align_button = CTkButton(self.menu_frame, text="방향 정렬", height=40)
        self.align_button.grid(row=11, column=0, padx=20, pady=10, sticky="s")

        self.reboot_button = CTkButton(self.menu_frame, text="재부팅", height=40)
        self.reboot_button.grid(row=12, column=0, padx=20, pady=10, sticky="s")

        self.poweroff_button = CTkButton(self.menu_frame, text="전원 끔", height=40)
        self.poweroff_button.grid(row=13, column=0, padx=20, pady=(10, 40), sticky="s")

        # create main frames
        self.nav_frame = NavFrame(self)
        self.setting_frame = SettingFrame(self)
        self.devel_frame = DevelFrame(self)

        # set default frame
        self.select_frame_by_name("devel")

        # configure grid weights for responsive resizing
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.menu_frame.grid_rowconfigure(3, weight=1)

        # create a periodic timer task
        self.event_loop.create_task(self.periodic_3_sec_timer())
        self.event_loop.create_task(self.read_redis())

    async def stop_application(self):
        await self.redis.close()
        print("Redis connection closed, stopping application")
        # self.destroy()
        # 활성화된 모든 비동기 작업을 안전하게 종료
        tasks = [
            task for task in asyncio.all_tasks() if task is not asyncio.current_task()
        ]
        for task in tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass  # 취소된 태스크에 대한 처리를 무시

        self.destroy()  # App 종료

    def select_frame_by_name(self, name):
        self.nav_button.configure(
            fg_color=("gray75", "gray25") if name == "nav" else "transparent"
        )
        self.setting_button.configure(
            fg_color=("gray75", "gray25") if name == "setting" else "transparent"
        )
        self.devel_button.configure(
            fg_color=("gray75", "gray25") if name == "devel" else "transparent"
        )

        if name == "nav":
            self.nav_frame.tkraise()
        elif name == "setting":
            self.setting_frame.tkraise()
        elif name == "devel":
            self.devel_frame.tkraise()

    async def periodic_3_sec_timer(self):
        while True:
            self.ping_fails += 1
            if self.ping_fails > 3:
                self.connected = False
                self.connect_button.configure(
                    state="normal"
                )  # Enable the connect button

            self.localization_var.set(
                f"위도: {self.latitude:.4f}, 경도: {self.longitude:.4f}, 헤딩: {self.heading:.1f}°, GPS 품질: {self.gps_quality}, 위성 수: {self.num_sats}"
            )

            heading_status = "Aligned" if self.heading_aligned else "Not Aligned"
            connection_status = "Connected" if self.connected else "Disconnected"
            self.status_var.set(
                f"연결 상태: {connection_status}, 헤딩 정렬: {heading_status}"
            )

            self.devel_frame.update()

            try:
                await self.redis.publish("channel::agent", "ping")
            except Exception as e:
                pass
            await asyncio.sleep(3.0)  # Non-blocking sleep

    async def read_redis(self):
        # redis = Redis.from_url(f"redis://{self.redis_ip}")
        pubsub = self.redis.pubsub()
        await pubsub.subscribe("channel::gui")

        while True:
            try:
                async with async_timeout.timeout(1):
                    message = await pubsub.get_message(ignore_subscribe_messages=True)
                    if message is not None:
                        data = message["data"].decode().lower().split(":")
                        print(f"(Reader) Data Received: {data}")
                        if data[0] == "stop":
                            print("(Reader) STOP")
                            await pubsub.unsubscribe()
                            await self.stop_application()
                            break
                        else:
                            await self.process(data)
                    await asyncio.sleep(0.01)
            except asyncio.TimeoutError:
                print("(Reader) Timeout, no message received")
                pass

    async def process(self, data):
        cmd = data[0]
        params = [] if len(data) == 1 else data[1:]
        if cmd == "pong" and len(params) == 6:
            # print("(App) Received pong from agent")
            self.connected = True
            self.connect_button.configure(
                state="disabled"
            )  # Disable the connect button
            self.ping_fails = 0

            self.latitude = float(params[0])
            self.longitude = float(params[1])
            self.heading = float(params[2])
            _ = params[3]
            self.gps_quality = int(params[4])
            self.num_sats = int(params[5])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--redis-ip",
        type=ipaddress.ip_address,
        default="127.0.0.1",
        help="IP address to connect to (e.g., 192.168.1.1",
    )
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
    )

    options, _ = ap.parse_known_args()
    # args = vars(ap.parse_args())
    args = vars(options)
    print(args)

    redis_ip = args.get("redis_ip", "127.0.0.1")
    redis_ip = ipaddress.ip_address(redis_ip)

    # create a new event loop
    event_loop = asyncio.new_event_loop()
    app = App(event_loop, redis_ip=redis_ip, debug=args.get("debug", False))
    app.setup_ui()
    try:
        app.async_mainloop(
            event_loop
        )  # call .async_mainloop() method instead of .mainloop()
    except asyncio.CancelledError:
        pass


if __name__ == "__main__":
    main()
