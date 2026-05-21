import ipaddress
import asyncio
import async_timeout
from redis.asyncio import Redis

import customtkinter as ctk
from customtkinter import CTk, CTkToplevel, CTkInputDialog
from CTkMessagebox import CTkMessagebox

from async_tkinter_loop import async_handler
from async_tkinter_loop.mixins import AsyncCTk
from gui_manager.menu_frame import MenuFrame
from gui_manager.nav_frame import NavFrame
from gui_manager.widgets import InputDialog, CLIDialog


class App(CTk, AsyncCTk):
    def __init__(self, event_loop, redis_ip="192.168.0.1"):
        super().__init__()

        self.event_loop = event_loop
        self.redis_ip = redis_ip

        # initialize Redis connection
        self.redis = Redis.from_url(f"redis://{self.redis_ip}", socket_timeout=5.0)
        self.protocol("WM_DELETE_WINDOW", lambda: self.stop_application())

        # initialize state variables
        self.connected: bool = False
        self.ping_fails: int = 0

        self.latitude: float = 0.0
        self.longitude: float = 0.0
        self.heading: float = 0.0
        self.gps_quality: int = 0
        self.num_sats: int = 0
        self.speed: float = 0.0

        # configure grid weights for responsive resizing
        # self.grid_columnconfigure((0, 2), weight=1)
        self.grid_columnconfigure((0), weight=0)
        self.grid_columnconfigure((1), weight=1)
        self.grid_rowconfigure(0, weight=1)

        # create a periodic timer task
        self.event_loop.create_task(self.periodic_3_sec_timer())
        self.redis_task = self.event_loop.create_task(self.read_redis())

    def setup_ui(self):
        ctk.set_appearance_mode("dark")

        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        # self.geometry(f"{screen_width}x{screen_height}+0+0")
        # place the window on the center of the screen
        window_width = screen_width  # 1280
        window_height = screen_height  # 1024
        window_x = 0
        window_y = 0
        self.geometry(f"{window_width}x{window_height}+{window_x}+{window_y}")
        # self.minsize(1280, 1024)
        self.title("UWTEC USV Control GUI")

        # menu frame
        self.menu_frame = MenuFrame(self)
        self.menu_frame.setup_ui(self)

        # nav frame
        self.nav_frame = NavFrame(self)

    async def periodic_3_sec_timer(self):
        while True:
            self.ping_fails += 1
            if self.ping_fails > 3:
                self.connected = False
                self.menu_frame.connect_button.configure(state="normal")
                self.menu_frame.connect_button.configure(text="연결")
            #
            # self.localization_var.set(
            #     f"위도: {self.latitude:.4f}, 경도: {self.longitude:.4f}, 헤딩: {self.heading:.1f}°, GPS 품질: {self.gps_quality}, 위성 수: {self.num_sats}"
            # )
            #
            # heading_status = "Aligned" if self.heading_aligned else "Not Aligned"
            # connection_status = "Connected" if self.connected else "Disconnected"
            # self.status_var.set(
            #     f"연결 상태: {connection_status}, 헤딩 정렬: {heading_status}"
            # )
            #
            # self.devel_frame.update()
            #
            try:
                await self.redis.publish("channel::agent", "ping")
            except Exception as _:
                # print(f"Failed to publish ping message to Redis: {e}")
                pass
            await asyncio.sleep(3.0)  # Non-blocking sleep

    async def read_redis(self):
        pubsub = self.redis.pubsub()
        await pubsub.subscribe("channel::gui")

        while True:
            try:
                async with async_timeout.timeout(1):
                    message = await pubsub.get_message(ignore_subscribe_messages=True)
                    if message is not None:
                        data = message["data"].decode().lower().split(":")
                        # print(f"(Reader) Data Received: {data}")
                        if data[0] == "stop":
                            print("(Reader) STOP")
                            await pubsub.unsubscribe()
                            self.stop_application()
                            break
                        else:
                            await self.process(data)
                    await asyncio.sleep(0.01)
            except asyncio.TimeoutError:
                print("(Reader) Timeout, no message received")
            except Exception as e:
                print(f"(Reader) Exception: {e}")

    async def process(self, data):
        cmd = data[0]
        params = [] if len(data) == 1 else data[1:]
        if cmd == "pong" and len(params) == 7:
            # print("(App) Received pong from agent")
            self.connected = True
            self.menu_frame.connect_button.configure(text="연결됨")
            # self.menu_frame.connect_button.configure(state="disabled")
            self.ping_fails = 0

            self.latitude = float(params[0])
            self.longitude = float(params[1])
            self.heading = float(params[2])
            _ = params[3]
            self.gps_quality = int(params[4])
            self.num_sats = int(params[5])
            self.speed = float(params[6])

            self.menu_frame.update_gps_info(
                self.latitude,
                self.longitude,
                self.heading,
                self.gps_quality,
                self.speed,
            )
            self.nav_frame.update_position(coords=(self.latitude, self.longitude))

    @async_handler
    async def stop_application(self):
        await self.redis.aclose()
        print("Redis connection closed, stopping application")
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

    @async_handler
    async def connect(self):
        dialog = InputDialog(
            self,
            title="IP Address",
            label_text="Type in a IP address:",
            entry_text=str(self.redis_ip),
        )
        # dialog = CTkInputDialog(
        #     label_text="Type in a IP address:",
        #     title="IP Address",
        #     entry_text=self.redis_ip,
        # )

        # Standard CTkInputDialog size is approx 300x200
        width = 400
        height = 260

        # Calculate screen center
        screen_width = dialog.winfo_screenwidth()
        screen_height = dialog.winfo_screenheight()

        x = int((screen_width / 2) - (width / 2))
        y = int((screen_height / 2) - (height / 2))
        dialog.geometry(f"{width}x{height}+{x}+{y}")

        redis_ip = dialog.get_input()
        if redis_ip is None:
            return  # User cancelled the dialog

        try:
            self.redis_ip = ipaddress.ip_address(redis_ip)
            # print(redis_ip)
        except ValueError:
            # Options for icon: "check", "info", "warning", "error", "question", "cancel"
            CTkMessagebox(
                master=self,
                title="Warning",
                message="Invalide IP address entered",
                icon="warning",
            )
        else:
            self.connected = False

            # close existing Redis connection if any
            self.redis_task.cancel()
            await self.redis.aclose()

            # create a new Redis connection with the updated IP address
            self.redis = Redis.from_url(f"redis://{self.redis_ip}", socket_timeout=5.0)
            self.redis_task = self.event_loop.create_task(self.read_redis())

    @async_handler
    async def open_developer_mode(self):
        if self.connected:
            # dialog = CTkInputDialog(text="Enter CLI command:", title="Developer Mode")
            dialog = CLIDialog(
                self,
                title="Developer Mode",
                label_text="Enter CLI command:",
                value_list=[
                    "test-run:forward:10:0",
                    "test-run:turn:0:90",
                    "test-run:nav-to:20:10",
                    "test-run:stop",
                ],
            )

            # Standard CTkInputDialog size is approx 300x200
            width = 400
            height = 260

            # Calculate screen center
            screen_width = dialog.winfo_screenwidth()
            screen_height = dialog.winfo_screenheight()

            x = int((screen_width / 2) - (width / 2))
            y = int((screen_height / 2) - (height / 2))
            dialog.geometry(f"{width}x{height}+{x}+{y}")
            cmd = dialog.get_input()
            print(cmd)
            if cmd is not None:
                await self.redis.publish("channel::agent", f"{cmd}")
        else:
            CTkMessagebox(
                master=self,
                title="Warning",
                message="Not connected to agent. Please connect first.",
                icon="warning",
            )

    @async_handler
    async def poweroff(self):
        if self.connected:
            await self.redis.publish("channel::agent", "poweroff")
        else:
            CTkMessagebox(
                master=self,
                title="Warning",
                message="Not connected to agent. Please connect first.",
                icon="warning",
            )

    @async_handler
    async def reboot(self):
        if self.connected:
            await self.redis.publish("channel::agent", "reboot")
        else:
            CTkMessagebox(
                master=self,
                title="Warning",
                message="Not connected to agent. Please connect first.",
                icon="warning",
            )

    @async_handler
    async def align_heading(self):
        if self.connected:
            await self.redis.publish("channel::agent", "calibrate-gyro")
        else:
            CTkMessagebox(
                master=self,
                title="Warning",
                message="Not connected to agent. Please connect first.",
                icon="warning",
            )


def main():
    # create a event loop
    event_loop = asyncio.new_event_loop()

    # set the event loop as the current loop
    app = App(event_loop)

    # setup the UI
    app.setup_ui()
    try:
        # call .async_mainloop() method instead of .mainloop()
        app.async_mainloop(event_loop)
    except asyncio.CancelledError:
        print("Application closed.")
    except Exception as e:
        print(f"Application closed: {e}")


if __name__ == "__main__":
    main()
