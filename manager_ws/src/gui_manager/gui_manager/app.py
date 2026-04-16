import ipaddress
import asyncio
import async_timeout
from redis.asyncio import Redis

import customtkinter as ctk
from customtkinter import CTk
from CTkMessagebox import CTkMessagebox

from async_tkinter_loop import async_handler
from async_tkinter_loop.mixins import AsyncCTk
from gui_manager.menu_frame import MenuFrame
from gui_manager.nav_frame import NavFrame
from gui_manager.setting_frame import SettingFrame


class App(CTk, AsyncCTk):
    def __init__(self, event_loop, redis_ip="192.168.0.1", debug=False):
        super().__init__()

        self.event_loop = event_loop
        self.redis_ip = redis_ip
        self.debug = debug

        # initialize Redis connection
        self.redis = Redis.from_url(f"redis://{self.redis_ip}", socket_timeout=5.0)
        # self.protocol("WM_DELETE_WINDOW", lambda: self.stop_application())

        # initialize state variables
        self.connected: bool = False
        self.ping_fails: int = 0

        # create a periodic timer task
        self.event_loop.create_task(self.periodic_3_sec_timer())
        self.redis_task = self.event_loop.create_task(self.read_redis())

    def setup_ui(self):
        # screen_width = self.winfo_screenwidth()
        # screen_height = self.winfo_screenheight()
        # self.geometry(f"{screen_width}x{screen_height}+0+0")
        # place the window on the center of the screen
        window_width = 1280
        window_height = 1024
        window_x = 0
        window_y = 0
        # window_x = (self.winfo_screenwidth() - window_width) * 1 // 3
        # window_y = (self.winfo_screenheight() - window_height) * 1 // 3
        #
        self.geometry(f"{window_width}x{window_height}+{window_x}+{window_y}")
        self.minsize(window_width, window_height)
        self.title("UWTEC USV Control GUI")

        # configure grid weights for responsive resizing
        # self.grid_columnconfigure((0, 2), weight=1)
        self.grid_columnconfigure((1), weight=6)
        self.grid_rowconfigure(0, weight=1)

        # menu frame
        self.menu_frame = MenuFrame(self)
        # nav frame
        self.nav_frame = NavFrame(self)
        # setting frame
        self.setting_frame = SettingFrame(self)

        # set default frame
        # self.select_frame_by_name("nav")
        #

    #     self.bind("<Configure>", self.on_window_configure)
    #
    # def on_window_configure(self, event):
    #     if event.widget == self:
    #         print(f"Window resized to: {event.width}x{event.height}")
    #         self.update_idletasks()
    #         self.update()

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
            except Exception as e:
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
                            await self.stop_application()
                            break
                        else:
                            await self.process(data)
                    await asyncio.sleep(0.01)
            # except asyncio.TimeoutError:
            except Exception as e:
                # print(f"(Reader) Timeout, no message received: {e}")
                pass

    async def process(self, data):
        cmd = data[0]
        params = [] if len(data) == 1 else data[1:]
        if cmd == "pong" and len(params) == 6:
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

    @async_handler
    async def stop_application(self):
        await self.redis.close()
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

    # def select_frame_by_name(self, name):
    #     selected_fg_color = ("#36719F", "#144870")
    #     fg_color = ("#3B8ED0", "#1F6AA5")
    #
    #     self.menu_frame.nav_button.configure(
    #         fg_color=selected_fg_color if name == "nav" else fg_color
    #     )
    #     self.menu_frame.setting_button.configure(
    #         fg_color=selected_fg_color if name == "setting" else fg_color
    #     )
    #     self.menu_frame.devel_button.configure(
    #         fg_color=selected_fg_color if name == "devel" else fg_color
    #     )
    #
    #     if name == "nav":
    #         self.nav_frame.tkraise()
    #     elif name == "setting":
    #         self.setting_frame.tkraise()
    #     # elif name == "devel":
    #     #     self.devel_frame.tkraise()
    #     #

    @async_handler
    async def poweroff(self):
        # await self.stop_application()
        await self.redis.publish("channel::agent", "poweroff")

    @async_handler
    async def reboot(self):
        # await self.stop_application()
        await self.redis.publish("channel::agent", "reboot")

    @async_handler
    async def connect(self):
        dialog = ctk.CTkInputDialog(text="Type in a IP address:", title="IP Address")

        # Standard CTkInputDialog size is approx 300x200
        width = 300
        height = 200

        # Calculate screen center
        # screen_width = dialog.winfo_screenwidth()
        # screen_height = dialog.winfo_screenheight()

        # x = int((screen_width / 2) - (width / 2))
        # y = int((screen_height / 2) - (height / 2))
        x = width // 2
        y = height // 2
        dialog.geometry(f"{width}x{height}+{x}+{y}")

        redis_ip = dialog.get_input()
        if redis_ip is None:
            return  # User cancelled the dialog

        try:
            redis_ip = ipaddress.ip_address(redis_ip)
        except ValueError:
            # Options for icon: "check", "info", "warning", "error", "question", "cancel"
            CTkMessagebox(
                master=self,
                title="Warning",
                message="Invalide IP address entered",
                icon="warning",
            )
        else:
            # print(redis_ip)
            # close existing Redis connection if any
            self.redis_task.cancel()
            await self.redis.close()

            # create a new Redis connection with the updated IP address
            self.redis = Redis.from_url(f"redis://{redis_ip}", socket_timeout=5.0)
            self.redis_task = self.event_loop.create_task(self.read_redis())
            # print(f"Updated Redis connection to {redis_ip}")
            #


def main():
    # ap = argparse.ArgumentParser()
    # ap.add_argument(
    #     "--redis-ip",
    #     type=ipaddress.ip_address,
    #     default="127.0.0.1",
    #     help="IP address to connect to (e.g., 192.168.1.1",
    # )
    # ap.add_argument(
    #     "--debug", action="store_true", help="Enable debug mode (default: False)"
    # )
    #
    # options, _ = ap.parse_known_args()
    # # args = vars(ap.parse_args())
    # args = vars(options)
    # print(args)
    #
    # redis_ip = args.get("redis_ip", "127.0.0.1")
    # redis_ip = ipaddress.ip_address(redis_ip)
    #
    # create a new event loop
    event_loop = asyncio.new_event_loop()
    # app = App(event_loop, redis_ip=redis_ip, debug=args.get("debug", False))
    app = App(event_loop)
    app.setup_ui()
    try:
        # call .async_mainloop() method instead of .mainloop()
        app.async_mainloop(event_loop)
    # except asyncio.CancelledError:
    #     print("Application closed.")
    except Exception as e:
        print(f"Application closed: {e}")


if __name__ == "__main__":
    main()
