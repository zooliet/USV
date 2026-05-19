import ipaddress
import asyncio
import async_timeout
from redis.asyncio import Redis

import customtkinter as ctk
from customtkinter import CTk, CTkToplevel  # , CTkInputDialog
from CTkMessagebox import CTkMessagebox

from async_tkinter_loop import async_handler
from async_tkinter_loop.mixins import AsyncCTk
from gui_manager.menu_frame import MenuFrame
from gui_manager.nav_frame import NavFrame
from gui_manager.widgets import InputDialog

from typing import Optional, Tuple, Union


class CTkInputDialog(CTkToplevel):
    """
    Dialog with extra window, message, entry widget, cancel and ok button.
    For detailed information check out the documentation.
    """

    def __init__(
        self,
        text_color: Optional[Union[str, Tuple[str, str]]] = None,
        fg_color: Optional[Union[str, Tuple[str, str]]] = None,
        button_fg_color: Optional[Union[str, Tuple[str, str]]] = None,
        button_hover_color: Optional[Union[str, Tuple[str, str]]] = None,
        button_text_color: Optional[Union[str, Tuple[str, str]]] = None,
        entry_fg_color: Optional[Union[str, Tuple[str, str]]] = None,
        entry_border_color: Optional[Union[str, Tuple[str, str]]] = None,
        entry_text_color: Optional[Union[str, Tuple[str, str]]] = None,
        title: str = "CTkDialog",
        font: Optional[Union[tuple, ctk.CTkFont]] = None,
        label_text: str = "CTkDialog",
        entry_text: Optional[str] = None,
    ):
        super().__init__(fg_color=fg_color)

        self._fg_color = (
            ctk.ThemeManager.theme["CTkToplevel"]["fg_color"]
            if fg_color is None
            else self._check_color_type(fg_color)
        )
        self._text_color = (
            ctk.ThemeManager.theme["CTkLabel"]["text_color"]
            if text_color is None
            else self._check_color_type(button_hover_color)
        )
        self._button_fg_color = (
            ctk.ThemeManager.theme["CTkButton"]["fg_color"]
            if button_fg_color is None
            else self._check_color_type(button_fg_color)
        )
        self._button_hover_color = (
            ctk.ThemeManager.theme["CTkButton"]["hover_color"]
            if button_hover_color is None
            else self._check_color_type(button_hover_color)
        )
        self._button_text_color = (
            ctk.ThemeManager.theme["CTkButton"]["text_color"]
            if button_text_color is None
            else self._check_color_type(button_text_color)
        )
        self._entry_fg_color = (
            ctk.ThemeManager.theme["CTkEntry"]["fg_color"]
            if entry_fg_color is None
            else self._check_color_type(entry_fg_color)
        )
        self._entry_border_color = (
            ctk.ThemeManager.theme["CTkEntry"]["border_color"]
            if entry_border_color is None
            else self._check_color_type(entry_border_color)
        )
        self._entry_text_color = (
            ctk.ThemeManager.theme["CTkEntry"]["text_color"]
            if entry_text_color is None
            else self._check_color_type(entry_text_color)
        )

        self._user_input: Union[str, None] = None
        self._running: bool = False
        self._title = title
        self._label_text = label_text
        self._entry_text = entry_text
        self._font = font

        self.title(self._title)
        self.lift()  # lift window on top
        self.attributes("-topmost", True)  # stay on top
        self.protocol("WM_DELETE_WINDOW", self._on_closing)
        self.after(
            10, self._create_widgets
        )  # create widgets with slight delay, to avoid white flickering of background
        self.resizable(False, False)
        self.grab_set()  # make other windows not clickable

    def _create_widgets(self):
        self.grid_columnconfigure((0, 1), weight=1)
        self.grid_rowconfigure(0, weight=1)

        self._label = ctk.CTkLabel(
            master=self,
            width=300,
            wraplength=300,
            fg_color="transparent",
            text_color=self._text_color,
            text=self._label_text,
            font=self._font,
        )
        self._label.grid(row=0, column=0, columnspan=2, padx=20, pady=20, sticky="ew")

        self._entry = ctk.CTkEntry(
            master=self,
            width=230,
            fg_color=self._entry_fg_color,
            border_color=self._entry_border_color,
            text_color=self._entry_text_color,
            font=self._font,
            textvariable=ctk.StringVar(self, self._entry_text),
        )
        self._entry.grid(
            row=1, column=0, columnspan=2, padx=20, pady=(0, 20), sticky="ew"
        )

        self._ok_button = ctk.CTkButton(
            master=self,
            width=100,
            border_width=0,
            fg_color=self._button_fg_color,
            hover_color=self._button_hover_color,
            text_color=self._button_text_color,
            text="Ok",
            font=self._font,
            command=self._ok_event,
        )
        self._ok_button.grid(
            row=2, column=0, columnspan=1, padx=(20, 10), pady=(0, 20), sticky="ew"
        )

        self._cancel_button = ctk.CTkButton(
            master=self,
            width=100,
            border_width=0,
            # fg_color=("#D30000", "#8B0000"),
            # hover_color=("#BF0000", "#610000"),
            fg_color=self._button_fg_color,
            hover_color=self._button_hover_color,
            text_color=self._button_text_color,
            text="Cancel",
            font=self._font,
            command=self._cancel_event,
        )
        self._cancel_button.grid(
            row=2, column=1, columnspan=1, padx=(10, 20), pady=(0, 20), sticky="ew"
        )

        # set focus to entry with slight delay, otherwise it won't work
        self.after(150, lambda: self._entry.focus())
        self._entry.bind("<Return>", self._ok_event)

    def _ok_event(self):
        self._user_input = self._entry.get()
        self.grab_release()
        self.destroy()

    def _on_closing(self):
        self.grab_release()
        self.destroy()

    def _cancel_event(self):
        self.grab_release()
        self.destroy()

    def get_input(self):
        self.master.wait_window(self)
        return self._user_input


# class SettingWindow(CTkToplevel):
#     def __init__(self, master):
#         super().__init__(master)
#         self.master = master
#
#         self.title("설정 윈도우")
#
#         # Standard CTkInputDialog size is approx 300x200
#         width = 400
#         height = 400
#
#         # Calculate screen center
#         screen_width = master.winfo_screenwidth()
#         screen_height = master.winfo_screenheight()
#
#         x = int((screen_width / 2) - (width / 2))
#         y = int((screen_height / 2) - (height / 2))
#         self.geometry(f"{width}x{height}+{x}+{y}")
#
#         # Entry 1
#         self.label_1 = ctk.CTkLabel(self, text="First Name:")
#         self.label_1.pack(pady=(10, 0))
#         self.entry_1 = ctk.CTkEntry(self)
#         self.entry_1.pack(pady=5)
#
#         # Entry 2
#         self.label_2 = ctk.CTkLabel(self, text="Last Name:")
#         self.label_2.pack(pady=(10, 0))
#         self.entry_2 = ctk.CTkEntry(self)
#         self.entry_2.pack(pady=5)
#
#         # Submit Button
#         self.button = ctk.CTkButton(self, text="Submit", command=self.submit)
#         self.button.pack(pady=20)
#
#         self.user_data = None
#         self.grab_set()  # Make window modal
#
#     def submit(self):
#         self.user_data = (self.entry_1.get(), self.entry_2.get())
#         print("Inputs:", self.user_data)
#         self.destroy()


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
        self.grid_columnconfigure((1), weight=1)
        self.grid_rowconfigure(0, weight=1)

        # create a periodic timer task
        self.event_loop.create_task(self.periodic_3_sec_timer())
        self.redis_task = self.event_loop.create_task(self.read_redis())

    def setup_ui(self):
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        # self.geometry(f"{screen_width}x{screen_height}+0+0")
        # place the window on the center of the screen
        window_width = screen_width  # 1280
        window_height = screen_height  # 1024
        window_x = 0
        window_y = 0
        self.geometry(f"{window_width}x{window_height}+{window_x}+{window_y}")
        self.minsize(1280, 1024)
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

    # @async_handler
    # async def open_settings(self):
    #     if self.connected:
    #         dialog = SettingWindow(self)
    #         # self.wait_window(dialog)  # Wait for dialog to close
    #         # print("Inputs:", dialog.user_data)

    @async_handler
    async def poweroff(self):
        if self.connected:
            await self.redis.publish("channel::agent", "poweroff")

    @async_handler
    async def reboot(self):
        if self.connected:
            await self.redis.publish("channel::agent", "reboot")

    @async_handler
    async def align_heading(self):
        if self.connected:
            await self.redis.publish("channel::agent", "calibrate-gyro")


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
