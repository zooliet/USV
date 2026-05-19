from customtkinter import CTkFrame, CTkButton, CTkLabel, CTkFont
# from async_tkinter_loop import async_handler


class MenuFrame(CTkFrame):
    def __init__(self, master):
        super().__init__(master, corner_radius=0, fg_color="transparent")
        self.master = master

        self.grid(row=0, column=0, padx=(20, 20), pady=(0, 0), sticky="nsew")
        self.grid_rowconfigure((10), weight=1)

    def setup_ui(self, master):
        CTkLabel(self, text="위도:", font=CTkFont(size=14)).grid(
            row=1, column=0, padx=(20, 10), pady=(40, 10), sticky="ne"
        )
        CTkLabel(self, text="경도:", font=CTkFont(size=14)).grid(
            row=2, column=0, padx=(20, 10), pady=(10, 10), sticky="ne"
        )
        CTkLabel(self, text="헤딩:", font=CTkFont(size=14)).grid(
            row=3, column=0, padx=(20, 10), pady=(10, 10), sticky="ne"
        )
        CTkLabel(self, text="속도:", font=CTkFont(size=14)).grid(
            row=4, column=0, padx=(20, 10), pady=(10, 10), sticky="ne"
        )
        CTkLabel(self, text="위성 품질:", font=CTkFont(size=14)).grid(
            row=5, column=0, padx=(20, 10), pady=(10, 10), sticky="ne"
        )
        self.lat_label = CTkLabel(self, text="0.000000", font=CTkFont(size=14))
        self.lat_label.grid(row=1, column=1, padx=(10, 20), pady=(40, 10), sticky="nw")

        self.lon_label = CTkLabel(self, text="0.000000", font=CTkFont(size=14))
        self.lon_label.grid(row=2, column=1, padx=(20, 10), pady=10, sticky="nw")

        self.heading_label = CTkLabel(self, text="0.00°", font=CTkFont(size=14))
        self.heading_label.grid(row=3, column=1, padx=(20, 10), pady=10, sticky="nw")

        self.vel_label = CTkLabel(self, text="0.0 m/s", font=CTkFont(size=14))
        self.vel_label.grid(row=4, column=1, padx=(20, 10), pady=10, sticky="nw")

        self.gps_quality = CTkLabel(self, text="--", font=CTkFont(size=14))
        self.gps_quality.grid(row=5, column=1, padx=(20, 10), pady=10, sticky="nw")

        self.connect_button = CTkButton(
            self, text="연결", height=40, command=master.connect
        )
        self.connect_button.grid(
            row=12, column=0, columnspan=2, padx=40, pady=10, sticky="n"
        )

        self.align_button = CTkButton(
            self, text="방향 정렬", height=40, command=master.align_heading
        )
        self.align_button.grid(
            row=13,
            column=0,
            columnspan=2,
            padx=40,
            pady=(10, 40),
            sticky="n",
        )

        # self.setting_button = CTkButton(
        #     self, text="셋팅", height=40, command=master.open_settings
        # )
        # self.setting_button.grid(
        #     row=14, column=0, columnspan=2, padx=20, pady=10, sticky="n"
        # )

        self.reboot_button = CTkButton(
            self, text="재부팅", height=40, command=master.reboot
        )
        self.reboot_button.grid(
            row=21, column=0, columnspan=2, padx=40, pady=10, sticky="s"
        )

        self.poweroff_button = CTkButton(
            self, text="전원 끔", height=40, command=master.poweroff
        )
        self.poweroff_button.grid(
            row=22, column=0, columnspan=2, padx=40, pady=10, sticky="s"
        )

        self.exit_button = CTkButton(
            self, text="프로그램 종료", height=40, command=master.stop_application
        )
        self.exit_button.grid(
            row=23, column=0, columnspan=2, padx=40, pady=(10, 140), sticky="s"
        )

    def update_gps_info(self, lat, lon, heading, gps_quality, speed):
        self.lat_label.configure(text=f"{lat:.6f}")
        self.lon_label.configure(text=f"{lon:.6f}")
        self.heading_label.configure(text=f"{heading:.2f}°")
        self.vel_label.configure(text=f"{speed:.2f} m/s")
        self.gps_quality.configure(text=str(gps_quality))
