from customtkinter import CTkFrame, CTkButton, CTkLabel, CTkFont
from datetime import datetime


class SettingFrame(CTkFrame):
    def __init__(self, master):
        super().__init__(master, corner_radius=0, fg_color="transparent")

        self.grid(row=0, column=2, padx=(20, 20), pady=(0, 0), sticky="nsew")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure((10), weight=1)

    def setup_ui(self, master):
        for widget in self.winfo_children():
            widget.destroy()

        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.time_label = CTkLabel(
            self, text=f"현재 시간: {current_time}", font=CTkFont(size=14)
        )
        self.time_label.grid(row=1, column=0, padx=(20, 10), pady=(40, 10), sticky="nw")
        # self.devel_button = CTkButton(
        #     self,
        #     text="개발자 모드",
        #     height=40,
        #     state="disabled",
        #     command=lambda: master.select_frame_by_name("devel"),
        # )
        # self.devel_button.grid(row=11, column=0, padx=40, pady=(10, 40), sticky="s")
