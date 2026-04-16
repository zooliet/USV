from customtkinter import CTkFrame, CTkButton, CTkLabel, CTkFont


class SettingFrame(CTkFrame):
    def __init__(self, master):
        super().__init__(master, corner_radius=0, fg_color="transparent")

        self.grid(row=0, column=2, padx=(20, 20), pady=(0, 0), sticky="nsew")
        # self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure((10), weight=1)

        # self.devel_button = CTkButton(
        #     self,
        #     text="개발자",
        #     height=40,
        #     state="disabled",
        #     command=lambda: master.select_frame_by_name("devel"),
        # )
        # self.devel_button.grid(row=11, column=0, padx=20, pady=(10, 40), sticky="s")
