from customtkinter import CTkFrame, CTkButton, CTkLabel, CTkFont
# from datetime import datetime


class WPSFrame(CTkFrame):
    def __init__(self, master):
        super().__init__(master, corner_radius=0, fg_color="transparent")

        self.grid(row=0, column=1, rowspan=2, padx=(20, 20), pady=(0, 0), sticky="nsew")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure((100), weight=1)

    def setup_ui(self, master):
        for widget in self.winfo_children():
            widget.destroy()

        CTkLabel(self, text="Waypoints:", font=CTkFont(size=12, weight="bold")).grid(
            row=0, column=0, padx=(10, 10), pady=(20, 0), sticky="nw"
        )

        self.clear_button = CTkButton(
            self,
            text="경로 지우기",
            height=36,
            font=CTkFont(size=12),
            command=master.clear_path,
            state="disabled",
        )
        self.clear_button.grid(
            row=100,
            column=0,
            padx=0,
            pady=(10, 20),
            sticky="s",
        )

        if master.markers:
            for i, marker in enumerate(master.markers):
                marker_label = CTkLabel(
                    self,
                    text=f"{i + 1}. {marker.position[0]:.6f}, {marker.position[1]:.6f}",
                    font=CTkFont(size=10),
                    justify="left",
                )
                marker_label.grid(
                    row=i + 1, column=0, padx=(10, 10), pady=0, sticky="nw"
                )
            self.clear_button.configure(state="normal")
