from customtkinter import CTkFrame, CTkButton, CTkLabel, CTkFont
# from datetime import datetime


class WPSFrame(CTkFrame):
    def __init__(self, master):
        super().__init__(master, corner_radius=0, fg_color="transparent")

        self.grid(row=0, column=2, padx=(20, 20), pady=(0, 0), sticky="nsew")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure((10), weight=1)

    def setup_ui(self, master):
        for widget in self.winfo_children():
            widget.destroy()

        if master.markers:
            CTkLabel(
                self, text="Waypoints:", font=CTkFont(size=16, weight="bold")
            ).grid(row=0, column=0, padx=(20, 10), pady=(20, 10), sticky="nw")

            for i, marker in enumerate(master.markers):
                marker_label = CTkLabel(
                    self,
                    text=f"{i + 1}. {marker.position[0]:.6f}, {marker.position[1]:.6f}",
                    font=CTkFont(size=14),
                    justify="left",
                )
                marker_label.grid(
                    row=i + 1, column=0, padx=(20, 10), pady=(10, 0), sticky="nw"
                )

            self.clear_button = CTkButton(
                self,
                text="경로 지우기",
                height=40,
                command=master.clear_path,
            ).grid(
                row=10,
                column=0,
                padx=20,
                pady=(10, 10),
                sticky="s",
            )
