import tkinter as tk
from customtkinter import CTkFrame, CTkButton, CTkLabel, CTkFont, CTkSlider
import tkintermapview as tkmap
import geocoder


class MapFrame(tkmap.AsyncTkinterMapView):
    def __init__(self, master):
        super().__init__(
            master,
            corner_radius=0,
            # width=800,
            # height=600,
        )

        self.grid(
            row=0, column=0, columnspan=2, padx=(0, 0), pady=(0, 0), sticky="nsew"
        )
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)


class ControlFrame(CTkFrame):
    def __init__(self, master):
        super().__init__(master, corner_radius=0, fg_color="transparent")

        self.grid(row=1, column=0, padx=0, pady=0, sticky="nsew")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.status_var = tk.StringVar(value="위치: (0.000, 0.000)")
        status_var = "현재 위치: (0.000, 0.000), 뱡향: 18.0, 위성 품질: 5"
        self.status_var.set(status_var)

        self.coordinate_label = CTkLabel(
            self,
            text="",
            textvariable=self.status_var,
            font=CTkFont(size=16),
        )
        self.coordinate_label.grid(row=0, column=0, padx=20, pady=20, sticky="nw")


class NavFrame(CTkFrame):
    def __init__(self, master):
        super().__init__(master, corner_radius=0, fg_color="transparent")

        self.grid(row=0, column=1, sticky="nsew")
        self.grid_columnconfigure((0, 1), weight=1)
        self.grid_rowconfigure(0, weight=3)

        self.start_button = CTkButton(self, text="주행", width=120, height=60)
        self.start_button.grid(row=10, column=0, padx=20, pady=(40, 40))

        self.stop_button = CTkButton(self, text="중지", width=120, height=60)
        self.stop_button.grid(row=10, column=1, padx=20, pady=(40, 40))

        # CTkLabel(self, text="주행 속도", font=CTkFont(size=14)).grid(
        #     row=11, column=0, columnspan=2, padx=20, pady=(10, 0)
        # )
        # self.speed_slider = CTkSlider(
        #     self, from_=0, to=10, number_of_steps=5, width=200
        # )
        # self.speed_slider.grid(row=12, column=0, columnspan=2, padx=20, pady=(0, 40))

        self.after_idle(self.setup_ui)

    def setup_ui(self):
        self.map_frame = MapFrame(self)

        # map tile servers
        #
        # google normal
        # self.map_frame.set_tile_server(
        #     "https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22
        # )
        #
        # google satellite
        self.map_frame.set_tile_server(
            "https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22
        )
        # openstreetmap
        # self.map_frame.set_tile_server(
        #     "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png"
        # )

        # set initial map position and zoom level
        g = geocoder.ip("me")
        coords = g.latlng
        # self.map_frame.set_position(coords[0], coords[1])  # 현재 위치
        self.map_frame.set_position(37.719457, 127.525468)  # 송산리
        # self.map_frame.set_address("대한민국 경기도 가평군 송산리")
        self.map_frame.set_zoom(18)

        # add click event to map
        # self.map_frame.add_left_click_map_command(self.recenter_map)
        self.map_frame.add_right_click_menu_command(
            "마커 추가", command=self.place_marker, pass_coords=True
        )

    def recenter_map(self, coords):
        print(f"Left-clicked at: {coords}")
        # Update map center to clicked position
        self.map_frame.set_position(coords[0], coords[1])

    def place_marker(self, coords):
        print(f"Right-clicked at: {coords}")
        # Place a marker at the right-clicked position
        self.map_frame.set_marker(coords[0], coords[1], command=self.marker_callback)

    def marker_callback(self, marker):
        print(f"Marker: {marker} was clicked.")
        marker.delete()  # Remove the marker when clicked
