import os
from collections import deque
from PIL import Image, ImageTk
import geocoder

import asyncio
from async_tkinter_loop import async_handler

import tkinter as tk
from customtkinter import CTkFrame, CTkButton, CTkLabel, CTkFont, CTkSlider
import tkintermapview as tkmap

from gui_manager.wps_frame import WPSFrame


class MapFrame(tkmap.AsyncTkinterMapView):
    def __init__(self, master):
        super().__init__(master, corner_radius=0)
        self.master = master

        self.grid(
            row=0, column=0, columnspan=2, padx=(0, 0), pady=(0, 0), sticky="nsew"
        )
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

    def setup_ui(self, master):
        # map tile servers:
        # # (1) google normal
        # self.set_tile_server(
        #     "https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22
        # )
        # (2) google satellite
        self.set_tile_server(
            "https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22
        )
        # # (3) openstreetmap
        # self.map_frame.set_tile_server(
        #     "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png"
        # )

        # set initial map position and zoom level
        coords = geocoder.ip("me").latlng
        self.set_position(coords[0], coords[1])  # 현재 위치
        # self.set_address("대한민국 경기도 가평군 송산리")
        # self.set_position(37.719457, 127.525468)  # 송산리
        self.set_zoom(18)

        # add click event to map
        # self.map_frame.add_left_click_map_command(master.recenter_map)
        self.add_right_click_menu_command(
            "마커 추가", command=master.place_marker, pass_coords=True
        )


class ControlFrame(CTkFrame):
    def __init__(self, master):
        super().__init__(master, corner_radius=0, fg_color="transparent")

        self.grid(row=1, column=0, padx=0, pady=0, sticky="nsew")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)

    def setup_ui(self, master):
        self.start_button = CTkButton(
            self,
            text="주행",
            width=120,
            height=60,
            command=async_handler(master.start_wps),
        )
        self.start_button.grid(row=10, column=0, padx=40, pady=(40, 40), sticky="e")

        self.stop_button = CTkButton(
            self,
            text="중지",
            width=120,
            height=60,
            command=async_handler(master.stop_wps),
        )
        self.stop_button.grid(row=10, column=1, padx=40, pady=(40, 40), sticky="w")

        # CTkLabel(self, text="주행 속도", font=CTkFont(size=14)).grid(
        #     row=11, column=0, columnspan=2, padx=20, pady=(10, 0)
        # )
        # self.speed_slider = CTkSlider(
        #     self, from_=0, to=10, number_of_steps=5, width=200
        # )
        # self.speed_slider.grid(row=12, column=0, columnspan=2, padx=20, pady=(0, 40))


class NavFrame(CTkFrame):
    def __init__(self, master):
        super().__init__(master, corner_radius=0, fg_color="transparent")
        self.master = master

        self.initial_position = False  # Flag to track if initial position is set
        self.markers = []  # List to store marker references
        self.current_path_markers = deque(
            maxlen=100
        )  # Store recent path coordinates for display
        self.driving = False  # Flag to indicate if navigation is active

        current_path = os.path.dirname(os.path.abspath(__file__))
        print(current_path)
        self.red_icon = ImageTk.PhotoImage(
            Image.open(os.path.join(current_path, "config", "red.png")).resize((20, 20))
        )
        self.blue_icon = ImageTk.PhotoImage(
            Image.open(os.path.join(current_path, "config", "blue.png")).resize(
                (30, 30)
            )
        )

        self.grid(row=0, column=1, sticky="nsew")
        self.grid_columnconfigure((0, 1), weight=1)
        self.grid_rowconfigure(0, weight=3)

        self.after_idle(self.setup_ui)

    def setup_ui(self):
        self.map_frame = MapFrame(self)
        self.map_frame.setup_ui(self)

        self.control_frame = ControlFrame(self)
        self.control_frame.setup_ui(self)

        self.wps_frame = WPSFrame(self)
        self.wps_frame.setup_ui(self)

    def recenter_map(self, coords):
        # print(f"Left-clicked at: {coords}")
        # Update map center to clicked position
        self.map_frame.set_position(coords[0], coords[1])

    def place_marker(self, coords):
        # print(f"Right-clicked at: {coords}")
        # Place a marker at the right-clicked position
        marker = self.map_frame.set_marker(
            coords[0], coords[1], icon=self.blue_icon, command=self.marker_callback
        )
        self.markers.append(marker)  # Store the marker reference
        self.wps_frame.setup_ui(self)  # Update the wps frame with new waypoint info
        self.draw_path()  # Redraw the path with the new waypoint

    def marker_callback(self, marker):
        # print(f"Marker: {marker} was clicked.")
        # remove the marker from the self.markers list
        self.markers = [m for m in self.markers if m != marker]
        marker.delete()  # Remove the marker when clicked
        self.wps_frame.setup_ui(self)  # Update the wps frame with new waypoint info
        self.draw_path()  # Redraw the path with the new waypoint

    def update_position(self, coords):
        if not self.initial_position:
            self.initial_position = True
            self.map_frame.set_position(coords[0], coords[1])
            # if self.driving:
            print(f"Updating position to: {coords[0]}, {coords[1]}")
            marker = self.map_frame.set_marker(coords[0], coords[1], icon=self.red_icon)
            self.current_path_markers.append(marker)

    def draw_path(self):
        self.map_frame.delete_all_path()  # Clear existing paths

        if len(self.markers) < 2:
            return  # Need at least 2 waypoints to draw a path

        # Extract positions from markers
        path_coords = [(m.position[0], m.position[1]) for m in self.markers]

        # Draw the path on the map
        self.map_frame.set_path(path_coords)

    def clear_path(self):
        for marker in self.markers:
            marker.delete()  # Remove all markers from the map
        self.markers.clear()  # Clear the markers list

        for marker in self.current_path_markers:
            marker.delete()  # Remove current position markers from the map
        self.current_path_markers.clear()  # Clear the current path markers list

        self.initial_position = False  # Reset initial position flag

        self.map_frame.delete_all_path()  # Clear all paths from the map
        self.wps_frame.setup_ui(self)  # Update the wps frame

    async def start_wps(self):
        if self.markers and self.master.connected:
            self.driving = True
            print("Starting waypoint navigation...")
            cmd_string = "upload-wps:wps.yaml"
            for marker in self.markers:
                cmd_string += f":{marker.position[0]},{marker.position[1]}"
            # print(f"Publishing command to Redis: {cmd_string}")
            await self.master.redis.publish("channel::agent", cmd_string)
            await asyncio.sleep(1.0)  # Short delay to ensure command is processed
            await self.master.redis.publish("channel::agent", "nav-to-wps:wps.yaml")

    async def stop_wps(self):
        if self.master.connected:
            self.driving = False
            print("Stopping waypoint navigation...")
            await self.master.redis.publish("channel::agent", "nav-to-wps")
