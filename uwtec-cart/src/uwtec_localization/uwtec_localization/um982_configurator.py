import argparse
import serial
import time
import re
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class UM982_Configurator:
    def __init__(self, port, baudrate, cmd=None, profile=None):
        self.port = port
        self.baudrate = baudrate
        self.cmd = cmd
        self.profile = profile
        self.device = serial.Serial(self.port, self.baudrate)

    def configure(self):
        print(f"Configuring UM982 with port: {self.port}, baudrate: {self.baudrate}")
        if self.cmd:
            print(f"Applying command: {self.cmd}")
            # Here you would add the actual code to send the command to the device
            cmd_str = f"{self.cmd}\x0d\x0a".encode("utf-8")
            self.device.write(cmd_str)
            time.sleep(1)

            if self.cmd.lower().startswith("config") and re.findall(r"\d+", self.cmd):
                baudrate = int(re.findall(r"\d+", self.cmd)[0])
                # close current device
                self.device.close()
                time.sleep(1)
                # open device with new baudrate
                self.device = serial.Serial(self.port, baudrate)

            # save
            cmd_str = "SAVECONFIG\x0d\x0a".encode("utf-8")
            self.device.write(cmd_str)
            time.sleep(1)
            print(f"{self.cmd.upper()} Done\n")

        if self.profile:
            print(f"Applying profile: {self.profile}")
            # Here you would add the actual code to apply the profile settings

            package_name = "uwtec_localization"
            file_name = "gnss.yaml"
            try:
                share_directory = get_package_share_directory(package_name)
                config_path = os.path.join(share_directory, "config", file_name)
                with open(config_path, "r") as file:
                    configs = yaml.safe_load(file)
                    configs = configs[self.profile]
                    print(configs)

                    # first unlog all
                    cmd_str = "unlog\x0d\x0a".encode("utf-8")
                    self.device.write(cmd_str)

                    for i, cmd in enumerate(configs):
                        print(i, cmd)
                        cmd_str = f"{cmd}\x0d\x0a".encode("utf-8")
                        self.device.write(cmd_str)
                        time.sleep(1)

                    cmd_str = "SAVECONFIG\x0d\x0a".encode("utf-8")
                    self.device.write(cmd_str)
                    time.sleep(1)
                    print("YAML configuration Done\n")

            except Exception as e:
                print(f"Could not load YAML file: {e}")

    def change_speed(self, baudrate):
        cmd_str = f"CONFIG COM3 {baudrate}\x0d\x0a".encode("utf-8")
        self.device.write(cmd_str)
        time.sleep(1)

        # close device
        self.device.close()
        time.sleep(1)
        # open device with new baudrate
        self.device = serial.Serial(self.port, baudrate)
        # save
        cmd_str = "SAVECONFIG\x0d\x0a".encode("utf-8")
        self.device.write(cmd_str)
        print(f"CONFIG {baudrate} Done\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default="/dev/ttyGNSS", help="GNSS device path")
    ap.add_argument(
        "-b", "--baudrate", type=int, required=True, help="current baudrate"
    )
    ap.add_argument("--cmd", help="configuration command")
    ap.add_argument("--profile", help="profile name in config/gnss.yaml")

    args = vars(ap.parse_args())
    print(args)
    configurator = UM982_Configurator(**args)
    configurator.configure()


if __name__ == "__main__":
    main()
