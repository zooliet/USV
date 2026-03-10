# import os
import subprocess
import time

# result = subprocess.run(
#     ["sudo", "sh", "cleanup.sh"], check=True, capture_output=True, text=True
# )
try:
    result = subprocess.run(
        ["supervisorctl", "stop", "all"],
        check=True,
        capture_output=True,
        text=True,
    )
except subprocess.CalledProcessError as e:
    print(f"Error stopping supervisor processes: {e}")
else:
    print(result.stdout)

processes_to_kill = [
    "micro_ros_agent",
    "localizer",
    "cmd_vel_joy",
    "twist_mux",
    "uwtec_agent",
    "nav_server"
]
for process in processes_to_kill:
    try:
        result = subprocess.run(
            ["pkill", "-9", process], check=True, capture_output=True, text=True
        )
    except subprocess.CalledProcessError as e:
        print(f"Error killing {process}: {e}")
        # print(f"Error killing {process}")
    else:
        print(f"{process}: killed")

time.sleep(3)
result = subprocess.run(
    ["sudo", "poweroff"], check=True, capture_output=True, text=True
)
