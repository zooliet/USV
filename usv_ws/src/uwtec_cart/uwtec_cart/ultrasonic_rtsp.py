import argparse
import ipaddress
import cv2
import subprocess


def main(device_index=0, resolution="640x480", fps=10, ip_address="127.0.0.1"):
    # Initialize the capture card (replace 1 with your device index)
    cap = cv2.VideoCapture(device_index)

    # Set resolution for 1080p (1920x1080) or 720p (1280x720) or 4K (3840x2160)
    width, height = map(int, resolution.split("x"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Set framerate (30 or 60)
    cap.set(cv2.CAP_PROP_FPS, fps)

    # Stream settings
    rtsp_url = f"rtsp://{ip_address}:8554/live/stream"

    # FFmpeg command for RTSP output
    command = [
        "ffmpeg",
        "-y",  # Overwrite output files
        "-f",
        "rawvideo",  # Input format
        "-vcodec",
        "rawvideo",
        "-pix_fmt",
        "bgr24",  # OpenCV uses BGR format
        "-s",
        f"{width}x{height}",  # Resolution
        "-r",
        str(fps),  # Framerate
        "-i",
        "-",  # Read input from stdin pipe
        "-c:v",
        "libx264",  # Encode to H.264
        "-pix_fmt",
        "yuv420p",  # Required for many players
        "-preset",
        "ultrafast",  # Low latency encoding
        "-f",
        "rtsp",  # Output format
        rtsp_url,
    ]

    # Start the FFmpeg process
    process = subprocess.Popen(command, stdin=subprocess.PIPE)

    if not cap.isOpened():
        print("Cannot open capture card")
        exit()

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Stream ended or failed.")
            break

        # # Display the resulting frame
        # cv2.imshow("HDMI Stream", frame)

        # Write frame to FFmpeg stdin
        process.stdin.write(frame.tobytes())

        # Press 'q' to exit
        if cv2.waitKey(1) == ord("q"):
            break

    # Release the capture and close windows
    cap.release()
    cv2.destroyAllWindows()
    process.stdin.close()
    process.wait()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()

    ap.add_argument(
        "--ip-address",
        type=ipaddress.ip_address,
        default="127.0.0.1",
        help="IP address to connect to (e.g., 192.168.1.1)",
    )
    ap.add_argument(
        "--resolution",
        type=str,
        default="640x480",
        help="Resolution for streaming (e.g., 1920x1080, 1280x720, 3840x2160)",
    )

    ap.add_argument(
        "--fps",
        type=int,
        default=10,
        help="Framerate for streaming (e.g., 30 or 60)",
    )

    ap.add_argument(
        "--device-index",
        type=int,
        default=0,
        help="Device index for the capture card (default: 0)",
    )

    options, _ = ap.parse_known_args()
    # args = vars(ap.parse_args())
    args = vars(options)
    print(args)

    main(args["device_index"], args["resolution"], args["fps"], args["ip_address"])
