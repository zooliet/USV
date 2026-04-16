# import argparse
# import ipaddress
import asyncio
from gui_manager.app import App


def main():
    # ap = argparse.ArgumentParser()
    # ap.add_argument(
    #     "--redis-ip",
    #     type=ipaddress.ip_address,
    #     default="127.0.0.1",
    #     help="IP address to connect to (e.g., 192.168.1.1",
    # )
    # ap.add_argument(
    #     "--debug", action="store_true", help="Enable debug mode (default: False)"
    # )
    #
    # options, _ = ap.parse_known_args()
    # # args = vars(ap.parse_args())
    # args = vars(options)
    # print(args)
    #
    # redis_ip = args.get("redis_ip", "127.0.0.1")
    # redis_ip = ipaddress.ip_address(redis_ip)
    #
    # create a new event loop
    event_loop = asyncio.new_event_loop()
    # app = App(event_loop, redis_ip=redis_ip, debug=args.get("debug", False))
    app = App(event_loop)
    app.setup_ui()
    try:
        # call .async_mainloop() method instead of .mainloop()
        app.async_mainloop(event_loop)
    # except asyncio.CancelledError:
    #     print("Application closed.")
    except Exception as e:
        print(f"Application closed: {e}")


if __name__ == "__main__":
    main()
