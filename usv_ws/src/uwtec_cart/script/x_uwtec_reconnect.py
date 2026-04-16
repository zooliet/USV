import asyncio
import async_timeout
import os

# import aioredis
from redis.asyncio import Redis
from redis.asyncio.client import PubSub
# Or, if using a pool:
# from redis.asyncio import ConnectionPool

STOPWORD = "STOP"


async def reader(channel: PubSub):
    while True:
        try:
            async with async_timeout.timeout(1):
                message = await channel.get_message(ignore_subscribe_messages=True)
                if message is not None:
                    # print(f"(Reader) Message Received: {message}")
                    if message["data"].decode() == "connect":
                        print("(Reader) RECONNECT")
                        # os.system("ls -al")
                        # executable_py = os.path.join(
                        #     get_package_share_directory("uwtec_agent"),
                        #     "script",
                        #     "uwtec_poweroff.py",
                        # )
                        # executable_py = ""
                        # os.system(f"python {executable_py}")
                        # os.system("zsh restart_supervisor.sh")
                        # check if file exists
                        # [[ ! -f /tmp/agent_node.pid ]] &&
                        if not os.path.exists("/tmp/agent_node.pid"):
                            os.system("supervisorctl restart all")
                        else:
                            print("(Reader) Agent node is already running.")

                    # elif message["data"].decode() == STOPWORD:
                    #     print("(Reader) STOP")
                    #     break
                await asyncio.sleep(0.01)
        except asyncio.TimeoutError:
            pass


async def main():
    redis = Redis.from_url("redis://localhost")
    pubsub = redis.pubsub()
    await pubsub.subscribe("channel::agent")
    future = asyncio.create_task(reader(pubsub))

    # await redis.publish("channel::agent", "Hello")
    # await redis.publish("channel:1", STOPWORD)

    await future


if __name__ == "__main__":
    asyncio.run(main())
