import asyncio
import signal


async def delay(time: float, function) -> any:
    if time > 0:
        await asyncio.sleep(time)
    return await function


def timer(_loop: asyncio.AbstractEventLoop, time: float, function):
    try:
        asyncio.run_coroutine_threadsafe(delay(time, function), _loop)
    except Exception as e:
        print(e)
        pass


def sync_timer(_loop: asyncio.AbstractEventLoop, time: float, function) -> any:
    try:
        f = asyncio.run_coroutine_threadsafe(delay(time, function), _loop)
        return f.result()
    except Exception as e:
        print(e)
        pass


def start_loop(loop: asyncio.BaseEventLoop):
    asyncio.set_event_loop(loop)
    loop.run_forever()
