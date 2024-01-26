import asyncio

class Ever:
    def __init__(self, interval_seconds):
        self.interval_seconds = interval_seconds
    
    async def __aiter__(self):
        while True:
            s = asyncio.sleep(self.interval_seconds)
            yield None
            await s