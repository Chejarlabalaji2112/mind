from langgraph.checkpoint.postgres.aio import AsyncPostgresSaver
from langgraph.checkpoint.memory import InMemorySaver


class CheckpointManager:
    def __init__(self, db_uri: str | None):
        self.db_uri = db_uri
        self._cm = None
        self.postgres = None
        self.memory = InMemorySaver()

    async def start(self):
        if not self.db_uri:
            return

        self._cm = AsyncPostgresSaver.from_conn_string(self.db_uri)
        self.postgres = await self._cm.__aenter__()
        await self.postgres.setup()

    async def stop(self, exc_type=None, exc_val=None, exc_tb=None):
        if self._cm:
            await self._cm.__aexit__(exc_type, exc_val, exc_tb)
