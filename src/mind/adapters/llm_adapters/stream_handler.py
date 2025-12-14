from typing import AsyncIterator

class StreamingResponseHandler:
    def __init__(self, llm):
        self.llm = llm

    async def astream(self, user_input) -> AsyncIterator[str]:
        async for chunk in self.llm.astream(user_input):
            content = chunk.content if hasattr(chunk, 'content') else str(chunk)
            if content:
                yield content

    def get_full_response(self) -> str:
        # Could collect internally, but optional — FastAPI can choose not to use
        raise NotImplementedError("Use astream for efficiency")  

