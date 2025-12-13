from typing import AsyncIterator

class StreamingResponseHandler: #I think I need  to store this class in different file
    def __init__(self, llm, user_input: str):
        self.llm = llm
        self.user_input = user_input

    async def astream(self) -> AsyncIterator[str]:
        async for chunk in self.llm.astream(self.user_input):
            content = chunk.content if hasattr(chunk, 'content') else str(chunk)
            if content:
                yield content

    def get_full_response(self) -> str:
        # Could collect internally, but optional — FastAPI can choose not to use
        raise NotImplementedError("Use astream for efficiency")  

