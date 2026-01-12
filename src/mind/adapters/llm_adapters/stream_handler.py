from typing import AsyncIterator
from langchain.messages import HumanMessage, AIMessage
from mind.utils import setup_logger

logger = setup_logger(__name__)

class StreamingResponseHandler:
    def __init__(self, agents):
        self.agents = agents
        self._cleared_doubt_history = False
        self._iagent = 0
        self.chat_name = None

    def _config(self, ask_doubt, chat_names):
        if ask_doubt:
            self._iagent = 1
            self._cleared_doubt_history = False
            self.chat_name = chat_names[1]
        else:
            if not self._cleared_doubt_history:
                self.agents[1].checkpointer.delete_thread(thread_id=self.chat_name)
            self._iagent = 0
            self.chat_name = chat_names[0]

    async def astream(self, user_input, ask_doubt, chat_names) -> AsyncIterator[str]:
        self._config(ask_doubt, chat_names)
        async for chunk, _ in self.agents[self._iagent].astream({"messages": [HumanMessage(user_input)]}, {"configurable": {"thread_id": f"{self.chat_name}"}}, stream_mode="messages" ):
            if isinstance(chunk, AIMessage) and chunk.content:
                yield chunk.content

    def get_full_response(self, user_input, ask_doubt, chat_names) -> str:
        # Could collect internally, but optional — FastAPI can choose not to use
        self._config(ask_doubt, chat_names)
        result = self.agents[self._iagent].invoke({"messages": [HumanMessage(user_input)]}, {"configurable": {"thread_id": f"{self.chat_name}"}})
        return result["messages"][-1].content


