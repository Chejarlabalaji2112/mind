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

    async def astream(self, user_input, ask_doubt, chat_names, context=None) -> AsyncIterator[str]:
        self._config(ask_doubt, chat_names)

        if ask_doubt and context:
            try:
                # Check if doubt thread is new/empty
                doubt_config = {"configurable": {"thread_id": f"{self.chat_name}"}}
                doubt_state = await self.agents[1].aget_state(doubt_config)
                
                if not doubt_state.values:
                    # It's empty. Use provided context.
                    context_text = f"context: {context}"
                    user_input = f"{context_text}\n\nUser Question:\n{user_input}"
                    logger.info("Injected context into doubt agent.")
            except Exception as e:
                logger.error(f"Error injecting context: {e}")

        async for chunk, _ in self.agents[self._iagent].astream({"messages": [HumanMessage(user_input)]}, {"configurable": {"thread_id": f"{self.chat_name}"}}, stream_mode="messages" ):
            if isinstance(chunk, AIMessage) and chunk.content:
                yield chunk.content

    def get_full_response(self, user_input, ask_doubt, chat_names, context=None) -> str:
        # Could collect internally, but optional — FastAPI can choose not to use
        self._config(ask_doubt, chat_names)
        result = self.agents[self._iagent].invoke({"messages": [HumanMessage(user_input)]}, {"configurable": {"thread_id": f"{self.chat_name}"}})
        return result["messages"][-1].content


