from mind.core.ports.decision_making_port import DecisionMaker
from mind.adapters.llm_adapters.stream_handler import StreamingResponseHandler
from mind.utils import setup_logger

from mind.adapters.llm_adapters.ollama.factory import OllamaAgentFactory
from mind.adapters.llm_adapters.langgraph_helpers.checkpoint_manager import CheckpointManager
from .prompts import prompts

logger = setup_logger(__name__)

AgentFactory = OllamaAgentFactory
class MainAdapter(DecisionMaker):
    def __init__(
        self,
        model: str = "smollm2:latest",
        remote_base_url: str = "http://REMOTE_HOST:11434",
        timeout: int = 5,
        db_uri: str = "postgresql://hitomi:hitomi@localhost:5432/hitomi",
        tools: list| None = None
    ):
        self.agent_factory = AgentFactory(
            model=model,
            remote_base_url=remote_base_url,
            timeout=timeout,
        )

        self.checkpoints = CheckpointManager(db_uri)
        self.agents = None
        self.tools = tools

    async def __aenter__(self):
        await self.checkpoints.start()

        self.agents = (
            self.agent_factory.create_agent(
                tools = self.tools,
                system_prompt=prompts["hitomi"],
                checkpointer=self.checkpoints.postgres,
            ),
            self.agent_factory.create_agent(
                system_prompt=prompts["ask_doubt"],
                checkpointer=self.checkpoints.memory,
            ),
        )

        logger.debug("MainAdapter initialized")
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.checkpoints.stop(exc_type, exc_val, exc_tb)

    # ---------- DecisionMaker interface ----------

    async def handle_input(
        self,
        user_input: str,
        ask_doubt: bool = False,
        thread_id: str = "1",
    ) -> str:
        from langchain.messages import HumanMessage

        agent = self.agents[1 if ask_doubt else 0]

        result = agent.invoke(
            {"messages": [HumanMessage(user_input)]},
            {"configurable": {"thread_id": thread_id}},
        )
        return result["messages"][-1].content

    def input_handler(self):
        return StreamingResponseHandler(self.agents)
