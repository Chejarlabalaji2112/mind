from mind.core.ports.decision_making_port import DecisionMaker
from mind.utils import setup_logger
from mind.adapters.llm_adapters.ollama.factory import OllamaAgentFactory
from langgraph.checkpoint.memory import InMemorySaver
from .prompts import prompts

logger = setup_logger(__name__)

AgentFactory = OllamaAgentFactory
class UtilsAdapter(DecisionMaker):
    def __init__(
            self,
            model:str = "smollm2:latest",
            remote_base_url:str = "http://REMOTE_HOST:11434",
            timeout: int = 5,
            tools: list| None = None 
    ):
        self.agent_factory = AgentFactory(
            model=model,
            remote_base_url=remote_base_url,
            timeout=timeout,
        )

        self.agent = self.agent_factory.create_agent(
            tools = tools,
            system_prompt=prompts["utils_manager"],
            checkpointer=InMemorySaver(),
            streaming=False,    #in future if utils grow to handling events/ or etc then hostory has to be stored.
        )

        
        logger.debug("utilsAdapter initialized")

    async def handle_input(self, input:str):
        response = await self.agent.ainvoke({"messages":[{"role": "user", "content" : input}]})

        return response["messages"][-1].content