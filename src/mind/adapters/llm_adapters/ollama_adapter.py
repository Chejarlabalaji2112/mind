from mind.core.ports.decision_making_port import DecisionMaker
from langchain_ollama import ChatOllama
from mind.adapters.llm_adapters.stream_handler import StreamingResponseHandler
from mind.utils import setup_logger
from langchain.agents import create_agent
from langgraph.checkpoint.postgres.aio import AsyncPostgresSaver
from langgraph.checkpoint.memory import InMemorySaver
from .prompts import prompts
import requests


logger = setup_logger(__name__)

class OllamaAdapter(DecisionMaker):
    def __init__(
        self,
        model: str = "smollm2:latest",
        remote_base_url: str = "http://REMOTE_HOST:11434",
        timeout: int = 5,
        db_uri: str = "postgresql://hitomi:hitomi@localhost:5432/hitomi"
    ):
        self.model = model
        self.remote_base_url = remote_base_url.rstrip("/")
        self.timeout = timeout
        self.db_uri = db_uri
        self._checkpointer_cm = None
        # The actual usable checkpointer object
        self.checkpointer_pst = None
        self.in_memory_ckp = InMemorySaver()
        self.agents = None

    async def __aenter__(self):
        # Initialize checkpointer asynchronously
        self._checkpointer_cm = AsyncPostgresSaver.from_conn_string(self.db_uri)
        self.checkpointer_pst = await self._checkpointer_cm.__aenter__()
        await self.checkpointer_pst.setup()
        # Initialize the LLM agent
        self.agents = await self._init_llm_with_fallback()
        logger.debug("inside __aenter__")
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self._checkpointer_cm:
            await self._checkpointer_cm.__aexit__(exc_type, exc_val, exc_tb)

    # ---------- Ollama capability probe ----------
    def _remote_has_model(self) -> bool:
        try:
            response = requests.get(
                f"{self.remote_base_url}/api/tags",
                timeout=self.timeout
            )
            response.raise_for_status()

            models = {m["name"] for m in response.json().get("models", [])}
            return self.model in models
        except Exception as e:
            logger.debug(f"Ollama probe failed: {e}")
            return False

    # ---------- LLM initialization ----------
    async def _init_llm_with_fallback(self):
        if self._remote_has_model():
            logger.info(
                f"Using remote Ollama at {self.remote_base_url} "
                f"with model '{self.model}'"
            )
            model = ChatOllama(
                model=self.model,
                base_url=self.remote_base_url,
                keep_alive="-1m",
                temperature=0,
                streaming=True,
            )
        else:
            logger.info(
                "Remote Ollama unavailable or model missing. "
                "Falling back to local Ollama."
            )
            model = ChatOllama(
                model=self.model,
                temperature=0,
                streaming=True,
            )

        # Create agent with PostgresSaver checkpointer
        return (
            create_agent(  #main chat agent
            model=model,
            system_prompt=prompts["hitomi"],
            checkpointer=self.checkpointer_pst,
        ),
        create_agent(   #ask doubt agent
            model=model,
            system_prompt=prompts["ask_doubt"],
            checkpointer = self.in_memory_ckp
        )
        )

    # ---------- DecisionMaker interface ----------
    async def handle_input(self, user_input: str, ask_doubt=False, thread_id='1' ) -> str:
        # Use standard invoke with HumanMessage
        i = 1 if ask_doubt else 0
        from langchain.messages import HumanMessage
        result = self.agents[i].invoke({"messages": [HumanMessage(user_input)]}, {"configurable": {"thread_id": f"{thread_id}"}})
        return result["messages"][-1].content

    def input_handler(self):
        return StreamingResponseHandler(self.agents)
    
