import requests
from langchain_ollama import ChatOllama
from langchain.agents import create_agent
from mind.utils import setup_logger

logger = setup_logger(__name__)


class OllamaAgentFactory:
    def __init__(
        self,
        model: str,
        remote_base_url: str,
        timeout: int = 5,
    ):
        self.model = model
        self.remote_base_url = remote_base_url.rstrip("/")
        self.timeout = timeout

    # ---------- remote probe ----------
    def _remote_has_model(self) -> bool:
        try:
            resp = requests.get(
                f"{self.remote_base_url}/api/tags",
                timeout=self.timeout,
            )
            resp.raise_for_status()
            models = {m["name"] for m in resp.json().get("models", [])}
            return self.model in models
        except Exception as e:
            logger.debug(f"Ollama probe failed: {e}")
            return False

    # ---------- public API ----------
    def create_agent(
        self,
        *,
        tools: list | None = None,
        system_prompt: str,
        checkpointer,

        streaming: bool = True,
        temperature: float = 0,
    ):
        if self._remote_has_model():
            logger.info(
                f"Using remote Ollama {self.remote_base_url} "
                f"with model '{self.model}'"
            )
            model = ChatOllama(
                model=self.model,
                base_url=self.remote_base_url,
                keep_alive="-1m",
                temperature=temperature,
                streaming=streaming,
            )
        else:
            logger.info("Falling back to local Ollama")
            model = ChatOllama(
                model=self.model,
                temperature=temperature,
                streaming=streaming,
            )

        return create_agent(
            model=model,
            tools = tools,
            system_prompt=system_prompt,
            checkpointer=checkpointer,
        )
