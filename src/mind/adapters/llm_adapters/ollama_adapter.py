from mind.core.ports.decision_making_port import DecisionMaker
from langchain_ollama import ChatOllama
from mind.adapters.llm_adapters.stream_handler import StreamingResponseHandler
from mind.utils import setup_logger
import requests

logger = setup_logger(__name__)


class OllamaAdapter(DecisionMaker):
    def __init__(
        self,
        model: str = "smollm2:latest",
        remote_base_url: str = "http://REMOTE_HOST:11434",
        timeout: int = 5
    ):
        self.model = model
        self.remote_base_url = remote_base_url.rstrip("/")
        self.timeout = timeout

        self.llm = self._init_llm_with_fallback()

    # ---------- Ollama capability probe (NON-BLOCKING) ----------
    def _remote_has_model(self) -> bool:
        
        try:
            response = requests.get(
                f"{self.remote_base_url}/api/tags",
                timeout=self.timeout
            )
            response.raise_for_status()

            models = {
                model["name"]
                for model in response.json().get("models", [])
            }
            return self.model in models

        except Exception as e:
            logger.debug(f"Ollama probe failed: {e}")
            return False

    # ---------- LLM initialization ----------
    def _init_llm_with_fallback(self) -> ChatOllama:
        if self._remote_has_model():
            logger.info(
                f"Using remote Ollama at {self.remote_base_url} "
                f"with model '{self.model}'"
            )

            return ChatOllama(
                model=self.model,
                base_url=self.remote_base_url,
                keep_alive="-1m",
                temperature=0,
                streaming=True,
            )

        logger.info(
            "Remote Ollama unavailable or model missing. "
            "Falling back to local Ollama."
        )

        return ChatOllama(
            model=self.model,
            temperature=0,
            streaming=True,
        )

    # ---------- DecisionMaker interface ----------
    def handle_input(self, user_input: str) -> str:
        result = self.llm.invoke(user_input)
        return result.content

    def input_handler(self):
        return StreamingResponseHandler(self.llm)
