from mind.core.ports.decision_making_port import DecisionMaker
from langchain_ollama import ChatOllama
from mind.adapters.llm_adapters.stream_handler import StreamingResponseHandler
import logging

logger = logging.getLogger(__name__)


class OllamaAdapter(DecisionMaker):
    def __init__(
        self,
        model: str = "smollm2:latest",
        remote_base_url: str = "http://REMOTE_HOST:11434",
        timeout: int = 5
    ):
        self.model = model
        self.remote_base_url = remote_base_url
        self.timeout = timeout

        self.llm = self._init_llm_with_fallback()

    def _init_llm_with_fallback(self):
        # 1️⃣ Try remote Ollama
        try:
            remote_llm = ChatOllama(
                model=self.model,
                base_url=self.remote_base_url,
                temperature=0,
                streaming=True,
                timeout=self.timeout
            )

            # 🔍 Force a lightweight call to verify connectivity
            remote_llm.invoke("ping")

            logger.info(f"Connected to remote Ollama at {self.remote_base_url}")
            return remote_llm

        except Exception as e:
            logger.warning(
                f"Remote Ollama unavailable, falling back to local. Reason: {e}"
            )

        # 2️⃣ Fallback to local Ollama
        local_llm = ChatOllama(
            model=self.model,
            temperature=0,
            streaming=True
        )

        logger.info("Using local Ollama")
        return local_llm

    def handle_input(self, user_input: str) -> str:
        result = self.llm.invoke(user_input)
        return result.content

    def input_handler(self):
        return StreamingResponseHandler(self.llm)
