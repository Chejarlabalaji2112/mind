from langchain_google_genai import ChatGoogleGenerativeAI
import dotenv
dotenv.load_dotenv()
# Custom/Local Imports
from mind.ports.decision_making import DecisionMaker


class GeminiLLMAdapter(DecisionMaker):
    """LLM adapter for Gemini models using LangChain agent + tools."""

    def __init__(self ,model="gemini-2.5-flash-lite"):
        self.model = model

        # Define tools for LangChain Agent


        # Create Gemini LLM wrapper
        self.llm = ChatGoogleGenerativeAI(model=self.model, temperature=0)



    def handle_input(self, user_input: str) -> str:
        result = self.llm.invoke(user_input)
        return result.content
