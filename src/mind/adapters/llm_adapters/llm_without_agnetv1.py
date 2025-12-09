from langchain_google_genai import ChatGoogleGenerativeAI
import dotenv
dotenv.load_dotenv()
# Custom/Local Imports
from mind.ports.decision_making_port import DecisionMaker
from langchain_ollama import ChatOllama


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
    
class OllamaAdapter(DecisionMaker):

    "use local models.."
    
    def __init__(self, model="smollm2:latest"):
        self.model = model

        self.llm = ChatOllama(model=self.model)

    def handle_input(self, user_input:str)-> str:
        result = self.llm.invoke(user_input)
        return result.content
    
if __name__ == "__main__":
    oladp = OllamaAdapter()
    print(oladp.handle_input("list out the alphabets."))
