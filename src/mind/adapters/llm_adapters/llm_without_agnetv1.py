from langchain_google_genai import ChatGoogleGenerativeAI
import dotenv
dotenv.load_dotenv()
# Custom/Local Imports
from mind.ports.decision_making_port import DecisionMaker
from langchain_ollama import ChatOllama
from mind.adapters.llm_adapters.stream_handler import StreamingResponseHandler


  

class GeminiLLMAdapter(DecisionMaker):
    """LLM adapter for Gemini models using LangChain agent + tools."""

    def __init__(self, model="gemini-2.0-flash"):  # Updated to a valid model (as of 2025)
        self.model = model

        # Create Gemini LLM wrapper (streaming enabled by default)
        self.llm = ChatGoogleGenerativeAI(
            model=self.model, 
            temperature=0,
            streaming=True  # Explicitly enable for .stream()
        )

    def handle_input(self, user_input: str) -> str:
        result = self.llm.invoke(user_input)
        return result.content

    async def stream(self, user_input: str):
        # Native streaming: Yields AIMessageChunk objects with .content
        for chunk in self.llm.astream(user_input):
            if chunk.content:  # Skip empty chunks
                yield chunk  # Yields full chunk (access .content in caller)
    

class OllamaAdapter(DecisionMaker):
    "use local models.."
    
    def __init__(self, model="smollm2:latest"):
        self.model = model
        self.llm = ChatOllama(model=self.model)

    def handle_input(self, user_input:str)-> str:
        return StreamingResponseHandler(self.llm, user_input)


    
if __name__ == "__main__":
    oladp = OllamaAdapter()
    print(oladp.handle_input("list out the alphabets."))