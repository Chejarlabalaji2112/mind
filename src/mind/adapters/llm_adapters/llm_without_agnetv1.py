
from mind.core.ports.decision_making_port import DecisionMaker
from langchain_ollama import ChatOllama
from mind.adapters.llm_adapters.stream_handler import StreamingResponseHandler
# from mind.adapters.llm_adapters.symbolic_handler import SymbolicResponseHandler
from langchain_core.globals import set_debug
  

set_debug(True)
class OllamaAdapter(DecisionMaker):
    "use local models.."
    
    def __init__(self, model:str ="smollm2:latest", base_url:str =""):
        self.model = model
        self.llm = ChatOllama(model=self.model)

    def handle_input(self, user_input:str)-> str:
        self.llm.invoke(user_input)
        pass #just to avoid error

    def input_handler(self)-> str:
        return StreamingResponseHandler(self.llm)


    
if __name__ == "__main__":
    oladp = OllamaAdapter()
    print(oladp.handle_input("list out the alphabets."))


