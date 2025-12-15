from langchain_ollama import ChatOllama
from langchain.memory import ConversationBufferMemory
from langchain.agents import initialize_agent, AgentType
from langchain.tools import StructuredTool,Tool
from mind.ports.decision_making_port import DecisionMaker
from tools.tool_registry.tool_registry_old import ToolRegistry
from langchain.prompts import MessagesPlaceholder


class Smollm2Adapter(DecisionMaker):
    """LLM adapter for smollm2 models using LangChain agent + tools."""

    def __init__(self, tools: ToolRegistry, model="smollm2"):
        self.tools = tools
        self.model = model

        # Define tools for LangChain Agent
        langchain_tools = [
            Tool(
                name="home",
                func=lambda _ :self.tools.home(),
                description="To return to home screen. if there is any ongoing task, it will be stopped. ",
            ),

            StructuredTool.from_function(
                func=self.tools.timer_tool,
                name="timer_tool",
                description=(
                    "This is the timer tool. Actions: start, pause, resume, stop, reset, status. "
                    "If action is 'start', then duration in seconds is required as well. "
                ),
            ),

            StructuredTool.from_function(
                func=self.tools.stopwatch_tool,
                name="stopwatch_tool",
                description="Interact with a stopwatch. Actions: start, pause, resume, lap, stop, reset, status.",
            ),

            StructuredTool.from_function(
                func=self.tools.pomodoro_tool,
                name="PomodoroTool",
                description="Control a Pomodoro timer. Actions: start (pom_type can be 'short' or 'long') if pom_type is not given it always short, pause, resume, stop, reset, status.",
            ),
]

        # Create Smollm2 LLM wrapper
        llm = ChatOllama(model=self.model, temperature=0)

        memory = ConversationBufferMemory(
            memory_key="chat_history", 
            return_messages=True
        )

        # Initialize Agent with tools
        self.agent = initialize_agent(
            tools=langchain_tools,
            llm=llm,
            agent=AgentType.STRUCTURED_CHAT_ZERO_SHOT_REACT_DESCRIPTION , # Changed to support StructuredTools
            memory=memory,
            agent_kwargs={
                "extra_prompt_messages":[MessagesPlaceholder(variable_name="chat_history")]
            },
            )

    async def handle_input(self, user_input: str) -> str:
        """Process user input with Gemini agent."""
        result = await self.agent.ainvoke(user_input)
        return result.get("output")
