from langchain_google_genai import ChatGoogleGenerativeAI
from langchain.memory import ConversationBufferMemory
from langchain.agents import initialize_agent, AgentType
from langchain.tools import Tool
from ports.decision_making import DecisionMaker
from core.tool_registry import ToolRegistry


class GeminiLLMAdapter(DecisionMaker):
    """LLM adapter for Gemini models using LangChain agent + tools."""

    def __init__(self, tools: ToolRegistry, model="gemini-1.5-pro"):
        self.tools = tools
        self.model = model

        # Define tools for LangChain Agent
        langchain_tools = [
            Tool(
                name="home",
                func=lambda _: self.tools.home(),
                description="Return to home screen"
            ),

            Tool(
                name="set_timer",
                func=lambda seconds: self.tools.set_timer(int(seconds)),
                description="Set a timer in seconds"
            ),
            Tool(
                name="pause_timer",
                func=lambda _: self.tools.pause_timer(),
                description="Pause the timer"
            ),
            Tool(
                name="resume_timer",
                func=lambda _: self.tools.resume_timer(),
                description="Resume the timer"
            ),
            Tool(
                name="stop_timer",
                func=lambda _: self.tools.stop_timer(),
                description="Stop the timer"
            ),
            Tool(
                name="create_skill",
                func=lambda skill_name: self.tools.create_skill(skill_name),
                description="Create a new skill"
            ),
            Tool(
                name="start_skill_session",
                func=lambda skill_name: self.tools.start_skill_session(skill_name),
                description="Start a skill session"
            ),
            Tool(
                name="end_skill_session",
                func=lambda skill_name: self.tools.end_skill_session(skill_name),
                description="End a skill session"
            ),
            Tool(
                name="get_skills",
                func=lambda _: self.tools.get_skills(),
                description="List all skills"
            ),
            Tool(
                name="get_weather",
                func=lambda location: self.tools.get_weather(location),
                description="Get weather by location"
            ),
        ]

        # Create Gemini LLM wrapper
        llm = ChatGoogleGenerativeAI(model=self.model, temperature=0)

        memory = ConversationBufferMemory(
            memory_key="chat_history", 
            return_messages=True
        )

        # Initialize Agent with tools
        self.agent = initialize_agent(
            tools=langchain_tools,
            llm=llm,
            agent=AgentType.CHAT_CONVERSATIONAL_REACT_DESCRIPTION,
            memory=memory,
            verbose=True,
        )

    async def handle_input(self, user_input: str) -> str:
        """Process user input with Gemini agent."""
        result = await self.agent.ainvoke(user_input)
        return result["output"]
