from dotenv import load_dotenv
import os
# your existing import
from langchain_openai import AzureChatOpenAI

# load env and initialize LLM (you already had this)
load_dotenv()

azure_llm = AzureChatOpenAI(
    model="gpt-4o",
    azure_deployment=os.getenv("AZURE_DEPLOYMENT_NAME"),
    openai_api_key=os.getenv("AZURE_OPENAI_API_KEY"),
    azure_endpoint=os.getenv("AZURE_OPENAI_ENDPOINT"), 
)