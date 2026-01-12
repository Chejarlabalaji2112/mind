import os 
from mcp.server.fastmcp import FastMCP
from mind.utils import BASE_DIR

mcp = FastMCP("LocalFiles")


LOCAL_BASE_PATH = BASE_DIR / "memory/local_files"

@mcp.tool()
def list_files() -> list[str]:
    """List all files in the allowed directory."""
    return os.listdir(LOCAL_BASE_PATH)

@mcp.tool()
def read_file(filename: str) -> str:
    """Read the contents of a specific file."""
    # Security: Prevent directory traversal
    safe_path = os.path.join(LOCAL_BASE_PATH, os.path.basename(filename))
    
    try:
        with open(safe_path, "r", encoding="utf-8") as f:
            return f.read()
    except Exception as e:
        return f"Error reading file: {str(e)}"

if __name__ == "__main__":
    mcp.run()