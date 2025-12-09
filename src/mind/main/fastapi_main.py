import argparse
from fastapi import FastAPI
from fastapi.responses import FileResponse
import uvicorn
from dataclasses import dataclass

#constants
from mind.utils import BASE_DIR

@dataclass
class Args:
    """Arguments for configuring the root entry."""
    sim: bool = True
    scene:str = "empty"

    wake_up_on_start: bool = False

    fastapi_host: str = "0.0.0.0"
    fastapi_port: int = 8000

    localhost_only: bool = False

def create_app(args: Args) -> FastAPI:
    app = FastAPI()

    @app.get("/")
    def read_index():
        return FileResponse(f"{BASE_DIR}/adapters/fastapi_adapters/static/index.html")

    return app
    

def run_app(args: Args) -> None:
    """Run the FastAPI app with Uvicorn."""
    app    = create_app(args)
    config = uvicorn.config(app, host=args.fastapi_host, port=args.fastapi_port)
    server = uvicorn.serve(config)
    
def main() -> None:
    """Run the FastAPI app with Uvicorn"""
    default_args = Args()

    parser = argparse.ArgumentParser(description="Main entry of the mind.")
    parser.add_argument(
        "--sim",
        action="store_true",
    )
    parser.add_argument(
        "--scene",
        type=str,
        default=default_args.scene,
        help="Name of the scene to load (default: empty)",
    )
    parser.add_argument(
        "--wake-up-on-start",
        action="store_true",
        help="Wake up the robot on daemon start (default: False).",
    )

    parser.add_argument(
        "--fastapi-host",
        type=str,
        default=default_args.fastapi_host,
    )
    parser.add_argument(
        "--fastapi-port",
        type=int,
        default=default_args.fastapi_port,
    )
    parser.add_argument(
        "--localhost-only",
        action="store_true",
        help="Restrict the server to localhost only (default: False).",
    )
    args = parser.parse_args()

    run_app(Args(**vars(args)))







if __name__== "__main__":
    main()