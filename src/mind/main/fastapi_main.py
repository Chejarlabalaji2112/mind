import os
import time
import uvicorn
import argparse
import threading
from mind.utils import BASE_DIR             #constants
from dataclasses import dataclass
from mind.core.agent import Agent
from fastapi import FastAPI, Request
from contextlib import asynccontextmanager
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from mind.utils.logging_handler import setup_logger
from mind.adapters.llm_adapters.llm_without_agnetv1 import OllamaAdapter
from mind.adapters.robot_controller_adapters.mujoco_robot_adapter import MujocoRobot


logger = setup_logger(__name__)

llm = None

@dataclass
class Args:
    """Arguments for configuring the root entry."""
    sim: bool = False
    scene:str = "empty"

    wake_up_on_start: bool = False

    fastapi_host: str = "0.0.0.0"
    fastapi_port: int = 8000

    localhost_only: bool = False

def create_app(args: Args) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        robot_controller = None
        try:
            if args.sim:
                robot_controller = MujocoRobot()
                app.state.robot_controller = robot_controller
                agent = Agent(llm, robot_controller)
                app.state.agent = agent
                sim_thread = threading.Thread(target=robot_controller.run, daemon=True, name="Mujocorobot_loop")
                app.state.sim_thread = sim_thread
                logger.info("created the sim robot thread and not started the thread yet.")
                sim_thread.start()
                logger.info("started the thread and waiting for viewer..")

                if robot_controller.wait_until_ready(timeout=10):
                    logger.info("Viewer is ready now ")
                else:
                    logger.warning("Timed out waiting for viewer") #should I use warning or error here.

            yield

        finally:
            if robot_controller:
                try:
                    logger.info("Shutting down robot_controller...")
                    robot_controller.stop()

                except Exception as e:
                    logger.error(f"Error closing app: {e}")




    app = FastAPI(lifespan=lifespan)
    app.mount("/static", StaticFiles(directory = os.path.join(BASE_DIR, "adapters/fastapi_adapters/static")), name="static")
    templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "adapters/fastapi_adapters/templates"))

    @app.get("/")
    def root(request: Request):
        return templates.TemplateResponse("index.html", {"request": request})
    
    @app.get("/favicon.ico")
    def favicon():
        return FileResponse(os.path.join(BASE_DIR, "adapters/fastapi_adapters/static/favicon.ico")) #use os for joining or fstrings

    return app
    

def run_app(args: Args) -> None:
    """Run the FastAPI app with Uvicorn."""
    app    = create_app(args)
    # config = uvicorn.config(app, host=args.fastapi_host, port=args.fastapi_port)
    # server = uvicorn.serve(config)
    try:
        uvicorn.run(app,host=args.fastapi_host, port=args.fastapi_port)
    except Exception as e:
        logger.error(f"error in run_app:{e}")


    
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