from fastapi import FastAPI
from fastapi.responses import FileResponse
import uvicorn

#constants
from mind.utils import BASE_DIR


app = FastAPI()

@app.get("/")
def read_index():
    return FileResponse(f"{BASE_DIR}/adapters/fastapi_adapters/static/index.html")

if __name__== "__main__":
    uvicorn.run("fastapi_main:app", host="0.0.0.0", port=8000, reload=True)