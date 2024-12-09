from fastapi import FastAPI, File, UploadFile
from fastapi.responses import JSONResponse
import shutil
from pathlib import Path
import uvicorn

app = FastAPI()

# 이미지 저장 경로 설정
UPLOAD_DIR = Path("./frame")
UPLOAD_DIR.mkdir(exist_ok=True)

@app.post("/upload-image/")
async def upload_image(file: UploadFile = File(...)):
    file_path = UPLOAD_DIR / file.filename
    try:
        with file_path.open("wb") as buffer:
            shutil.copyfileobj(file.file, buffer)
        return JSONResponse(content={"message": f"Image '{file.filename}' uploaded successfully!"}, status_code=200)
    except Exception as e:
        return JSONResponse(content={"error": str(e)}, status_code=500)



if __name__ == "__main__":
    uvicorn.run("frame_test_server:app", host="172.16.43.231", port=8080, reload=True) #"192.168.0.218"
