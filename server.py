from fastapi import FastAPI, HTTPException, File, UploadFile
from pydantic import BaseModel 
from fastapi.responses import JSONResponse# frame 전송
import shutil# frame 전송
from pathlib import Path# frame 전송
import uvicorn

app = FastAPI()

# 텍스트 데이터를 저장할 저장소
data_store = {"text": ""}

# 데이터 모델 정의
class TextRequest(BaseModel):
    text: str

@app.post("/update_text")
async def update_text(request: TextRequest):
    """
    iOS로부터 텍스트를 업데이트하는 엔드포인트
    """
    try:
        data_store["text"] = request.text
        print(data_store["text"])
        return {"message": "Text updated", "text": request.text}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update text: {e}")

@app.get("/get_text")
async def get_text():
    """
    ROS 노드가 텍스트를 가져갈 수 있는 엔드포인트
    """
    return {"text": data_store["text"]}


# 이미지 저장 경로 설정
UPLOAD_DIR = Path("./frame")
UPLOAD_DIR.mkdir(exist_ok=True)

@app.post("/upload-image/")
async def upload_image(file: UploadFile = File(...)):
    file_path = UPLOAD_DIR / file.filename
    try:
        with file_path.open("wb") as buffer:
            shutil.copyfileobj(file.file, buffer)
        return JSONResponse(content={"message": f"Image '{file.filename}' 업로드 성공 아기모띠"}, status_code=200)
    except Exception as e:
        return JSONResponse(content={"error": str(e)}, status_code=500)

def main():
    uvicorn.run("server:app", host="0.0.0.0", port=8000, reload=True)

if __name__ == "__main__":
    main()
