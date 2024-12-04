from fastapi import FastAPI, HTTPException, File, UploadFile, Form
from fastapi.responses import JSONResponse, StreamingResponse, FileResponse
from pydantic import BaseModel
from pathlib import Path
import shutil
import io
import json

app = FastAPI()

# 텍스트 데이터를 저장할 저장소
data_store = {"text": ""}

# 업로드된 이미지와 관련된 탐지 정보를 저장하는 딕셔너리
detection_store = {}

# 이미지 저장 경로 설정
UPLOAD_DIR = Path("./frame")
UPLOAD_DIR.mkdir(exist_ok=True)

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

@app.post("/upload-image/")
async def upload_image(
    file: UploadFile = File(...),  # 업로드된 이미지 파일
    detection_data: str = Form(...)  # 클라이언트에서 전송된 탐지 정보 (JSON 문자열)
):
    """
    이미지를 업로드하고 탐지 정보를 저장하는 엔드포인트
    """
    file_path = UPLOAD_DIR / file.filename
    try:
        # 이미지 파일 저장
        with file_path.open("wb") as buffer:
            shutil.copyfileobj(file.file, buffer)

        # 탐지 정보 처리
        detection_info = json.loads(detection_data)  # JSON 문자열을 Python 객체로 변환
        print("Detection Data Received:")
        print(detection_info)

        # 탐지 정보 저장 (파일 이름을 키로 저장)
        detection_store[file.filename] = detection_info

        # 성공 응답 반환
        return JSONResponse(content={"message": f"Image '{file.filename}' and detection data processed successfully."}, status_code=200)

    except Exception as e:
        # 에러 처리
        return JSONResponse(content={"error": str(e)}, status_code=500)

@app.get("/get_frame_with_info/{frame_name}")
async def get_frame_with_info(frame_name: str):
    """
    사용자 요청 시 저장된 이미지와 관련된 탐지 정보를 멀티파트 형식으로 반환
    """
    file_path = UPLOAD_DIR / frame_name

    # 파일 확인
    if not file_path.exists():
        raise HTTPException(status_code=404, detail=f"Frame '{frame_name}' not found.")

    # 탐지 정보 확인
    detection_info = detection_store.get(frame_name)
    if detection_info is None:
        raise HTTPException(status_code=404, detail=f"No detection data found for frame '{frame_name}'.")

    # 이미지 파일 읽기
    with file_path.open("rb") as image_file:
        image_data = image_file.read()

    # 멀티파트 응답 생성
    boundary = "boundary12345"
    body = (
        f"--{boundary}\r\n"
        f"Content-Type: application/json\r\n\r\n"
        f"{json.dumps({'detection_data': detection_info})}\r\n"
        f"--{boundary}\r\n"
        f"Content-Disposition: attachment; filename={frame_name}\r\n"
        f"Content-Type: image/jpeg\r\n\r\n"
    ).encode("utf-8") + image_data + f"\r\n--{boundary}--\r\n".encode("utf-8")

    headers = {
        "Content-Type": f"multipart/mixed; boundary={boundary}",
    }
    return StreamingResponse(io.BytesIO(body), headers=headers)

@app.get("/frame/{frame_name}")
async def serve_frame(frame_name: str):
    """
    저장된 프레임 이미지를 사용자에게 전송
    """
    try:
        file_path = UPLOAD_DIR / frame_name

        # 이미지 파일 존재 여부 확인
        if not file_path.exists():
            raise HTTPException(status_code=404, detail=f"Frame '{frame_name}' not found.")

        # 이미지 파일 반환
        return FileResponse(file_path)

    except Exception as e:
        # 에러 처리
        raise HTTPException(status_code=500, detail=f"Failed to serve frame: {e}")


def main():
    """
    FastAPI 서버 실행
    """
    import uvicorn
    uvicorn.run("server:app", host="0.0.0.0", port=8000, reload=True)

if __name__ == "__main__":
    main()
