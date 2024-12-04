from fastapi import FastAPI, HTTPException, File, UploadFile, Form
from pydantic import BaseModel 
from fastapi.responses import JSONResponse, FileResponse  # frame 전송
import shutil  # frame 전송
from pathlib import Path  # frame 전송
import uvicorn
import json
from typing import Optional, Union

app = FastAPI()

# 텍스트 데이터를 저장할 저장소
data_store = {"text": ""}

# 업로드된 이미지와 관련된 탐지 정보를 저장하는 딕셔너리
detection_store = {}

# 데이터 모델 정의
# BaseModel: Pydantic의 기본 클래스임. 이를 상속받아 데이터 모델을 정의
# Pydantic은 Python 데이터 유효성 검사와 설정 관리를 위한 라이브러리
# TextRequest 클래스는 클라이언트로부터 받은 데이터의 구조를 정의하고, 데이터의 유효성을 자동으로 검사
class TextRequest(BaseModel):
    text: Union[str, int, list, None] = None  # 문자열, 정수, 리스트 모두 허용
    # text: str
    # text: Optional[str] = None  # 문자열이 없을 수도 있음


@app.post("/update_text")
async def update_text(request: TextRequest): # request는 클라이언트로부터 받아오는 요청 매개변수임. 해당 매개변수를 TextRequest인스턴스에 넘겨서 유효성 검사를 진행
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
async def upload_image(
    file: UploadFile = File(...),  # 업로드된 이미지 파일
    detection_data: str = Form(...)  # 클라이언트에서 전송된 탐지 정보 (JSON 문자열)
):
    file_path = UPLOAD_DIR / file.filename
    try:
        # 이미지 파일 저장
        with file_path.open("wb") as buffer:
            shutil.copyfileobj(file.file, buffer)
        print('########')

        # 탐지 정보 처리
        detection_info = json.loads(detection_data)  # JSON 문자열을 Python 객체로 변환
        print('-=-=-=-=-=-=-=-=', detection_info, '-=--=-=-=-=-=-=-=-=-=')
        print("Detection Data Received:")

        # 탐지 정보 저장 (파일 이름을 키로 저장)
        detection_store[file.filename] = detection_info
        
        # for detection in detection_info:
            #print(f"idx: {detection['idx']} Label: {detection['label']}, Confidence: {detection['confidence']}, Bounding Box: {detection['bounding_box']}, 3d_point: {detection['rgb_3d_coordinates']}")

        # 성공 응답 반환
        return JSONResponse(content={"message": f"Image '{file.filename}' and detection data processed successfully."}, status_code=200)

    except Exception as e:
        # 에러 처리
        print("exception ====> error")
        return JSONResponse(content={"error": str(e)}, status_code=500)

@app.get("/get-frame/{frame_name}")
async def get_frame(frame_name: str):
    """
    사용자 요청 시 저장된 이미지와 관련된 탐지 정보를 반환
    """
    try:
        # 이미지 파일 경로
        file_path = UPLOAD_DIR / frame_name # pathlib을 사용하면 운영 체제에 따라 다른 파일 경로 구분자(\ 또는 /)를 자동으로 처리해 주므로 플랫폼 독립적인 코드 작성이 가능

        # 이미지 파일 존재 여부 확인
        if not file_path.exists():
            raise HTTPException(status_code=404, detail=f"Frame '{frame_name}' not found.")

        # 탐지 정보 가져오기
        detection_info = detection_store.get(frame_name)
        if detection_info is None:
            raise HTTPException(status_code=404, detail=f"No detection data found for frame '{frame_name}'.")

        # JSON 응답에 이미지 경로와 탐지 정보 포함
        return JSONResponse(content={
            "message": f"Frame '{frame_name}' and detection data retrieved successfully.",
            "detection_data": detection_info,
            "image_url": f"/frame/{frame_name}"
        })

    except Exception as e:
        # 에러 처리
        raise HTTPException(status_code=500, detail=f"Failed to retrieve frame and detection data: {e}")


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
    uvicorn.run("server:app", host="0.0.0.0", port=8000, reload=True)

if __name__ == "__main__":
    main()
