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
data_store = {"x": "", "y": "", "z": ""}

detection_info = [
    {
        "idx": 1,
        "label": "person",
        "confidence": 0.9094,
        "bounding_box": [462, 110, 851, 720],
        "depth_3d_coordinates": {
            "x": 0.0405,
            "y": 0.1353,
            "z": 1.691
        },
        "rgb_3d_coordinates": {
            "x": 292, "y": 238, "z": 0.56
        }
    },
    {
        "idx": 2,
        "label": "person",
        "confidence": 0.8246,
        "bounding_box": [421, 199, 622, 622],
        "depth_3d_coordinates": {
            "x": -0.2586,
            "y": 0.1606,
            "z": 2.15
        },
        "rgb_3d_coordinates": {
            "x": 521, "y": 195, "z": 0.57
        }
    },
    {
        "idx": 3,
        "label": "person",
        "confidence": 0.8753,
        "bounding_box": [400, 220, 700, 600],
        "depth_3d_coordinates": {
            "x": 0.1001,
            "y": 0.1402,
            "z": 1.73
        },
        "rgb_3d_coordinates": {
            "x'": 0.1189,
            "y'": 0.1450,
            "z'": 1.729
        }
    },
    {
        "idx": 4,
        "label": "person",
        "confidence": 0.8024,
        "bounding_box": [480, 150, 820, 720],
        "depth_3d_coordinates": {
            "x": -0.1203,
            "y": 0.1521,
            "z": 2.05
        },
        "rgb_3d_coordinates": {
            "x'": -0.1004,
            "y'": 0.1556,
            "z'": 2.0512
        }
    },
    {
        "idx": 5,
        "label": "person",
        "confidence": 0.9001,
        "bounding_box": [500, 190, 830, 740],
        "depth_3d_coordinates": {
            "x": 0.0895,
            "y": 0.1208,
            "z": 1.69
        },
        "rgb_3d_coordinates": {
            "x'": 0.0998,
            "y'": 0.1241,
            "z'": 1.6921
        }
    },
    {
        "idx": 6,
        "label": "person",
        "confidence": 0.8123,
        "bounding_box": [400, 200, 700, 650],
        "depth_3d_coordinates": {
            "x": -0.1894,
            "y": 0.1583,
            "z": 2.13
        },
        "rgb_3d_coordinates": {
            "x": 997, "y": 537, "z": 0.41
        }
    },
    {
        "idx": 7,
        "label": "person",
        "confidence": 0.8950,
        "bounding_box": [430, 210, 750, 690],
        "depth_3d_coordinates": {
            "x": 0.0507,
            "y": 0.1449,
            "z": 1.72
        },
        "rgb_3d_coordinates": {
            "x'": 0.0693,
            "y'": 0.1428,
            "z'": 1.7215
        }
    },
    {
        "idx": 8,
        "label": "person",
        "confidence": 0.8436,
        "bounding_box": [390, 170, 650, 600],
        "depth_3d_coordinates": {
            "x": -0.2154,
            "y": 0.1405,
            "z": 2.18
        },
        "rgb_3d_coordinates": {
            "x'": -0.2001,
            "y'": 0.1383,
            "z'": 2.1796
        }
    }
]


# 업로드된 이미지와 관련된 탐지 정보를 저장하는 딕셔너리
detection_store = {}

# 데이터 모델 정의
# BaseModel: Pydantic의 기본 클래스임. 이를 상속받아 데이터 모델을 정의
# Pydantic은 Python 데이터 유효성 검사와 설정 관리를 위한 라이브러리
# TextRequest 클래스는 클라이언트로부터 받은 데이터의 구조를 정의하고, 데이터의 유효성을 자동으로 검사
class TextRequest(BaseModel):
    x: Union[str, int, list, float] = None  # 문자열, 정수, 리스트 모두 허용
    y: Union[str, int, list, float] = None  # 문자열, 정수, 리스트 모두 허용
    z: Union[str, int, list, float] = None  # 문자열, 정수, 리스트 모두 허용
    # text: str
    # text: Optional[str] = None  # 문자열이 없을 수도 있음

@app.post("/xyz")
async def update_text(request: TextRequest): # request는 클라이언트로부터 받아오는 요청 매개변수임. 해당 매개변수를 TextRequest인스턴스에 넘겨서 유효성 검사를 진행
    """
    iOS로부터 텍스트를 업데이트하는 엔드포인트
    """
    print(request)
    try:
        data_store["x"] = request.x
        data_store["y"] = request.y
        data_store["z"] = request.z
        print("==recieve==")
        print("x: ", data_store["x"])
        print("y: ", data_store["y"])
        print("z: ", data_store["z"])
        return {"message": "Text updated", "x": request.x, "y": request.y, "z": request.z}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update text: {e}")

# @app.get("/get_text")
# async def get_text():
#     """
#     ROS 노드가 텍스트를 가져갈 수 있는 엔드포인트
#     """
#     return {"text": data_store["text"]}


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
        # detection_info = detection_store.get(frame_name)
        # if detection_info is None:
        #     raise HTTPException(status_code=404, detail=f"No detection data found for frame '{frame_name}'.")

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
    uvicorn.run("server:app", host="172.16.43.231", port=8000, reload=True) #host="0.0.0.0"

if __name__ == "__main__":
    main()
