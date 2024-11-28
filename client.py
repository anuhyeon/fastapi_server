import requests

# 서버 URL
url = "http://192.168.0.218:8080/upload-image/"

# 업로드할 이미지 파일 경로
image_path = "./images/image.jpg"

# 이미지 전송
with open(image_path, "rb") as file:
    files = {"file": file}
    response = requests.post(url, files=files)

# 응답 출력
print(response.json())
