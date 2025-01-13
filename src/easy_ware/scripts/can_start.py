import os
import subprocess

# 현재 스크립트의 디렉토리 경로 가져오기
current_dir = os.path.dirname(os.path.abspath(__file__))

# 실행할 파일 경로 설정
script_path = os.path.join(current_dir, "can_start.h")

# 파일 실행
try:
    # subprocess를 사용하여 실행
    result = subprocess.run(["gcc", script_path, "-o", "can_start", "&&", "./can_start"],
                            cwd=current_dir,
                            shell=True,
                            check=True,
                            text=True,
                            capture_output=True)
    # 출력 결과 표시
    print("Execution Output:\n", result.stdout)
    if result.stderr:
        print("Execution Errors:\n", result.stderr)
except subprocess.CalledProcessError as e:
    print("Failed to execute the script:")
    print("Error Output:\n", e.stderr)
