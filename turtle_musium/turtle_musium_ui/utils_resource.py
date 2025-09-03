# utils_resource.py (새 파일로 패키지에 두면 편함)
import os
from ament_index_python.packages import get_package_share_directory

_PKG = "turtle_musium"

def res_path(*parts: str) -> str:
    """share/<pkg>/resource/... 경로 생성"""
    base = get_package_share_directory(_PKG)
    return os.path.join(base, "resource", *parts)
