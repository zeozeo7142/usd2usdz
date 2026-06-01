"""
Step 3: 파일명 공백 제거 및 USD 내부 경로 정리
원본은 보존하고, output/ 폴더에 정리된 복사본을 생성합니다.

사용법: python fix_paths.py "<usd_file_path>" [--output-dir <dir>]
예시:  python fix_paths.py "USDZ\USDZ_ERTI1\lcc-usd-result\260521_ERTI 1.usd"
"""

import sys
import os
import shutil
import argparse

def sanitize_name(name: str) -> str:
    return name.replace(" ", "_")

def fix_usd_paths(usd_path: str, output_dir: str):
    try:
        from pxr import Usd, Sdf
    except ImportError:
        print("[ERROR] pxr 모듈 없음. Isaac Sim Python 또는 pip install usd-core 필요.")
        sys.exit(1)

    usd_path = os.path.abspath(usd_path)
    src_dir = os.path.dirname(usd_path)
    src_name = os.path.basename(usd_path)
    clean_name = sanitize_name(src_name)

    os.makedirs(output_dir, exist_ok=True)
    dst_usd = os.path.join(output_dir, clean_name)

    print(f"[fix_paths] 원본: {usd_path}")
    print(f"[fix_paths] 출력: {dst_usd}")

    # USD 파일 복사 후 내부 레이어 경로 수정
    shutil.copy2(usd_path, dst_usd)

    stage = Usd.Stage.Open(dst_usd)
    if not stage:
        print("[ERROR] USD Stage 열기 실패")
        sys.exit(1)

    root_layer = stage.GetRootLayer()
    modified = False

    # SdfLayer에서 외부 참조 경로 공백 제거
    for layer in stage.GetLayerStack():
        refs = layer.GetExternalReferences()
        for ref in refs:
            if " " in ref:
                clean_ref = sanitize_name(ref)
                # 레이어 내 문자열 치환 (저수준 edit)
                layer_content = layer.ExportToString()
                if ref in layer_content:
                    new_content = layer_content.replace(ref, clean_ref)
                    layer.ImportFromString(new_content)
                    print(f"  경로 수정: '{ref}' → '{clean_ref}'")
                    modified = True

                    # 참조된 파일도 output_dir에 복사 (공백 제거 이름으로)
                    ref_src = os.path.join(src_dir, ref)
                    ref_dst = os.path.join(output_dir, clean_ref)
                    if os.path.exists(ref_src) and not os.path.exists(ref_dst):
                        os.makedirs(os.path.dirname(ref_dst), exist_ok=True)
                        shutil.copy2(ref_src, ref_dst)
                        print(f"  파일 복사: {ref_src} → {ref_dst}")

    if modified:
        root_layer.Save()
        print("[fix_paths] USD 파일 저장 완료 (경로 수정됨)")
    else:
        print("[fix_paths] 공백 경로 없음. 파일만 복사됨.")

    print(f"[fix_paths] 완료 → {dst_usd}")
    return dst_usd


def fix_obj(obj_path: str, output_dir: str) -> str:
    obj_path = os.path.abspath(obj_path)
    clean_name = sanitize_name(os.path.basename(obj_path))
    dst = os.path.join(output_dir, clean_name)

    os.makedirs(output_dir, exist_ok=True)
    shutil.copy2(obj_path, dst)
    print(f"[fix_paths] OBJ 복사: {obj_path} → {dst}")

    # MTL 파일도 같이 복사
    mtl_src = obj_path.replace(".obj", ".mtl")
    if os.path.exists(mtl_src):
        mtl_dst = dst.replace(".obj", ".mtl")
        shutil.copy2(mtl_src, mtl_dst)
        print(f"[fix_paths] MTL 복사: {mtl_src} → {mtl_dst}")

    return dst


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="USD/OBJ 파일명 공백 제거 및 경로 정리")
    parser.add_argument("usd_path", help="처리할 USD 파일 경로")
    parser.add_argument("--obj-path", help="함께 정리할 OBJ 파일 경로 (선택)")
    parser.add_argument("--output-dir", default="output", help="출력 디렉토리 (기본: output)")
    args = parser.parse_args()

    # USD 파일에서 인덱스 추출하여 output 서브폴더 결정
    basename = os.path.basename(args.usd_path)
    # "260521_ERTI 1.usd" → "ERTI1"
    index = ""
    for part in basename.split():
        if part[0].isdigit() and len(part) <= 3:
            index = part.replace(".usd", "").replace(".obj", "")
    out_dir = os.path.join(args.output_dir, f"ERTI{index}") if index else args.output_dir

    fix_usd_paths(args.usd_path, out_dir)

    if args.obj_path:
        fix_obj(args.obj_path, out_dir)
    else:
        # USD와 같은 폴더 구조에서 OBJ 추측
        usd_dir = os.path.dirname(args.usd_path)
        parent_dir = os.path.dirname(usd_dir)
        guessed_obj = os.path.join(parent_dir, "mesh-files", basename.replace(".usd", ".obj"))
        if os.path.exists(guessed_obj):
            print(f"[fix_paths] OBJ 파일 자동 감지: {guessed_obj}")
            fix_obj(guessed_obj, out_dir)
        else:
            print("[fix_paths] OBJ 파일을 찾지 못했습니다. --obj-path 옵션으로 지정하세요.")
