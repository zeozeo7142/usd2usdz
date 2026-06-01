"""
Step 6: USD + 모든 에셋을 USDZ 아카이브로 패키징
build_combined_usd.py가 생성한 *_combined.usda를 입력으로 사용합니다.

사용법: python package_usdz.py --index 1 [--output-dir output]
출력:   output/ERTI1/260521_ERTI_1.usdz
"""

import os
import sys
import argparse

ERTI_MAP = {
    1: {"base": "260521_ERTI_1", "sub": "ERTI1"},
    2: {"base": "260521_ERTI_2", "sub": "ERTI2"},
    3: {"base": "260521_ERTI_3", "sub": "ERTI3"},
}


def package_usdz(index: int, output_dir: str):
    try:
        from pxr import UsdUtils, Usd
    except ImportError:
        print("[ERROR] pxr 모듈 없음.")
        sys.exit(1)

    info = ERTI_MAP[index]
    base = info["base"]
    sub  = info["sub"]
    erti_dir = os.path.join(output_dir, sub)

    combined_usda = os.path.join(erti_dir, f"{base}_combined.usda")
    usdz_out      = os.path.join(erti_dir, f"{base}.usdz")

    if not os.path.exists(combined_usda):
        print(f"[ERROR] combined USD 없음: {combined_usda}")
        print("  먼저 build_combined_usd.py를 실행하세요:")
        print(f"  python build_combined_usd.py --index {index}")
        sys.exit(1)

    print(f"[package_usdz] 입력: {combined_usda}")
    print(f"[package_usdz] 출력: {usdz_out}")

    # 패키징 전 에셋 목록 출력
    stage = Usd.Stage.Open(combined_usda)
    if stage:
        print("[package_usdz] 포함될 레이어:")
        for layer in stage.GetLayerStack():
            size_mb = 0
            if os.path.exists(layer.identifier):
                size_mb = os.path.getsize(layer.identifier) / (1024**2)
            print(f"  {layer.identifier} ({size_mb:.1f} MB)")

    # USDZ 생성
    success = UsdUtils.CreateNewUsdzPackage(
        assetPath=combined_usda,
        usdzFilePath=usdz_out
    )

    if success:
        size_mb = os.path.getsize(usdz_out) / (1024**2)
        print(f"[package_usdz] 성공: {usdz_out} ({size_mb:.1f} MB)")
    else:
        print("[package_usdz] USDZ 생성 실패")
        print("  대안: usdzip CLI 사용")
        print(f"  usdzip \"{usdz_out}\" \"{combined_usda}\"")
        sys.exit(1)

    return usdz_out


def validate_usdz(usdz_path: str):
    """usdchecker로 유효성 검사"""
    import subprocess
    print(f"\n[validate] usdchecker 실행: {usdz_path}")
    result = subprocess.run(
        ["usdchecker", usdz_path],
        capture_output=True, text=True
    )
    if result.returncode == 0:
        print("[validate] 유효성 검사 통과")
    else:
        print("[validate] 경고/오류 발견:")
        print(result.stdout)
        print(result.stderr)

    # 아카이브 내 파일 목록 확인
    import zipfile
    try:
        with zipfile.ZipFile(usdz_path, "r") as z:
            print("\n[validate] USDZ 아카이브 내 파일:")
            for info in z.infolist():
                print(f"  {info.filename} ({info.file_size / 1024:.1f} KB)")
    except Exception as e:
        print(f"[validate] 아카이브 목록 오류: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="USDZ 패키징")
    parser.add_argument("--index", type=int, choices=[1, 2, 3], required=True,
                        help="ERTI 인덱스 (1, 2, 3)")
    parser.add_argument("--output-dir", default="output", help="출력 루트 디렉토리")
    parser.add_argument("--validate", action="store_true",
                        help="패키징 후 usdchecker 자동 실행")
    parser.add_argument("--all", action="store_true",
                        help="ERTI 1~3 모두 처리")
    args = parser.parse_args()

    indices = [1, 2, 3] if args.all else [args.index]

    for idx in indices:
        usdz = package_usdz(idx, args.output_dir)
        if args.validate:
            validate_usdz(usdz)
