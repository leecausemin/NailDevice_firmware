#!/usr/bin/env bash
# 빌드 + 보드 플래시
# 첫 플래시 실패 시 APPROTECT recover 후 SoftDevice와 앱 재플래시

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARM_UNIX="$SCRIPT_DIR/ble_app_uart/pca10056/s140/armgcc"

# 실행 중인 Bash 환경에 따라 Windows 도구 경로 설정
if [[ -d "/mnt/c/Program Files (x86)/GnuWin32/bin" ]]; then
    # WSL Bash
    MAKE="/mnt/c/Program Files (x86)/GnuWin32/bin/make.exe"
    NRFJPROG="/mnt/c/Program Files/Nordic Semiconductor/nrf-command-line-tools/bin/nrfjprog.exe"

    # Windows make.exe가 이해할 수 있도록 C:/... 형식으로 변환
    ARM="$(wslpath -m "$ARM_UNIX")"

elif [[ -d "/c/Program Files (x86)/GnuWin32/bin" ]]; then
    # Git Bash
    MAKE="/c/Program Files (x86)/GnuWin32/bin/make.exe"
    NRFJPROG="/c/Program Files/Nordic Semiconductor/nrf-command-line-tools/bin/nrfjprog.exe"

    if command -v cygpath >/dev/null 2>&1; then
        ARM="$(cygpath -m "$ARM_UNIX")"
    else
        ARM="$ARM_UNIX"
    fi

else
    echo "오류: GnuWin32 Make 설치 경로를 찾지 못했습니다."
    exit 1
fi

GNU="GNU_INSTALL_ROOT=C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/"
VER="GNU_VERSION=10.3.1"

# 필요한 파일 확인
if [[ ! -f "$MAKE" ]]; then
    echo "오류: make.exe를 찾을 수 없습니다."
    echo "확인한 경로: $MAKE"
    exit 1
fi

if [[ ! -f "$NRFJPROG" ]]; then
    echo "오류: nrfjprog.exe를 찾을 수 없습니다."
    echo "확인한 경로: $NRFJPROG"
    exit 1
fi

if [[ ! -d "$ARM_UNIX" ]]; then
    echo "오류: ARM GCC 프로젝트 폴더를 찾을 수 없습니다."
    echo "확인한 경로: $ARM_UNIX"
    exit 1
fi

echo "== 환경 확인 =="
echo "Make     : $MAKE"
echo "nrfjprog : $NRFJPROG"
echo "Project  : $ARM"
echo ""

echo "== 빌드 + 앱 플래시 =="

if "$MAKE" -C "$ARM" flash "$GNU" "$VER"; then
    echo ">> 완료."
else
    echo ""
    echo "!! 첫 번째 플래시가 실패했습니다."
    echo "!! APPROTECT recover 후 SoftDevice와 앱을 다시 플래시합니다."

    "$NRFJPROG" -f nrf52 --recover

    "$MAKE" -C "$ARM" flash_softdevice "$GNU" "$VER"
    "$MAKE" -C "$ARM" flash "$GNU" "$VER"

    echo ">> 완료 (recover 경유)."
fi