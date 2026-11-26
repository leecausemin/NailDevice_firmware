#!/usr/bin/env bash
# 폰 앱이 녹음한 파일을 PC로 가져온다.
#
# 사용법:  bash pull_recordings.sh
#
# 앱 저장 폴더(Android/data/...)는 Android 11+에서 파일탐색기나 USB(MTP)로
# 접근이 막혀 있다. adb로 꺼내는 게 사실상 유일한 방법이다.
set -e

ADB="$LOCALAPPDATA/Android/Sdk/platform-tools/adb.exe"
REMOTE="/sdcard/Android/data/com.example.naildeviceapp/files/recordings"
DEST="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/pc_receiver/from_phone"

# Git Bash가 /sdcard 를 Windows 경로로 바꿔버리는 것을 막는다.
export MSYS_NO_PATHCONV=1

if [ ! -x "$ADB" ]; then
  echo "adb를 찾을 수 없습니다: $ADB"
  exit 1
fi

if ! "$ADB" get-state >/dev/null 2>&1; then
  echo "폰이 연결되어 있지 않습니다. USB 연결과 'USB 디버깅 허용'을 확인하세요."
  exit 1
fi

mkdir -p "$DEST"
echo "=> $DEST 로 가져옵니다"
"$ADB" pull -a "$REMOTE/." "$DEST/"

echo
echo "가져온 파일:"
ls -lt "$DEST" | head -20
