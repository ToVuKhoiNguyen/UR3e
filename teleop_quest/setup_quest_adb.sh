#!/bin/bash
# ==============================================================================
# Script tự động kết nối Meta Quest qua ADB Developer Mode (Cáp USB hoặc Wi-Fi)
# Hỗ trợ chuyển tiếp toàn bộ cổng WebXR, WebSocket và Viser vào localhost
# ==============================================================================

echo "=========================================================="
echo "    THIET LAP KET NOI META QUEST DEVELOPER MODE (ADB)"
echo "=========================================================="

# 1. Kiểm tra adb đã cài đặt chưa
if ! command -v adb &> /dev/null; then
    echo "[!] adb chua duoc cai dat tren he thong."
    echo "[*] Dang cai dat android-tools-adb..."
    sudo apt update && sudo apt install -y adb
fi

# 2. Khoi dong adb server
adb start-server

# 3. Kiem tra thiet bi Quest
echo "[*] Dang tim kiem kinh Meta Quest ket noi qua USB..."
DEVICES=$(adb devices | grep -v "List of devices attached" | grep -v "^$" | awk '{print $1}')

if [ -z "$DEVICES" ]; then
    echo "[!] Khong tim thay kinh Meta Quest nao duoc ket noi qua USB!"
    echo "    Vui long kiem tra:"
    echo "    1. Cap USB-C da cam chac chan giua kinh va may tinh."
    echo "    2. Developer Mode da bat tren app Meta Horizon."
    echo "    3. Deo kinh len mat va bam 'Allow USB Debugging' (Cho phep go loi USB)."
    exit 1
fi

echo "[+] Da tim thay thiet bi Quest: $DEVICES"

# 4. Thiet lap ADB Reverse Port Forwarding
echo "[*] Dang thiet lap ADB Reverse Port Forwarding..."
adb reverse tcp:8443 tcp:8443
adb reverse tcp:8444 tcp:8444
adb reverse tcp:8445 tcp:8445
adb reverse tcp:8080 tcp:8080

echo ""
echo "=========================================================="
echo "    KET NOI THANH CONG! HUONG DAN SU DUNG:"
echo "=========================================================="
echo "1. Tren kinh Meta Quest, mo Meta Quest Browser."
echo "2. Truy cap vao dia chi:"
echo "   https://localhost:8443"
echo "   (Khong can quan tam IP LAN, khong lo song Wi-Fi chap chon!)"
echo ""
echo "3. De xem man hinh va Debug truc tiep tu PC:"
echo "   Mo Chrome tren Ubuntu -> vao chrome://inspect/#devices -> bam 'inspect'"
echo "=========================================================="
