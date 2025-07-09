#!/bin/bash
set -e
cd "$(dirname "$0")"

export LC_ALL=C.UTF-8
export LANG=C.UTF-8

if [ "$EUID" -ne 0 ]; then
  echo "Пожалуйста, запускайте скрипт с sudo."
  exit 1
fi

if dpkg -s python3-dev &>/dev/null; then
    echo "✅ python3-dev уже установлен."
else
    echo "🔧 Устанавливаем python3-dev..."
    apt update
    apt install -y python3-dev
fi

if dpkg -s git &>/dev/null; then
    echo "✅ git уже установлен."
else
    echo "🔧 Устанавливаем git..."
    apt update
    apt install -y git
fi

git clone https://github.com/OnisOris/gion

echo "Выберите версию Python для виртуального окружения:"
echo "1) 3.10"
echo "2) 3.11"
echo "3) 3.12"
echo "4) 3.13"
read -p "Введите номер [1-4]: " PY_VER_CHOICE

case "$PY_VER_CHOICE" in
    1) PYTHON_VER="3.10" ;;
    2) PYTHON_VER="3.11" ;;
    3) PYTHON_VER="3.12" ;;
    4) PYTHON_VER="3.13" ;;
    *) echo "Неверный выбор. Прерывание установки."; exit 1 ;;
esac

echo "Используется Python версии $PYTHON_VER"

REAL_USER=$(logname)
REAL_HOME=$(eval echo "~$REAL_USER")
REAL_PATH="$REAL_HOME/.local/bin:$PATH"

echo "Пользователь: $REAL_USER"
echo "Домашняя директория: $REAL_HOME"

if sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c 'command -v uv &>/dev/null'; then
    echo "✅ uv уже установлен. Установка не требуется."
else
    echo "🔧 uv не найден. Устанавливаю..."
    sudo -u "$REAL_USER" bash -c 'curl -LsSf https://astral.sh/uv/install.sh | sh'
fi


INSTALL_DIR="$REAL_HOME/gion"
VENV_DIR="$INSTALL_DIR/.venv"


echo "Создаём виртуальное окружение..."
sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c "\"$REAL_HOME/.local/bin/uv\" venv --python $PYTHON_VER --prompt pion \"$VENV_DIR\""



echo "Устанавливаем pionsdk с Git (ветка dev)..."
sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c "source \"$VENV_DIR/bin/activate\" && \"$REAL_HOME/.local/bin/uv\" pip install \"git+https://github.com/OnisOris/gion.git\""

echo "Создаём systemd unit файл /etc/systemd/system/gion.service..."

cat > /etc/systemd/system/gion.service << EOF
[Unit]
Description=Pion Autostart Service
After=network.target geobot.service

[Service]
Type=simple
ExecStart=$VENV_DIR/bin/python -m gion.__main__
WorkingDirectory=$INSTALL_DIR
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
User=$REAL_USER

[Install]
WantedBy=multi-user.target
EOF


echo "Перезагружаем systemd и запускаем сервис..."
systemctl daemon-reexec
systemctl daemon-reload
systemctl enable gion.service
systemctl restart gion.service

echo "✅ Установка завершена. Сервис 'gion.service' активен под пользователем $REAL_USER."
