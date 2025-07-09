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

REAL_USER=$(logname)
REAL_HOME=$(eval echo "~$REAL_USER")
REAL_PATH="$REAL_HOME/.local/bin:$PATH"

echo "Пользователь: $REAL_USER"
echo "Домашняя директория: $REAL_HOME"

INSTALL_DIR="$REAL_HOME/gion"
VENV_DIR="$INSTALL_DIR/.venv"

# Добавляем директорию в безопасные для Git
echo "🔒 Добавляем репозиторий в безопасные директории Git..."
sudo -u "$REAL_USER" git config --global --add safe.directory "$INSTALL_DIR"

# Обработка существующего репозитория
if [ -d "$INSTALL_DIR" ]; then
    echo "🔄 Обновляем существующий репозиторий..."
    cd "$INSTALL_DIR"

    # Сбрасываем изменения и переключаем на main
    sudo -u "$REAL_USER" git reset --hard
    sudo -u "$REAL_USER" git clean -fd
    sudo -u "$REAL_USER" git checkout main
    sudo -u "$REAL_USER" git pull --rebase

    # Исправляем права доступа
    chown -R "$REAL_USER:$REAL_USER" .
    cd ..
else
    echo "⏬ Клонируем репозиторий..."
    sudo -u "$REAL_USER" git clone https://github.com/OnisOris/gion
    chown -R "$REAL_USER:$REAL_USER" "$INSTALL_DIR"
fi

PYTHON_VER="3.13"
echo "Используется Python версии $PYTHON_VER"

if sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c 'command -v uv &>/dev/null'; then
    echo "✅ uv уже установлен. Установка не требуется."
else
    echo "🔧 uv не найден. Устанавливаю..."
    sudo -u "$REAL_USER" bash -c 'curl -LsSf https://astral.sh/uv/install.sh | sh'
fi

# Удаляем старое окружение если существует
if [ -d "$VENV_DIR" ]; then
    echo "🗑️ Удаляем старое виртуальное окружение..."
    rm -rf "$VENV_DIR"
fi

# Создаем директорию с правильными правами
mkdir -p "$VENV_DIR"
chown -R "$REAL_USER:$REAL_USER" "$VENV_DIR"

echo "🐍 Создаём виртуальное окружение..."
sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c "\"$REAL_HOME/.local/bin/uv\" venv --python $PYTHON_VER --prompt pion \"$VENV_DIR\""

echo "📦 Устанавливаем gion..."
sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c "source \"$VENV_DIR/bin/activate\" && \"$REAL_HOME/.local/bin/uv\" pip install -e \"$INSTALL_DIR\""

echo "⚙️ Создаём systemd unit файл /etc/systemd/system/gion.service..."

cat > /etc/systemd/system/gion.service << EOF
[Unit]
Description=Pion Autostart Service
After=network.target

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

echo "🔄 Перезагружаем systemd и запускаем сервис..."
systemctl daemon-reload
systemctl enable gion.service
systemctl restart gion.service

systemctl enable geobot.service
systemctl restart geobot.service

echo "✅ Установка завершена. Сервис 'gion.service' активен под пользователем $REAL_USER."