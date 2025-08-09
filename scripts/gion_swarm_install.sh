#!/bin/bash
set -e
cd "$(dirname "$0")"

export LC_ALL=C.UTF-8
export LANG=C.UTF-8

if [ "$EUID" -ne 0 ]; then
  echo "Пожалуйста, запускайте скрипт с sudo."
  exit 1
fi

# Функция для установки пакетов
install_package() {
  local package=$1
  if dpkg -s "$package" &>/dev/null; then
    echo "✅ $package уже установлен."
  else
    echo "🔧 Устанавливаем $package..."
    apt update
    apt install -y "$package"
  fi
}

# Установка зависимостей
install_package python3-dev
install_package git

REAL_USER=$(logname)
REAL_HOME=$(eval echo "~$REAL_USER")
REAL_PATH="$REAL_HOME/.local/bin:$PATH"

echo "Пользователь: $REAL_USER"
echo "Домашняя директория: $REAL_HOME"

INSTALL_DIR="$REAL_HOME/gion"
VENV_DIR="$INSTALL_DIR/.venv"
RUN_SCRIPT_PATH="/usr/local/bin/gion-run.sh"


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
sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c "\"$REAL_HOME/.local/bin/uv\" venv --python \"$PYTHON_BIN\" --prompt pion \"$VENV_DIR\""

echo "📦 Устанавливаем зависимости сборки..."
sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c "source \"$VENV_DIR/bin/activate\" && \"$REAL_HOME/.local/bin/uv\" pip install setuptools wheel Cython numpy"

echo "📦 Устанавливаем pionsdk..."
sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c "source \"$VENV_DIR/bin/activate\" && \"$REAL_HOME/.local/bin/uv\" pip install \"git+https://github.com/OnisOris/pion.git@dev\""

echo "📦 Устанавливаем gion..."
sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c "source \"$VENV_DIR/bin/activate\" && \"$REAL_HOME/.local/bin/uv\" pip install -e \"$INSTALL_DIR\""

echo "📝 Создаём скрипт запуска в $RUN_SCRIPT_PATH"

# Создаем скрипт запуска
cat > "$RUN_SCRIPT_PATH" << EOF
#!/bin/bash
set -e

# Переходим в директорию проекта
cd "$INSTALL_DIR"

# Обновляем репозиторий
echo "🔄 Обновляем репозиторий через git pull --rebase..."
git pull --rebase

sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c "source \"$VENV_DIR/bin/activate\" && \"$REAL_HOME/.local/bin/uv\" sync -U"

# Запускаем приложение
echo "🚀 Запускаем приложение..."
"$VENV_DIR/bin/python" -m gion.__main__
EOF

# Даем права на выполнение
chmod +x "$RUN_SCRIPT_PATH"
echo "✅ Скрипт запуска создан: $RUN_SCRIPT_PATH"

echo "⚙️ Создаём systemd unit файл /etc/systemd/system/gion.service..."

# Обновляем сервис для использования скрипта запуска
cat > /etc/systemd/system/gion.service << EOF
[Unit]
Description=Pion Autostart Service
After=network.target

[Service]
Type=simple
ExecStart=$RUN_SCRIPT_PATH
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

# Добавляем geobot.service, если он установлен
if [ -f "/etc/systemd/system/geobot.service" ]; then
  echo "🔄 Активируем geobot.service..."
  systemctl enable geobot.service
  systemctl restart geobot.service
else
  echo "⚠️ geobot.service не найден. Пропускаем активацию."
fi

echo "✅ Установка завершена. Сервис 'gion.service' активен под пользователем $REAL_USER."
echo "✅ Скрипт запуска доступен по пути: $RUN_SCRIPT_PATH"
