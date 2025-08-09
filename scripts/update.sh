#!/usr/bin/env bash
# Обновление всех геоботов в подсети: git pull --rebase + uv sync -U
# Использует sshpass; параллелит задачи.

set -uo pipefail

# ---- Параметры по умолчанию (можно переопределить переменными окружения) ----
USER_NAME="${USER_NAME:-gion}"
PASS="${PASS:-gion}"
NET="${1:-10.1.100}"   # база подсети без последнего октета (аргумент 1)
START="${2:-1}"        # начало диапазона (аргумент 2)
END="${3:-254}"        # конец диапазона (аргумент 3)
JOBS="${JOBS:-16}"     # степень параллелизма
SSH_TIMEOUT="${SSH_TIMEOUT:-5}"  # таймаут SSH-соединения (сек)

# ---- Проверки ----
if ! command -v sshpass >/dev/null 2>&1; then
  echo "Устанавливаю sshpass (нужен для парольного входа)..." >&2
  if command -v apt >/dev/null 2>&1; then
    sudo apt update && sudo apt install -y sshpass
  else
    echo "Не найден apt. Установите sshpass вручную и повторите." >&2
    exit 1
  fi
fi

# ---- Функция работы с одним хостом ----
do_host() {
  local ip="$1"

  # Пробуем подключиться и выполнить обновление
  sshpass -p "$PASS" \
  ssh -o ConnectTimeout="$SSH_TIMEOUT" \
      -o StrictHostKeyChecking=no \
      -o UserKnownHostsFile=/dev/null \
      -o PreferredAuthentications=password \
      -o NumberOfPasswordPrompts=1 \
      -l "$USER_NAME" "$ip" \
      'set -e
       export PATH="$HOME/.local/bin:$PATH"
       if [ ! -d "$HOME/gion" ]; then
         echo "[$HOSTNAME] ~/gion не найден — пропуск."
         exit 0
       fi
       cd "$HOME/gion"
       echo "[`hostname` @ '"$ip"'] git pull --rebase..."
       git pull --rebase || exit $?
       echo "[`hostname` @ '"$ip"'] uv sync -U..."
       uv sync -U
       echo "[`hostname` @ '"$ip"'] ✅ Готово."
      ' \
  >/tmp/geobot_update_"$ip".log 2>&1

  rc=$?
  if [ $rc -eq 0 ]; then
    echo "[$ip] ✅ Обновлено. Лог: /tmp/geobot_update_$ip.log"
  else
    # тихо пропускаем не наши/недоступные устройства
    echo "[$ip] ❌ Пропуск (код $rc). Лог: /tmp/geobot_update_$ip.log"
  fi
}

export -f do_host
export USER_NAME PASS SSH_TIMEOUT

# ---- Генерация списка IP и параллельный запуск ----
ips=()
for i in $(seq "$START" "$END"); do
  ips+=("$NET.$i")
done

printf "%s\n" "${ips[@]}" | xargs -n1 -P "$JOBS" bash -c 'do_host "$@"' _

sudo systemctl restart geobot
sudo systemctl restart gion

echo "✅ Завершено. Смотрите логи в /tmp/geobot_update_*.log"
