# Инструкция для жюри: сборка, запуск, проверка

Пакет `tram_backup_odometry` (ROS 2 Humble, Python/rclpy) в реальном времени оценивает скорость
и положение трамвая по скоростям двух тележек и позиции контроллера водителя. Выходы:
`/result/velocity` и `/result/position`.

Подробности: алгоритм — [MODEL.md](MODEL.md), параметры — [PARAMETERS.md](PARAMETERS.md),
результаты — [RESULTS.md](RESULTS.md), ограничения — [LIMITATIONS_ROADMAP.md](LIMITATIONS_ROADMAP.md),
офлайн-инструменты — [tools/README.md](../tools/README.md), калибровка —
[tools/calibration/README.md](../tools/calibration/README.md).

Команды выполняются из корня решения. Датасет и чекер организаторов лежат рядом с ним:

```
<parent>/
├── dataset/data/<bag>/{metadata.yaml, <bag>_0.db3}   # распакованный data.zip организаторов
├── check_code/check-code/                           # чекер организаторов
│   ├── src/checker_ros/                             #   пакет hackathon_solution_checker (metrics.py)
│   └── bags/30618_88aea4d9/                         #   тестовый bag с эталоном /localization/kinematic_state
└── solution/                                        # этот репозиторий
```

## Соглашения по умолчанию

- **Точка выхода** — `base_link`: на пути (pathgraph или ветка разворотной петли) в 9.873 м
  впереди master-антенны (`output_along_offset_m: 9.873`); z = z pathgraph
  (`output_z_offset_m: 0.0`); система `map` (`output_frame: map`).
- **Метки времени.** `header.stamp` выхода равен header stamp входа, вызвавшего публикацию (5.2).
  Положение считается на сам stamp, `/result/velocity` — оценка скорости на момент
  stamp − 0.09 с (`output_velocity_delay_s: 0.09`): задержка подобрана под эталон чекера, у
  которого `twist.linear.x` отстаёт от своего stamp; `0.0` — скорость на сам stamp.
- **GNSS.** Скорость считается только по тележкам, контроллеру и карте. GNSS используется для
  выставки на старте и для коррекции положения вдоль пути (`gnss_aiding: true`); на скорость не
  влияет. Выключить коррекцию: `-p gnss_aiding:=false` или свой `params_file` (4.1). Подробно —
  [MODEL.md](MODEL.md) §9.
- **Вагон.** Launch-файл по умолчанию передаёт `vehicle_id:=30618` — вагон скрытой проверки. Для
  bag другого вагона передавайте префикс имени bag (`30639_…` → `vehicle_id:=30639`);
  `run_bag.sh` и `run_checker.sh` делают это сами. В `params.yaml` значение пустое, поэтому при
  `ros2 run` передавайте `-p vehicle_id:=…`.

Все соглашения переключаются параметрами без пересборки (4.1).

---

## 0. Коротко: основные команды

```bash
# 1) образ ROS 2 Humble с собранным workspace + проверка офлайн-сборки (colcon build в --network none)
./scripts/build.sh
# 2) прогон одного bag через ноду в Docker; сверка выходов с входным bag (метки, frame_id, частота)
./scripts/run_bag.sh ../dataset/data/30618_0e41eac3 /tmp/run_0e41eac3 --duration 120
cat /tmp/run_0e41eac3/check.txt
# 3) замер задержки, CPU и RSS при воспроизведении в реальном времени (первые 120 с)
./scripts/measure.sh ../dataset/data/30618_0e41eac3 /tmp/measure_0e41eac3
# 4) чекер организаторов (hackathon_solution_checker) на их тестовом bag, в Docker, в реальном времени (~22 мин)
./scripts/run_checker.sh ../check_code/check-code/bags/30618_88aea4d9 /tmp/chk_88aea4d9 --measure
cat /tmp/chk_88aea4d9/checker_final.txt
# 5) то же офлайн за секунды: эмуляция чекера (хостовый Python, ROS не нужен)
OMP_NUM_THREADS=1 PYTHONPATH=src/tram_backup_odometry python3 tools/checker_sim.py --variants final
# 6) офлайн-оценка точности по всем bag (хостовый Python с numpy/scipy/matplotlib/pyyaml)
python3 tools/build_cache.py && python3 tools/bag_index.py
python3 tools/run_cv.py --tag check --bags all --antenna all   # -> results/check/summary.md
```

Ручной запуск (нода + `ros2 bag play`) — раздел 4.

---

## 1. Требования

| Сценарий | Что нужно |
|---|---|
| Docker (рекомендуется) | Docker 20+ (Linux или Docker Desktop), bash, ~2 ГБ на образ `ros:humble`. Интернет нужен только для `docker build`; сборка workspace и прогоны идут без сети |
| Нативно | Ubuntu 22.04 + ROS 2 Humble, apt-пакеты из раздела 3 (в т. ч. sqlite3-плагин `ros-humble-rosbag2-storage-default-plugins`) |
| Офлайн-оценка (`tools/`) | Python 3.8+ с `numpy`, `scipy`, `matplotlib`, `PyYAML`. ROS не нужен |

---

## 2. Сборка в Docker: `scripts/build.sh`

```bash
./scripts/build.sh                       # params.yaml -> docker build -> офлайн colcon build
RUN_TESTS=1 ./scripts/build.sh           # ... + pytest пакета внутри образа (--network none)
IMAGE=myname:tag ./scripts/build.sh      # другое имя образа (по умолчанию tram_backup_odometry:humble)
```

Переменные окружения: `IMAGE`, `GEN_PARAMS` (1 по умолчанию, 0 = пропустить),
`OFFLINE_CHECK` (1 по умолчанию, 0 = пропустить), `RUN_TESTS` (0 по умолчанию, 1 = запустить).

| шаг | ожидаемый результат |
|---|---|
| генерация `config/params.yaml` (`GEN_PARAMS=1`) | файл обновлён из dataclass `Params` |
| `docker build` по `docker/Dockerfile` | образ `tram_backup_odometry:humble`; `CMD` — `ros2 launch tram_backup_odometry backup_odometry.launch.py` |
| офлайн-проверка (`OFFLINE_CHECK=1`): чистый `colcon build` в `docker run --network none` | `network: unreachable as intended (...)`, `2 packages finished`, `import OK` |
| тесты (`RUN_TESTS=1`): pytest пакета в образе без сети | `120 passed, 11 skipped` |

Без скрипта (из корня решения): `docker build -f docker/Dockerfile -t tram_backup_odometry:humble .`;
контекст ограничивает `docker/Dockerfile.dockerignore`.

Перенос образа без интернета: `docker save tram_backup_odometry:humble | gzip > tram.tar.gz`,
затем `docker load < tram.tar.gz`.

**`package.xml` пакета `tram_vehicle_msgs`.** В этой копии пакета из датасета нет тега
`<maintainer>`, и `catkin_pkg` в Humble такой манифест не принимает. Dockerfile и скрипты
добавляют заглушку только в копию для сборки; для нативной сборки команда — в разделе 3.

---

## 3. Нативная сборка в ROS 2 Humble

```bash
# зависимости (один раз, нужен apt-репозиторий ROS 2)
sudo apt install ros-humble-ros-base python3-colcon-common-extensions \
                 python3-numpy python3-yaml ros-humble-rosbag2-storage-default-plugins python3-pytest

mkdir -p ~/tram_ws/src
cp -r <solution>/src/tram_vehicle_msgs <solution>/src/tram_backup_odometry ~/tram_ws/src/
cd ~/tram_ws
# заглушка <maintainer> в копии tram_vehicle_msgs (без неё catkin_pkg не принимает манифест)
for f in src/*/package.xml; do grep -q '<maintainer' "$f" || \
    sed -i 's|</license>|</license>\n  <maintainer email="noreply@example.com">unknown</maintainer>|' "$f"; done
source /opt/ros/humble/setup.bash
colcon build                       # 2 packages finished; интернет не нужен
source install/setup.bash
python3 -m pytest -q src/tram_backup_odometry/test     # необязательно
```

Нода импортирует `tram_vehicle_msgs.msg`: оба пакета должны быть в одном workspace или в underlay.

---

## 4. Запуск ноды и воспроизведение bag

### 4.1 Launch-файл

```bash
ros2 launch tram_backup_odometry backup_odometry.launch.py vehicle_id:=30618
# в другом терминале (с тем же source install/setup.bash):
ros2 bag play <dataset>/data/30618_0e41eac3
```

| аргумент | по умолчанию | назначение |
|---|---|---|
| `vehicle_id` | `30618` | масштаб колёс по номеру вагона, = префикс имени bag (`30618_0e41eac3` → `30618`); `''` = из `params_file`; значения — [PARAMETERS.md](PARAMETERS.md) §2 |
| `output_frame` | `''` (из yaml: `map`) | `map` / `utm` / `start` / `enu`, см. 5.3 |
| `use_sim_time` | `false` | `true` требует `ros2 bag play --clock` |
| `params_file` | `share/tram_backup_odometry/config/params.yaml` | файл параметров (`backup_odometry: ros__parameters:`) |
| `maps_dir` | `''` (установленные `share/.../maps`) | каталог с `t2s.json`, `s2t.json`, `loops.json` |
| `log_level` | `info` | |

Пустые `output_frame`, `vehicle_id` и `maps_dir` не перекрывают `params_file`.

**Остальные параметры** ([PARAMETERS.md](PARAMETERS.md)) задаются только через `-p` или свой
`params_file`: например, `gnss_aiding:=false` как аргумент `ros2 launch` молча игнорируется.

```bash
ros2 run tram_backup_odometry backup_odometry_node --ros-args \
    --params-file "$(ros2 pkg prefix tram_backup_odometry)/share/tram_backup_odometry/config/params.yaml" \
    -p vehicle_id:=30618 -p gnss_aiding:=false -p output_velocity_delay_s:=0.0
```

Свой `params_file` может содержать только изменяемые ключи:
`backup_odometry: {ros__parameters: {gnss_aiding: false}}`.

Параметры соглашений выхода (кроме `vehicle_id` и `output_frame`):

| параметр | по умолчанию | значения |
|---|---|---|
| `output_along_offset_m` | `9.873` | точка выхода вдоль пути от master-антенны: 9.873 = `base_link`, 0 = master-антенна |
| `output_z_offset_m` | `0.0` | смещение z от pathgraph |
| `output_velocity_delay_s` | `0.09` | скорость на момент stamp − задержка; `0.0` = на сам stamp |
| `gnss_aiding` | `true` | `false`: GNSS только для выставки, после неё подписки уничтожаются |
| `enu_origin_auto` | `true` | `false`: начало ENU в `enu_origin_lat`/`_lon`/`_alt` |
| `publish_on` | `all` | `cmd` — строго монотонные метки, около 20 Гц (5.2) |

Остальное — [PARAMETERS.md](PARAMETERS.md) §1.

Недопустимое значение перечислимого параметра (с учётом регистра) останавливает ноду при старте:
`ValueError: output_frame must be one of map|utm|start|enu, got 'ENU'`. Параметры read-only
(`ros2 param set` не работает); ключ в yaml — имя узла `backup_odometry`.

### 4.2 То же в Docker

Один контейнер, внутри нода в фоне и `ros2 bag play`:

```bash
docker run --rm -it --name tram -v <dataset>/data:/data:ro tram_backup_odometry:humble bash
# внутри контейнера:
ros2 launch tram_backup_odometry backup_odometry.launch.py vehicle_id:=30618 &
ros2 bag play /data/30618_0e41eac3
# второй терминал хоста: docker exec -it tram bash  (.bashrc подключает /ws/install)
ros2 topic hz /result/position
ros2 topic echo /backup_odometry/diagnostics --once
```

Нода и `ros2 bag play` в разных контейнерах (Linux): оба запускать с `--network host --ipc host`,
например
`docker run --rm -it --network host --ipc host tram_backup_odometry:humble ros2 launch tram_backup_odometry backup_odometry.launch.py vehicle_id:=30618`.

### 4.3 Что ожидать в логе

```
[backup_odometry]: ready: vehicle_id='30618' k=3.5970 output_frame=map (frame_id=map) publish_on=all maps=/ws/install/tram_backup_odometry/share/tram_backup_odometry/maps (pipeline init 0.01 s); waiting for inputs
[backup_odometry]: first pose: map (103634.04, 86057.61, 165.18) yaw 1.569, mode init_provisional:single_front
...
[backup_odometry] shutdown: inputs front=12216 rear=12225 cmd=26249 fix master=438 rover=435 | outputs velocity=38476 position=38473 | callback mean 0.693 ms p50 0.668 p95 1.585 p99 1.995 max 6.320 ms | errors 0, stamp fallbacks 0, non-finite outputs dropped 0
```

Пример — тестовый bag организаторов целиком, параметры по умолчанию
(`results/checker_ros_30618_88aea4d9_final/node.log`); на других bag и длительностях счётчики
другие.

| что видно | когда |
|---|---|
| `/result/velocity` | с первого принятого сообщения колёс |
| `/result/position`, строка `first pose: … mode init_provisional:…` | после первого пригодного GNSS fix; поза уточняется с каждым fix, окончательная выставка — примерно через 3 с |
| `/result/position` с `frame_id = odom`, `mode no_gnss:…` | за первые 10 с нет пригодного fix: счисление, x = путь от старта, y = z = 0 |
| строка `initial alignment done … GNSS subscriptions destroyed` | после выставки при `gnss_aiding:=false`, а также при переходе на счисление без GNSS |

Состояние выставки видно в диагностике (5.5). При завершении (SIGINT) нода печатает сводку
`shutdown:` (входы, выходы, время callback, ошибки). Выставка подробно — [MODEL.md](MODEL.md) §7.

---

## 5. Интерфейс

### 5.1 Топики и QoS

| направление | топик | тип | QoS |
|---|---|---|---|
| вход | `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor`, **км/ч** | best effort, 50 |
| вход | `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/DriverControllerCommand` (int8, −15..15) | best effort, 50 |
| вход, только положение | `/sensing/gnss/master/fix`, `/sensing/gnss/rover/fix` | `sensor_msgs/NavSatFix` | best effort, 50 |
| выход | `/result/velocity` | `tram_vehicle_msgs/VelocitySensor`, поле `velocity`, **м/с**, `frame_id = base_link` | reliable, volatile, 50 |
| выход | `/result/position` | `nav_msgs/Odometry`, `frame_id` = выходной фрейм, `child_frame_id = base_link` | reliable, volatile, 50 |
| выход | `/backup_odometry/diagnostics` | `diagnostic_msgs/DiagnosticArray`, 1 Гц | reliable, 10 |
| выход | `/backup_odometry/slip` | `std_msgs/Bool`, при изменении | reliable, transient local (latched) |

### 5.2 Метки времени (`header.stamp`)

- **stamp выхода = header stamp входа, вызвавшего публикацию.**
- **Один выход на уникальный stamp входа.** Команда контроллера ~20 Гц, пара тележек ~10 Гц с
  общим stamp → около **30 Гц** на каждый выходной топик (`publish_on: all`).
- Метки не монотонны (выходы идут в порядке прихода входов; в `check.txt` — `stamp decreases`).
  Строго монотонные метки — `publish_on:=cmd` (~20 Гц).

### 5.3 Системы координат (`output_frame`)

| `output_frame` | `header.frame_id` | координаты |
|---|---|---|
| `map` (**по умолчанию**) | `map` | Система карт pathgraph: UTM 37N (WGS84, lon0 = 39°) минус (300000, 6100000) |
| `utm` | `utm_37n` | Полные UTM 37N easting/northing |
| `start` | `odom` | `map`, сдвинутая в положение master-антенны при выставке (z0 — высота GNSS); оси как у `map` |
| `enu` | `enu` | Локальная East-North-Up плоскость WGS84; начало — положение master-антенны при выставке или `enu_origin_*` (4.1) |
| нет GNSS за 10 с | `odom` | Счисление при любом `output_frame`: x = путь от старта, y = z = 0 |

- Начало `start` и `enu` фиксируется при окончательной выставке.
- Разворотные петли у конечных (их нет в pathgraph) берутся из `maps/loops.json`; см.
  [MODEL.md](MODEL.md) §6.3.

### 5.4 Содержимое `nav_msgs/Odometry`

| поле | значение |
|---|---|
| `header.stamp` / `header.frame_id` | stamp входа (5.2) / выходной фрейм (5.3) |
| `child_frame_id` | `base_link` |
| `pose.pose.position` | x, y, z точки выхода (см. «Соглашения по умолчанию») |
| `pose.pose.orientation` | кватернион только рыскания (roll = pitch = 0); yaw — касательная пути по ходу движения |
| `twist.twist.linear.x` | продольная скорость, м/с (≥ 0), то же значение, что в `/result/velocity` |
| `pose.covariance`, `twist.covariance` | заполнены оценками фильтра; формулы — [MODEL.md](MODEL.md) §10 |

### 5.5 Диагностика

`/backup_odometry/diagnostics` — 1 Гц, одна запись `backup_odometry: estimator`:

- уровень: ERROR — исключения в callback; WARN — нет входов (или их нет дольше 1 с), нет позы,
  подозрение на боксование или юз;
- `mode` = `<этап>:<колёса>`: этап `init` / `init_provisional` (выставка), `nav*` (навигация),
  `no_gnss` (счисление); колёса `both`, `disagree`, `single_front`, `single_rear`, `model_only`;
- выставку показывают `has_pose=True` и этап `nav*`. `init_done` означает «GNSS больше не нужен»:
  при `gnss_aiding: true` он и после выставки остаётся `False` — это норма;
- полный список ключей — [MODEL.md](MODEL.md) §11.4–11.5.

`/backup_odometry/slip` (Bool, latched) публикуется при смене флага боксования или юза.

```bash
ros2 topic echo /backup_odometry/slip
ros2 topic echo /backup_odometry/diagnostics --field status   # или rqt_runtime_monitor
```

---

## 6. Проверка на bag и замер реального времени (Docker)

### 6.1 `scripts/run_bag.sh`

```bash
scripts/run_bag.sh <bag_dir> <out_dir> [опции] [-- доп. launch-аргументы, напр. output_frame:=utm]
```

| опция | смысл |
|---|---|
| `--rate R` | скорость `ros2 bag play` (по умолчанию 1.0) |
| `--duration S` | остановить воспроизведение через S секунд (по умолчанию весь bag) |
| `--start S` | начать с S-й секунды bag (выставке нужен GNSS в начале прогона) |
| `--vehicle ID` | `vehicle_id`; по умолчанию из префикса имени bag (`30618_`, `30639_`) |
| `--sim-time` | нода с `use_sim_time:=true`, воспроизведение с `--clock 100` |
| `--dev` / `--src DIR` | взять текущий `src/` (или DIR) и пересобрать пакет в контейнере вместо копии из образа |
| `--measure` | запустить рядом `tools/latency_probe.py` и `tools/resource_monitor.py` |
| `--image NAME` | образ (по умолчанию `tram_backup_odometry:humble`) |

Контейнер запускается с `--network none`; `<out_dir>` монтируется в `/out`, `<out_dir>/result_bag`
не должен существовать заранее.

Результаты в `<out_dir>`:

- `result_bag/` — rosbag2 с `/result/velocity`, `/result/position`, `/backup_odometry/diagnostics`, `/backup_odometry/slip`;
- `node.log`, `play.log`, `record.log`;
- `check.txt` — сверка выходов с входным bag по каждому топику: число сообщений, доля stamp,
  равных stamp входов, частота по stamp, покрытие сетки 0.1 с, `frame_id`/`child_frame_id`,
  дубли и убывания stamp, последний статус диагностики. Ожидается: `stamps==input` 100 %,
  ~30 Гц, покрытие ≈ 100 %, `duplicate stamps 0`; ненулевое `stamp decreases` — норма (5.2).
  Строка `last diagnostics` снята после конца воспроизведения: WARN `no input for X s` и (при
  `gnss_aiding: true`) `init_done=False` — норма (5.5);
- с `--measure` — `latency.{txt,json,csv}` и `resources.{txt,json,csv}` (6.2).

### 6.2 `scripts/measure.sh`

```bash
scripts/measure.sh <bag_dir> [out_dir] [опции run_bag.sh]   # = run_bag.sh --measure --duration 120; весь bag: --duration <длина bag>
```

`out_dir` по умолчанию: `${TMPDIR:-/tmp}/tram_measure_<bag>_<время>`. Кроме файлов `run_bag.sh`
(6.1) пишутся:

- **`latency.txt`** (`tools/latency_probe.py`; также `.json`, `.csv`): задержка = приход выхода
  минус приход входа с тем же header stamp; по каждому топику p50/p95/p99/max в строках `all` и
  `after 2 s` (без стартовой пачки bag).
- **`resources.txt`** (`tools/resource_monitor.py`, psutil, 2 Гц; также `.json`, `.csv`): CPU
  процесса ноды в ядрах (mean/p50/p95/max) и RSS в МиБ (start/mean/max).

Ожидается: в строке `after 2 s` p99 ≈ 2–3 мс, max < 30 мс; строка `all` включает стартовую пачку
bag (на 120-секундном прогоне p99 и max ≈ 40–60 мс, на всём тестовом bag max ≈ 32 мс). Требование
ТЗ: ≤ 100 мс, пик ≤ 250 мс. CPU ≈ 0.05–0.07 ядра, RSS ≈ 62–64 МиБ без роста (лимит ТЗ: 2 ядра,
0.5 ГБ). Сохранённые замеры — `results/rt/` и `results/checker_ros_30618_88aea4d9_final/`, разбор —
[RESULTS.md](RESULTS.md) §7.1–7.2.

---

## 7. Проверка точности: чекер организаторов и офлайн-оценка

7.1 — чекер организаторов на их тестовом bag; 7.2–7.5 — офлайн-оценка по 122 bag датасета против
GNSS-эталона (RTK).

Офлайн-инструменты (`tools/`) вызывают тот же `Pipeline`, что и нода, и запускаются хостовым
Python без ROS, из корня решения:

```bash
export PY=python3            # Python 3.8+ с numpy, scipy, matplotlib, PyYAML
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
```

### 7.1 Проверка чекером организаторов

#### (а) Офлайн-эмуляция: `tools/checker_sim.py`

Прогоняет тестовый bag и эмулирует синхронизатор чекера; несколько секунд.

```bash
PYTHONPATH=src/tram_backup_odometry $PY tools/checker_sim.py --variants final
```

Ожидается: v RMSE ≈ 0.0335 м/с, 3D RMSE ≈ 1.11 м. Другие варианты и опции —
[tools/README.md](../tools/README.md) (`checker_sim.py`), результаты — [RESULTS.md](RESULTS.md) §2.

#### (б) Настоящий чекер в Docker: `scripts/run_checker.sh`

Нужен образ из раздела 2.

```bash
./scripts/run_checker.sh ../check_code/check-code/bags/30618_88aea4d9 /tmp/chk_88aea4d9 --measure
cat /tmp/chk_88aea4d9/checker_final.txt
./scripts/run_checker.sh ../check_code/check-code/bags/30618_88aea4d9 /tmp/chk_120 --duration 120   # короткая проверка
```

Использование: `scripts/run_checker.sh <bag_dir> <out_dir> [опции] [-- аргументы launch]`.
Опции как у `run_bag.sh` (6.1): `--rate R`, `--duration S`, `--vehicle ID`, `--measure`,
`--image NAME`; плюс `--checker DIR` (по умолчанию `../check_code/check-code/src/checker_ros`).

Скрипт в одном контейнере без сети собирает чекер, запускает ноду и чекер, проигрывает bag и
завершает чекер по SIGINT.

Результаты в `<out_dir>`: `checker_final.txt` / `checker_final.json` (итоговые RMSE, max, n; если
итогового отчёта нет — последний 5-секундный, с пометкой), `checker.log`, `checker_build.log`,
`node.log`, `play.log`, `record.log`, `result_bag/` (выходы и эталон), с `--measure` — файлы 6.2.

Ожидается на всём тестовом bag (~22 мин): `velocity RMSE` ≈ 0.0324 м/с, `position distance
RMSE` ≈ 1.111 м. Эталонный прогон — `results/checker_ros_30618_88aea4d9_final/`.

`<out_dir>/result_bag` не должен существовать: для повтора нужен новый каталог. Параметры, которых
нет среди аргументов launch (4.1), передавайте частичным YAML в `<out_dir>`:

```bash
mkdir -p /tmp/chk_noaid
printf 'backup_odometry:\n  ros__parameters:\n    gnss_aiding: false\n' > /tmp/chk_noaid/noaid.yaml
./scripts/run_checker.sh ../check_code/check-code/bags/30618_88aea4d9 /tmp/chk_noaid -- params_file:=/out/noaid.yaml
```

### 7.2 Офлайн-оценка по датасету: кэш и индекс

`build_cache.py` читает bag из `../dataset/data` (другой путь — `--data`); `bag_index.py` берёт
оттуда же `metadata.yaml`, а если их нет — время из кэша. Остальные инструменты работают по кэшу
`.work/cache/<bag>.pkl` и индексу `.work/bag_index.json`.

```bash
$PY tools/build_cache.py                  # все bag -> .work/cache/<bag>.pkl (параллельно; существующие пропускаются)
$PY tools/build_cache.py 0e41eac3 --force # выбранные bag (полное имя или уникальный суффикс), перезаписать
$PY tools/build_cache.py --list           # что есть в кэше
$PY tools/bag_index.py                    # -> .work/bag_index.json
```

Кэш — 122 файла, ~0.5 ГБ. `eval_ok` — уникальный bag с ≥ 60 с RTK-эталона; поля индекса —
[tools/README.md](../tools/README.md). Ожидается: 122 bag, 97 уникальных, 60 `eval_ok`, 6 групп.

### 7.3 Прогон и сводка: `tools/run_cv.py`

```bash
$PY tools/run_cv.py --tag check --bags all --antenna all   # -> results/check/summary.md
```

Эталон — fix master-антенны (с `--antenna all` также rover и середина базы), поэтому `run_cv.py`
ставит точку выхода в master-антенну (0 м, z +3.10) без задержки скорости; GNSS подаётся только
первые 10 с (`--gnss-limit`). Остальные опции — [tools/README.md](../tools/README.md).

Ожидается (медианы по 60 `eval_ok` bag): v_rmse ≈ 0.0295 м/с, al_rmse ≈ 1.98 м,
e3d_mean ≈ 1.66 м. Сохранённые прогоны и разбор — [RESULTS.md](RESULTS.md).

### 7.4 Один bag вручную

```bash
# replay -> est.npz; внутри Docker: ros2 run tram_backup_odometry replay ...
# точка выхода = master-антенна, без задержки скорости: так же, как в tools/run_cv.py
PYTHONPATH=src/tram_backup_odometry $PY -m tram_backup_odometry.replay .work/cache/30618_0e41eac3.pkl \
    --out /tmp/est.npz --set output_along_offset_m=0 output_z_offset_m=3.10 output_velocity_delay_s=0
# оценка против GNSS-эталона
$PY tools/evaluate.py 30618_0e41eac3 --est /tmp/est.npz
# рисунок: скорость, ошибки, along-track, карта с зумами на начало и конец
$PY tools/plots.py --tag mytag --bags 30618_0e41eac3,40ffd323   # -> results/mytag/plots/<bag>.png
```

`replay` подаёт GNSS весь прогон, CV — первые 10 с; число, как в CV:
`$PY tools/run_cv.py --tag one --bags 30618_0e41eac3`. Опции и метрики `evaluate.py` —
[tools/README.md](../tools/README.md).

### 7.5 Робастность

```bash
$PY tools/robustness.py --list                                # сценарии отказов
$PY tools/robustness.py --out /tmp/rb --plots /tmp/rb/plots   # всё: faults, clean, real, plots -> /tmp/rb/summary.md
```

Сохранённый прогон — `results/robustness/summary.md`; без `--out` и `--plots` он и
`docs/plots/robust_*.png` перезаписываются. Сценарии и опции — [tools/README.md](../tools/README.md).
