# Инструкция для жюри: сборка, запуск, проверка

Пакет `tram_backup_odometry` (ROS 2 Humble, Python/rclpy) оценивает скорость и положение
трамвая в реальном времени. Входы: скорости двух тележек и позиция контроллера водителя.
Выходы: `/result/velocity` и `/result/position`. **Скорость** считается только по тележкам,
контроллеру и карте, GNSS в неё не попадает. **Положение** привязывается по GNSS в окне
выставки на старте, а дальше (по умолчанию, `gnss_aiding: true`) уточняется вдоль пути по
редким GNSS-фиксам в середине маршрута. Организаторы это разрешили без штрафа. С
`gnss_aiding:=false` нода, как в первой версии, отписывается от GNSS сразу после выставки.

Что изменилось после уточнений организаторов (подробно в разделах 4.1, 5.3, 7.1 и 9):

| уточнение организаторов | что сделано в пакете |
|---|---|
| эталон `/localization/kinematic_state` — точка `base_link` на pathgraph в 9.873 м впереди master-антенны; z = z pathgraph; ошибка положения 3D | выход по умолчанию в `base_link`: `output_along_offset_m` = 9.873, `output_z_offset_m` = 0.0 (z = z pathgraph) |
| чекер сопоставляет выходы с эталоном через `ApproximateTimeSynchronizer` (slop 0.05 с, очередь 100); скорость = `twist.linear.x`, она отстаёт от своего stamp примерно на 0.09 с | `output_velocity_delay_s` = 0.09: в `/result/velocity` публикуется скорость на момент stamp − 0.09 с. Это подгонка под эталон, а не физика, подробнее в 4.1 |
| GNSS в середине маршрута можно использовать для **положения**. Скорость должна идти только от тележек и контроллера | коррекция положения по GNSS (`gnss_*`); скорость побитово совпадает с `gnss_aiding` и без него (тест `test/test_gnss_aiding.py`) |
| скрытая проверка идёт только на вагоне 30618; RTK в окне старта гарантирован; старт на конечной перед разворотной петлёй | масштаб колёс 30618 (`vehicle_id`, 4.1); геометрия разворотных петель `maps/loops.json` (5.3) |
| сообщения `tram_vehicle_msgs` берутся из датасета | `src/tram_vehicle_msgs` побайтно совпадает с пакетом датасета (раздел 2) |

Алгоритм описан в [MODEL.md](MODEL.md), все параметры в [PARAMETERS.md](PARAMETERS.md),
результаты и абляции в [RESULTS.md](RESULTS.md), ограничения в
[LIMITATIONS_ROADMAP.md](LIMITATIONS_ROADMAP.md), калибровочные скрипты в
[tools/calibration/README.md](../tools/calibration/README.md). Проверка чекером организаторов
описана в разделе 7.1, параметры, которые переключают соглашения выхода, собраны в
разделах 4.1 и 9.
Все команды ниже проверены на исходниках из этого репозитория. Пути даны от корня решения
(каталог с `README.md`, `src/`, `scripts/`, `tools/`). Датасет и чекер организаторов по
умолчанию лежат рядом с решением:

```
<parent>/
├── dataset/data/<bag>/{metadata.yaml, <bag>_0.db3}   # распакованный data.zip организаторов
├── check_code/check-code/                           # чекер организаторов
│   ├── src/checker_ros/                             #   пакет hackathon_solution_checker (metrics.py)
│   └── bags/30618_88aea4d9/                         #   тестовый bag с эталоном /localization/kinematic_state
└── solution/                                        # этот репозиторий
```

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
# 5) то же офлайн за секунды: эмуляция чекера + варианты (хостовый Python, ROS не нужен)
OMP_NUM_THREADS=1 PYTHONPATH=src/tram_backup_odometry python tools/checker_sim.py --gnss-start-only 35 --out /tmp/chk_sim
# 6) офлайн-оценка точности по всем bag (хостовый Python с numpy/scipy/matplotlib/pyyaml)
python tools/build_cache.py && python tools/bag_index.py
python tools/run_cv.py --tag check --bags all --antenna all --jobs 8   # -> results/check/summary.md (--jobs по умолчанию: число ядер − 2)
```

Ручной запуск в духе README организаторов: сначала нода, затем `ros2 bag play` (раздел 4).

---

## 1. Требования

| Сценарий | Что нужно |
|---|---|
| Docker (рекомендуется) | Docker 20+ (Linux, или Docker Desktop на macOS/Windows), bash, ~2 ГБ на образ `ros:humble`. Интернет нужен только при `docker build` (базовый образ и apt-пакеты). Сборка workspace внутри образа и все прогоны идут без сети |
| Нативно | Ubuntu 22.04 + ROS 2 Humble (`ros-humble-ros-base`), `python3-numpy`, `python3-yaml`, `ros-humble-rosbag2-storage-default-plugins` (sqlite3-плагин для `ros2 bag play`), `python3-colcon-common-extensions`; для тестов ещё `python3-pytest` |
| Офлайн-оценка (`tools/`) | Python 3.8+ с `numpy`, `scipy`, `matplotlib`, `PyYAML`. ROS не нужен. Мы использовали Python 3.13 (conda). Для `ml` окружения: `PY=/opt/miniconda3/envs/ml/bin/python` |

Ресурсы ноды: 0.04–0.07 ядра в среднем (0.071, max 0.259 под чекером организаторов на итоговом
образе) и не больше 64.1 МиБ RSS. Роста нет ни за 31 минуту (раздел 6), ни за 22 минуты под
чекером организаторов (7.1). Лимит ТЗ: 2 ядра и 0.5 ГБ.

---

## 2. Сборка в Docker: `scripts/build.sh`

```bash
./scripts/build.sh                       # params.yaml -> docker build -> офлайн colcon build
RUN_TESTS=1 ./scripts/build.sh           # ... + pytest пакета внутри образа (--network none)
IMAGE=myname:tag ./scripts/build.sh      # другое имя образа (по умолчанию tram_backup_odometry:humble)
```

Переменные окружения: `IMAGE`, `GEN_PARAMS` (1 по умолчанию, 0 = пропустить),
`OFFLINE_CHECK` (1 по умолчанию, 0 = пропустить), `RUN_TESTS` (0 по умолчанию, 1 = запустить).

Что делает скрипт, по шагам:

1. **Генерация `config/params.yaml`.** В контейнере `ros:humble` без сети запускается
   `tools/gen_params_yaml.py`. Он пишет `src/tram_backup_odometry/config/params.yaml` из
   dataclass `Params` (`core/config.py`, единый источник имён, типов и значений по умолчанию).
   Затем читает файл обратно и сверяет каждое поле по значению и типу. Проверить актуальность
   без записи: `python3 tools/gen_params_yaml.py --check` (код 1, если файл устарел).
2. **`docker build`** по `docker/Dockerfile`. Контекст передаётся tar-потоком и содержит только
   `src/ tools/ docker/`: каталог `.work` с кэшами весит сотни МБ. Внутри:
   - apt ставит `python3-numpy python3-yaml python3-pytest python3-psutil ros-humble-rosbag2-storage-default-plugins`;
   - `src/` копируется в `/ws/src`, к `package.xml` без `<maintainer>` добавляется заглушка (см. ниже);
   - `colcon build`, затем удаляются `build/` и `log/`; `tools/` копируется в `/ws/tools`;
   - entrypoint подключает `/opt/ros/humble` и `/ws/install`. `CMD` по умолчанию:
     `ros2 launch tram_backup_odometry backup_odometry.launch.py`;
   - `STOPSIGNAL SIGINT`: `docker stop` завершает ноду штатно, и она печатает итоговую сводку.

   Обычный `docker build -f docker/Dockerfile -t tram_backup_odometry:humble .` из корня
   тоже работает: `docker/Dockerfile.dockerignore` ограничивает контекст теми же тремя каталогами.
3. **Офлайн-проверка сборки (`OFFLINE_CHECK=1`).** Это требование ТЗ «сборка стандартным colcon
   build без внешнего интернета». Скрипт запускает `docker run --network none` с `src/`,
   смонтированным только для чтения, и внутри:
   - пытается открыть соединение с `pypi.org:443` и печатает
     `network: unreachable as intended (...)`;
   - копирует исходники в чистый `/tmp/ws/src`, применяет ту же заглушку `<maintainer>` и
     выполняет `colcon build` с нуля (ожидается `2 packages finished`);
   - выводит `ros2 pkg prefix`, список исполняемых файлов (`backup_odometry_node`, `replay`),
     проверку импорта `tram_backup_odometry.node, tram_vehicle_msgs.msg` (`import OK`) и
     содержимое `share/tram_backup_odometry/{maps,config,launch}`.
4. **Тесты (`RUN_TESTS=1`).** Выполняется
   `python3 -m pytest -q -p no:cacheprovider src/tram_backup_odometry/test` в образе без сети.
   Покрыты модули ядра, генерация параметров и ROS-нода: параметры, побитовое копирование
   stamp, подписки на GNSS (по умолчанию остаются после выставки, при `gnss_aiding:=false`
   уничтожаются), содержимое сообщений, сквозной тест через DDS. `test/test_gnss_aiding.py` (5 тестов) проверяет коррекцию положения по
   GNSS и неизменность скорости. Пачки фиксов исправляют дрейф вдоль пути, а поток скорости при
   этом побитово тот же (`test_bursts_correct_along_track_drift_and_never_touch_speed`). Масштаб
   пути оценивается по базе RTK→RTK (`test_scale_is_learned_between_rtk_bursts`). Пачка из окна
   старта не отключает поправки TRN (`test_start_window_burst_keeps_trn_corrections`, регрессия
   исправления `anchor_delta` 27.09). Несогласованная пачка, например в 30 м от пути,
   отбрасывается (`test_inconsistent_burst_rejected`). Хвост кольца и `RunPath.project_near`
   проверяет `test_run_path_loop_tail_and_project_near`. Тесты
   `test_evaluate.py` пропускаются (skip), так как им нужен scipy, а в образе его нет.
   Итог в образе на текущем коде: `120 passed, 11 skipped` (лог сборки
   `.work/build_final3.log`). На хосте без ROS
   `python -m pytest -q src/tram_backup_odometry/test` даёт `106 passed, 12 skipped`:
   дополнительно пропускается модуль `test_ros_node.py`, которому нужен `rclpy`.

Если у жюри нет интернета, образ можно перенести файлом:
`docker save tram_backup_odometry:humble | gzip > tram.tar.gz`, затем `docker load < tram.tar.gz`.

### Про патч `package.xml` пакета `tram_vehicle_msgs`

`src/tram_vehicle_msgs` побайтно совпадает с пакетом из датасета (`dataset/tram_vehicle_msgs`,
`diff -r` пуст). Организаторы подтвердили, что использовать нужно именно его. В его
`package.xml` (format 3) нет тега `<maintainer>`. Актуальный `catkin_pkg`
1.x в Humble отклоняет такой манифест с ошибкой *«Package 'tram_vehicle_msgs' must declare at
least one maintainer»*, и colcon не собирает пакет. Поэтому Dockerfile, офлайн-проверка
`build.sh` и режим `run_bag.sh --dev` выполняют одну и ту же идемпотентную правку, **только в
копии для сборки** и только если тега нет:

```bash
for f in src/*/package.xml; do
    grep -q '<maintainer' "$f" || \
    sed -i 's|</license>|</license>\n  <maintainer email="noreply@example.com">unknown</maintainer>|' "$f"
done
```

Исходный файл в репозитории не меняется. На сообщения, ROS-типы и wire-формат правка не влияет.

---

## 3. Нативная сборка в ROS 2 Humble

```bash
# зависимости (один раз, нужен apt-репозиторий ROS 2)
sudo apt install ros-humble-ros-base python3-colcon-common-extensions \
                 python3-numpy python3-yaml ros-humble-rosbag2-storage-default-plugins python3-pytest

mkdir -p ~/tram_ws/src
cp -r <solution>/src/tram_vehicle_msgs <solution>/src/tram_backup_odometry ~/tram_ws/src/
cd ~/tram_ws
# заглушка <maintainer> в копии tram_vehicle_msgs (см. выше; без неё catkin_pkg 1.x отказывает)
for f in src/*/package.xml; do grep -q '<maintainer' "$f" || \
    sed -i 's|</license>|</license>\n  <maintainer email="noreply@example.com">unknown</maintainer>|' "$f"; done
source /opt/ros/humble/setup.bash
colcon build                       # 2 packages finished; интернет не нужен
source install/setup.bash
python3 -m pytest -q src/tram_backup_odometry/test     # необязательно
```

- Если в workspace жюри уже есть собранный `tram_vehicle_msgs`, достаточно скопировать только
  `tram_backup_odometry`.
- Оба пакета нужны в одном workspace или в подключённом underlay: нода импортирует
  `tram_vehicle_msgs.msg`.
- `tram_backup_odometry` имеет тип `ament_python`. В `share/` ставятся карты `maps/{t2s,s2t}.json`
  (это файлы pathgraph организаторов: `таллинская - щукинская.json` = T2S и
  `щукинская - таллинская.json` = S2T, побайтно), `maps/loops.json`, `config/params.yaml` и
  `launch/`. `loops.json` содержит геометрию разворотных петель у конечных. Её офлайн строит
  `tools/build_loops.py` по GNSS обучающих bag (5.3), нода только читает файл.
- В `install/` попадает уже сгенерированный `config/params.yaml`. После правки значений по
  умолчанию в `core/config.py` его нужно перегенерировать в каталоге решения
  (`python3 tools/gen_params_yaml.py`) и пересобрать пакет.

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
| `vehicle_id` | `30618` (`vehicle_id:=''` = значение из `params_file`, там `''`) | **Важно.** `30618` или `30639` выбирает откалиброванный масштаб колёс K (v[м/с] = raw[км/ч] / K): 3.597 (`wheel_k_30618`) и 3.59 (`wheel_k_30639`). Пусто или другой номер означает компромисс `wheel_k` = 3.595. Берётся из префикса имени bag: `30618_0e41eac3` → `30618`. Скрытая проверка идёт на вагоне 30618, это умолчание launch-файла; для 30639 задавайте `vehicle_id:=30639`. `run_bag.sh` и `run_checker.sh` подставляют его из имени bag сами, у `tools/checker_sim.py` по умолчанию стоит `--vehicle-id 30618` |
| `output_frame` | `''` (из yaml: `map`) | `map` / `utm` / `start` / `enu`, см. 5.3. Для `enu` начало задаётся параметрами `enu_origin_*` (только через yaml или `-p`, см. ниже) |
| `use_sim_time` | `false` | Результат от него не зависит: оценка идёт по header stamp входов. С `true` нужен `ros2 bag play --clock` |
| `params_file` | `share/tram_backup_odometry/config/params.yaml` | Полный файл параметров (`backup_odometry: ros__parameters:`) |
| `maps_dir` | `''` (установленные `share/.../maps`) | Каталог с `t2s.json`, `s2t.json` и `loops.json` |
| `log_level` | `info` | |

Пустые `output_frame`, `vehicle_id` и `maps_dir` не перекрывают `params_file`, поэтому
отредактированный yaml не затеняется значениями launch по умолчанию.

Нода объявляет 199 параметров dataclass `Params` (полный справочник в
[PARAMETERS.md](PARAMETERS.md); `python3 tools/gen_params_yaml.py --check` печатает
`... is up to date (199 parameters)`) и отдельный параметр `maps_dir`. Всё, что не вынесено в
аргументы launch, задаётся своим yaml (`params_file:=/path/p.yaml`, скопированным из
установленного и отредактированным) или через `ros2 run` с `-p`:

```bash
ros2 run tram_backup_odometry backup_odometry_node --ros-args \
    --params-file "$(ros2 pkg prefix tram_backup_odometry)/share/tram_backup_odometry/config/params.yaml" \
    -p vehicle_id:=30618 -p gnss_aiding:=false -p output_velocity_delay_s:=0.0
```

Параметры, которые задают соглашения выхода. Значения по умолчанию выбраны под чекер
организаторов (раздел 7.1), а в последнем столбце сказано, когда их менять:

| параметр | по умолчанию | смысл и когда менять |
|---|---|---|
| `vehicle_id` | `''` в `params.yaml` (launch по умолчанию подставляет `30618`) | задавать префикс имени bag (`30618` / `30639`), для скрытой проверки `30618`. Без него K = 3.595 и along-track хуже (раздел 8) |
| `output_frame` | `map` | эталон чекера (`/localization/kinematic_state`) задан во фрейме `map`. Для полных UTM → `utm`, для локальной геодезической ENU → `enu`, для системы относительно точки старта в осях карты → `start` |
| `output_along_offset_m` | `9.873` | точка выхода вдоль оси вагона, отсчёт от master-антенны: 9.873 = `base_link` (эталон чекера), 0 = master-антенна, 6.22 = середина базы, 12.44 = rover |
| `output_z_offset_m` | `0.0` | z выхода = z pathgraph + `output_z_offset_m`. 0 = z pathgraph, как у эталона чекера; 3.10 = высота master-антенны (как `altitude` GNSS). Параметр `z_offset_m` = 3.10 остался внутренним: это измеренная высота антенны над z карты, по нему при выставке высота GNSS переводится в z старта и моста |
| `output_velocity_delay_s` | `0.09` | в `/result/velocity` и `twist.linear.x` публикуется оценка скорости на момент stamp − 0.09 с, взятая интерполяцией по истории фильтра. Это **подгонка под эталон**: скорость в `/localization/kinematic_state` тестового bag отстаёт от своего stamp примерно на 0.09 с, и с задержкой v_rmse чекера падает с 0.0566 до 0.0335 м/с (`results/checker_30618_88aea4d9/summary.md`). Калибровка сделана на единственном тестовом bag. `0.0` = физическая скорость на момент stamp. На положение параметр не влияет |
| `gnss_aiding` | `true` | `true`: после выставки подписки на GNSS остаются, пачки фиксов уточняют положение вдоль пути (5.3). `false`: GNSS только в окне выставки, после неё подписки уничтожаются. Скорость одинакова при обоих значениях |
| `loops_enable` | `true` | геометрия разворотных петель из `maps/loops.json`. `false` возвращает мост и хвост первой версии (5.3) |
| `enu_origin_auto` | `true` | `true`: начало ENU = положение master-антенны при выставке (x, y и высота GNSS). `false`: начало в `enu_origin_lat`, `enu_origin_lon` (градусы WGS84), `enu_origin_alt` (м, в той же системе высот, что `altitude` в NavSatFix). Если начало известно, лучше задать его явно |
| `publish_on` | `all` | `cmd`, если нужны строго монотонные метки (около 20 Гц, раздел 5.2) |

Launch-файл передаёт ноде только объявленные аргументы (таблица выше). Если написать
`gnss_aiding:=false` как launch-аргумент, ошибки не будет, но нода его не получит. Параметры
из таблицы задаются через `-p`, через свой `params_file` или через yaml в каталоге результатов
`run_checker.sh` (7.1).

Пример для ENU с заданным началом:

```bash
ros2 run tram_backup_odometry backup_odometry_node --ros-args \
    --params-file "$(ros2 pkg prefix tram_backup_odometry)/share/tram_backup_odometry/config/params.yaml" \
    -p vehicle_id:=30618 -p output_frame:=enu -p enu_origin_auto:=false \
    -p enu_origin_lat:=55.80 -p enu_origin_lon:=37.45 -p enu_origin_alt:=170.0
```

Нода объявляет все параметры со значениями из `core/config.py`, поэтому `params_file` может
содержать только изменяемые ключи, например
`backup_odometry: {ros__parameters: {gnss_aiding: false}}`.

Значения перечислимых параметров проверяются при старте: `publish_on` ∈ {`all`, `cmd`,
`wheels`}, `tail_mode` ∈ {`decay`, `straight`}, `output_frame` ∈ {`map`, `utm`, `start`,
`enu`}, с учётом регистра. Опечатка останавливает ноду до подписки на топики, например
`ValueError: output_frame must be one of map|utm|start|enu, got 'ENU'`. Так ошибка
конфигурации не превращается в тихую деградацию.

Все параметры объявлены read-only: изменить их на лету через `ros2 param set` нельзя. Ключ в
yaml совпадает с именем узла `backup_odometry`, поэтому при запуске в namespace файл не
подхватится.

### 4.2 То же в Docker

Самый простой вариант: один контейнер, внутри нода в фоне и `ros2 bag play`.

```bash
docker run --rm -it --name tram -v <dataset>/data:/data:ro tram_backup_odometry:humble bash
# внутри контейнера:
ros2 launch tram_backup_odometry backup_odometry.launch.py vehicle_id:=30618 &
ros2 bag play /data/30618_0e41eac3
# второй терминал хоста: docker exec -it tram bash  (.bashrc подключает /ws/install)
ros2 topic hz /result/position
ros2 topic echo /backup_odometry/diagnostics --once
```

Нода и `ros2 bag play` в разных контейнерах (Linux): оба запускать с `--network host`, например
`docker run --rm -it --network host tram_backup_odometry:humble ros2 launch tram_backup_odometry backup_odometry.launch.py vehicle_id:=30618`.

### 4.3 Что ожидать в логе

```
[backup_odometry]: ready: vehicle_id='30618' k=3.5970 output_frame=map (frame_id=map) publish_on=all maps=/ws/install/tram_backup_odometry/share/tram_backup_odometry/maps (pipeline init 0.01 s); waiting for inputs
[backup_odometry]: first pose: map (103634.04, 86057.61, 165.18) yaw 1.569, mode init_provisional:single_front
...
[backup_odometry] shutdown: inputs front=12216 rear=12225 cmd=26249 fix master=438 rover=435 | outputs velocity=38476 position=38473 | callback mean 0.693 ms p50 0.668 p95 1.585 p99 1.995 max 6.320 ms | errors 0, stamp fallbacks 0, non-finite outputs dropped 0
```

Это `results/checker_ros_30618_88aea4d9_final/node.log`: тестовый bag организаторов целиком,
значения по умолчанию. Коррекция по GNSS включена, поэтому GNSS-фиксы принимаются до конца
прогона (fix master 438), а строки об уничтожении подписок нет. С `gnss_aiding: false` после
выставки появляется строка вида (`results/rt/0e41eac3_120s_final/node.log`, версия F3):

```
[backup_odometry]: initial alignment done (30 master / 35 rover fixes); GNSS subscriptions destroyed, running on wheels + controller only
```

Порядок событий:

- `/result/velocity` публикуется с первого принятого сообщения колёс.
- `/result/position` появляется сразу после первого пригодного GNSS fix: предварительная
  выставка (режим `init_provisional`) уточняется с каждым новым fix. В офлайн-реплее
  (`results/F4_final/per_bag.json`, поле `first_pose_s`) первая поза выходит так:
  - на 86 bag с GNSS через 0.1 с после первого выхода скорости (медиана; максимум 0.21 с);
  - на 35 из 36 bag без master-fix через 10 с, уже в режиме счисления;
  - на 30639_e4379d7f (есть только rover-fix) через 0.1 с.
- Окончательная выставка происходит, когда по header-времени прошло `init_window_s` = 3 с от
  первого пригодного fix и накоплено не меньше `init_min_fixes` = 5 fix. До этого момента
  привязка (и начало систем `start` и `enu` при `enu_origin_auto: true`) пересчитывается после
  каждого пригодного fix, при окончательной выставке — последний раз; поза в эти моменты может
  сместиться скачком. Если старт внутри разворотной петли, выставка идёт на ветку петли
  (`maps/loops.json`, тип выставки `loop`), иначе на карту или G1-мост (5.3).
- **После выставки при `gnss_aiding: true` (по умолчанию)** подписки на GNSS остаются, и
  строки `GNSS subscriptions destroyed` в логе нет. Следующие fix собираются в пачки и
  уточняют положение вдоль пути (5.3), в диагностике тогда `mode = nav_gnss:...`,
  `gnss_subscribed=True`, растёт `pipe_gnss_bursts`. **При `gnss_aiding:=false`** подписки
  уничтожаются сразу после выставки, и в логе появляется строка
  `initial alignment done (N master / M rover fixes); GNSS subscriptions destroyed, running on wheels + controller only`.
- Отсчёт `init_timeout_s` = 10 с идёт по header-времени от первого принятого сообщения колёс.
  Если к этому моменту нет ни одного пригодного fix, включается счисление от точки старта (5.3),
  коррекция по GNSS выключается, и подписки уничтожаются (поздние fix уже не используются).
  Если fix есть, но их меньше 5, выставка завершается по имеющимся.

---

## 5. Интерфейс

### 5.1 Топики и QoS

| направление | топик | тип | QoS |
|---|---|---|---|
| вход | `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor`, **км/ч** (так в данных, вопреки README датасета) | best effort, KEEP_LAST 50 (совместимо и с reliable-издателем `ros2 bag play`) |
| вход | `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/DriverControllerCommand` (int8, −15..15) | best effort, 50 |
| вход, только положение | `/sensing/gnss/master/fix`, `/sensing/gnss/rover/fix` | `sensor_msgs/NavSatFix` | best effort, 50. Используются для выставки и затем (`gnss_aiding: true`) для коррекции положения вдоль пути. В скорость не попадают. При `gnss_aiding:=false` или счислении без выставки подписки уничтожаются |
| выход | `/result/velocity` | `tram_vehicle_msgs/VelocitySensor`, **м/с**, `frame_id = base_link` | reliable, volatile, 50 (совместимо с best-effort подписчиком судьи) |
| выход | `/result/position` | `nav_msgs/Odometry`, `frame_id` = выходной фрейм, `child_frame_id = base_link` | reliable, volatile, 50 |
| выход | `/backup_odometry/diagnostics` | `diagnostic_msgs/DiagnosticArray`, 1 Гц (таймер на steady clock) | reliable, 10 |
| выход | `/backup_odometry/slip` | `std_msgs/Bool`, при изменении | reliable, transient local (latched) |

`/sensing/gnss/*/vel` нода не использует. Скорость GNSS служит только эталоном в `tools/evaluate.py`.
`/localization/kinematic_state` (эталон чекера в тестовом bag) нода тоже не читает.

### 5.2 Метки времени (`header.stamp`)

- **stamp выхода равен header stamp входного сообщения, которое вызвало публикацию.**
  Копируется исходный объект `builtin_interfaces/Time`, поэтому совпадение побитовое. Время
  записи bag и часы ноды не используются. Оценка ведётся во времени header stamp:
  состояние фильтра экстраполируется ровно к этой метке.
- **Один выход на каждый уникальный stamp входа.** Команда контроллера идёт с частотой около
  20 Гц. Передняя и задняя тележки приходят парой примерно на 10 Гц и имеют общий stamp,
  поэтому пара даёт один выход. Итого около **30 Гц** на каждый выходной топик
  (`publish_on: all`). Каждый stamp публикуется один раз; память дедупликации
  `STAMP_MEMORY` = 64 рассчитана на стартовую пачку: в ней сообщение второй тележки с тем же
  stamp приходит после 25–51 других выходов (6.2). Повторных stamp 0 и на образе F3
  (`results/rt/0e41eac3_120s_final`, `results/rt/27e994fc_full_final`), и в прогоне текущей
  версии под чекером (`results/checker_ros_30618_88aea4d9_final/latency.txt`). В старом замере
  `results/rt/0e41eac3_120s` с памятью 16 их было 25.
- **Метки выходов не монотонны.** Выходы идут в порядке прихода входов, а header stamp колёс
  отстаёт от времени их прихода на 30–65 мс, тогда как у команды всего на 1–5 мс. Поэтому
  выход по колёсам часто получает метку раньше, чем предыдущий выход по команде: в `check.txt`
  прогона `0e41eac3_120s_final` это 1177 из 3645 выходов (`stamp decreases`). Если судья
  сопоставляет по ближайшей метке, это не мешает.
  Если нужны строго монотонные метки, задайте `publish_on:=cmd`: 20 Гц, в офлайн-реплее всего
  bag 30618_0e41eac3 0 убываний, метрики на уровне `all` (v_rmse 0.028 против 0.027 м/с,
  al_rmse 2.94 против 2.97 м; замер на версии F3, до петель и коррекции по GNSS). Вариант
  `publish_on:=wheels` даёт около 9.3 Гц, это ниже требования ТЗ, и тоже не строго монотонен
  (11 убываний на том же bag). Чекеру организаторов немонотонность не мешает:
  `ApproximateTimeSynchronizer` держит очередь из 100 сообщений на топик и подбирает пару по
  близости stamp. В офлайн-эмуляции чекера на тестовом bag в пары попали 38 439 из 38 476
  выходов (`results/checker_30618_88aea4d9/summary.json`, `n_vel_pairs`, `n_out`).
- **Скорость с задержкой.** Stamp выхода остаётся stamp входа, но значение скорости в
  `/result/velocity` и `twist.linear.x` — это оценка на момент stamp − `output_velocity_delay_s`
  (0.09 с). Её дают линейная интерполяция по истории фильтра и экстраполяция, если момент
  новее истории. Так скорость согласуется с эталоном чекера, который отстаёт от своего stamp
  примерно на 0.09 с (4.1). Положение публикуется на сам stamp.
- 100 % выходных stamp совпадают со stamp входов. Покрытие сетки 0.1 с с допуском ±0.05 с
  составляет 100 % (`check.txt`, `latency.txt`).

### 5.3 Системы координат (`output_frame`)

| `output_frame` | `header.frame_id` | координаты |
|---|---|---|
| `map` (**по умолчанию**) | `map` | Система карт pathgraph: UTM 37N (WGS84, lon0 = 39°) минус (300000, 6100000). Проверено по RTK-фиксам: сдвиг около 2 см, поворот и масштаб отсутствуют. В этой же системе задан эталон чекера `/localization/kinematic_state` |
| `utm` | `utm_37n` | Полные UTM 37N easting/northing |
| `start` | `odom` | Система `map`, сдвинутая в начальную точку (x0, y0, z0) = положение master-антенны при выставке (z0 — высота GNSS). Оси как у `map` (сетка UTM). Выход идёт в `base_link`, поэтому уже на старте x, y выхода отстоят от нуля примерно на 9.873 м вдоль пути |
| `enu` | `enu` | Локальная East-North-Up касательная плоскость WGS84. Пересчёт точный: обратная проекция UTM 37N (ряд Крюгера) → широта/долгота → ECEF → ENU; z карты + `output_z_offset_m` трактуется как высота над эллипсоидом, выход z = компонента Up. Начало: положение master-антенны при выставке (`enu_origin_auto: true`) или заданные `enu_origin_lat/lon/alt` (`enu_origin_auto: false`). От `start` отличается поворотом на сближение меридианов UTM (около 1.25–1.27° на маршруте), масштабом UTM и кривизной Земли |
| нет GNSS за 10 с | `odom` | Счисление при любом `output_frame`: x = пройденный путь от старта, y = z = 0 |

- **Точка, положение которой публикуется.** По умолчанию это `base_link`
  (`output_along_offset_m: 9.873`). По уточнению организаторов эталон
  `/localization/kinematic_state` — это `base_link`, он лежит на pathgraph в 9.873 м впереди
  master-антенны по пути. Смещение откладывается по дуге пути (карта или ветка петли) и
  применяется до перевода в выходную систему. Rover стоит на 12.44 м впереди master, середина
  базы на 6.22 м. На тестовом bag выход в точке master-антенны (`output_along_offset_m: 0`,
  z + 3.10) даёт 3D RMSE 10.26 м против 1.11 м у `base_link`
  (`results/checker_30618_88aea4d9/summary.md`, вариант `master_antenna_point`).
- **z** = высота карты pathgraph + `output_z_offset_m` (0.0), то есть z pathgraph, как у
  эталона чекера. Ошибка чекера трёхмерная, и z RMSE на тестовом bag равен 0.04 м против
  3.07 м при z антенны (там же). Прежнее соглашение первой версии (z антенны = z карты +
  3.10 м, сопоставимо с `altitude` NavSatFix) включается через `output_z_offset_m:=3.10`.
  Разность высоты GNSS-антенны и z карты измерена как 3.10 ± 0.008 м, это значение
  `z_offset_m`, и нода использует его внутри при выставке. В `start` z отсчитывается от
  высоты master-антенны при выставке (высота GNSS), поэтому при `output_z_offset_m: 0` на
  старте z ≈ −3.1 м. В `enu` это компонента Up относительно начала ENU.
- **Начало `start` и автоматического `enu`** берётся из текущей выставки: пересчитывается
  после каждого пригодного fix в окне выставки и последний раз при окончательной выставке
  (через ~3 с), дальше не меняется. В эти первые секунды координаты в этих системах могут
  сдвигаться скачками. В `map` и `utm` начала нет, скачки только от уточнения привязки.
  При `enu_origin_auto: false` начало ENU фиксировано с самого начала.
- **Вне карты (разворотные петли у конечных).** Каждый прогон начинается на конечной перед
  петлёй (s < 0, до начала pathgraph) и заканчивается после конца карты (s > L) в петле
  другой конечной. Геометрия петель лежит в `maps/loops.json`. Её офлайн строит
  `tools/build_loops.py` по RTK-фиксам master-антенны **обучающих** bag (решение команды,
  рисунок `docs/plots/loops.png`): осевая линия по биннингу фиксов, отклонение дуги от хорды
  не больше 0.25 м. Петля S (конец T2S → начало S2T) имеет ветку A. Петля T (конец S2T →
  начало T2S) имеет ветки A, B и C. T/B — двухпутный участок: разворот за пределами записей
  ни разу не наблюдался, и там стоит заглушка. Кластеры остановок не используются.
  - До входа на карту (s < 0): если стартовый fix лежит не дальше `loop_max_dist_m` = 8 м от
    ветки петли, выставка идёт на эту ветку (тип `loop`, `loop_sigma0_m` = 1.5 м), иначе
    строится G1-мост Эрмита от точки старта к началу маршрута, как в первой версии.
  - После конца карты (s > L): путь продолжается по ветке петли (хвост). Без GNSS в середине
    маршрута ветка по умолчанию A. Пачка GNSS-фиксов, большинство которых легло на другую
    ветку, переключает на неё голову или хвост. За концом ветки остаётся экстраполяция с
    затухающей кривизной (`tail_mode: decay`).
  - `loops_enable:=false` возвращает мост и хвост первой версии. На тестовом bag это даёт
    3D RMSE 23.09 м против 1.11 м (`results/checker_30618_88aea4d9/summary.md`, `no_loops`).
- **Коррекция положения по GNSS (`gnss_aiding: true`).** Организаторы разрешили использовать
  GNSS в середине маршрута для положения, но не для скорости. После выставки:
  - fix собираются в пачки: пачка закрывается после паузы больше `gnss_burst_gap_s` = 0.5 с
    или на `gnss_burst_max_fixes` = 20 fix;
  - каждый fix ставится на историю одометра (`gnss_hist_s` = 30 с) по своему header stamp,
    rover-fix сдвигаются назад на 12.44 м к master-антенне;
  - fix проецируется на путь прогона (голова-петля + pathgraph + хвост-петля) в окне
    ±max(80 м, 5σ_along) вокруг прогноза (`RunPath.project_near`) и проходит ворота по
    расстоянию до пути: 2 м для RTK (status 2), 6 м для остальных. Если в пачке не меньше
    2 RTK-fix, берутся только они;
  - невязка пачки ν — медиана разностей вдоль пути. Если медианное отклонение от неё больше
    1.0 м (RTK) или 4.0 м (иначе), пачка отбрасывается (`gnss_rejected`);
  - принятая пачка ставит якорь фильтром Калмана вдоль пути:
    $k = \sigma^2_{prior}/(\sigma^2_{prior}+\sigma^2)$, $\sigma$ = `gnss_sigma_rtk_m` = 1.2 м
    (не RTK: 3.0 м; откалибровано на CV по покрытию 95 %);
  - дальше положение считается от якоря:
    $s = s_{anchor} + (1+\varepsilon_{pos})(s_{odo} - s_{odo,anchor}) + \Delta_{TRN}$. Здесь
    $\Delta_{TRN}$ — смешивание с изменением поправки TRN после якоря (`gnss_trn_blend`);
  - масштаб $\varepsilon_{pos}$ обучается только по базе RTK → RTK, у которой оба якоря на
    pathgraph и база длиннее 300 м: коэффициент 0.3, ограничение ±2 %;
  - σ_along растёт между пачками как `gnss_drift_frac` · путь: 0.006, а после оценки масштаба
    0.004.

  TRN продолжает работать и остаётся источником уклона для модели скорости, поэтому **поток
  скорости побитово одинаков с коррекцией и без неё** (тест
  `test_bursts_correct_along_track_drift_and_never_touch_speed`). На тестовом bag коррекция
  уменьшает 3D RMSE с 7.16 до 1.11 м при той же v_rmse 0.0335 м/с (`no_gnss_aiding` против
  `final`). Диагностика: режим `nav_gnss`, поля `pipe_gnss_bursts`, `pipe_gnss_rejected`,
  `pipe_pos_eps`, `pipe_gnss_nu`.

### 5.4 Содержимое `nav_msgs/Odometry`

| поле | значение |
|---|---|
| `header.stamp` / `header.frame_id` | stamp входа (5.2) / выходной фрейм (5.3) |
| `child_frame_id` | `base_link` (`child_frame_id`) |
| `pose.pose.position` | x, y, z опорной точки на пути (по умолчанию `base_link`, z = z pathgraph; см. `output_along_offset_m`, `output_z_offset_m`) |
| `pose.pose.orientation` | Кватернион только рыскания (roll = pitch = 0). yaw = направление касательной пути в сторону движения |
| `pose.covariance` (6×6, x y z roll pitch yaw) | `[0,1,6,7]` = R(yaw)·diag(σ²_along, σ²_cross)·R(yaw)ᵀ; `[14]` = 0.5² (z); `[21]`, `[28]` = 0.02² (roll, pitch); `[35]` = σ²_yaw |
| `twist.twist.linear.x` | Продольная скорость, м/с (≥ 0), в `child_frame_id`; то же значение, что в `/result/velocity` (оценка на stamp − `output_velocity_delay_s`) |
| `twist.covariance` | `[0]` = σ²_v фильтра; `[7]`, `[14]` = 1e-4 (боковая и вертикальная скорость на рельсах); `[21,28,35]` = 1e6 (угловые скорости не оцениваются) |

Как считаются дисперсии:

Обозначения: `off` — расстояние за пределами карты вдоль пути, м (0 на карте; на мосту
`off` = −s, на хвосте `off` = s − L); параметры из `core/config.py`.

- **σ²_along:**
  - до срабатывания привязки по уклонам (TRN): σ0² выставки + дисперсия пути s фильтра;
  - после: (σ²_TRN + (ещё не применённая поправка)² + P_ss фильтра) · `pose_trn_sigma_scale`²,
    где `pose_trn_sigma_scale` = 0.6 (калибровка по покрытию 95 %-эллипса);
  - после первой принятой GNSS-пачки: апостериорная дисперсия якоря $(1-k)\sigma^2_{prior}$
    плюс рост $(f \cdot D)^2$, где D — путь от якоря, f = `gnss_drift_frac` (0.006; 0.004 после
    оценки масштаба). При смешивании с TRN рост умножается на $1-w$,
    $w = \sigma^2_{odo}/(\sigma^2_{odo}+\sigma^2_{TRN})$;
  - за концом ветки петли (или за концом карты без петель) добавляется
    (`tail_sigma_frac`·off)² + (`tail_cross_frac`·off)² (0.03 и 0.3); на мосту (s < 0)
    σ²_along не растёт: длина моста откалибрована.
- **σ²_cross:** `pose_sigma_cross_m`² = 0.8² на карте. На ветке петли (голова или хвост)
  + `loop_cross_sigma_m`² = 1.0² (геометрия ветки — медиана прогонов, хорды). На мосту
  + (`bridge_cross_frac`·off)² (0.06), за концом петли + (`tail_cross_frac`·off)² (0.3; там путь
  может повернуть, поэтому эта добавка изотропна и входит и в σ²_along).
- **σ²_yaw:** min(π, hypot(`pose_sigma_yaw_rad`, `offmap_yaw_rad_per_m`·off))² = 0.03² на
  карте и на ветке петли (off = 0), рост 0.005 рад на метр на мосту и за концом петли.
- **σ_z, крен и тангаж:** постоянные `pose_sigma_z_m` = 0.5 м и `pose_sigma_tilt_rad` = 0.02 рад.
- **Без GNSS (`no_gnss`):** σ²_along = дисперсия пути фильтра, σ²_cross = 0.8², σ²_yaw = 0.03².
- **Согласованность** (60 оценочных bag, эталон master, `results/<тег>/summary.md`). Доля
  эталонных точек внутри 95 %-эллипса опубликованной ковариации xy (`p_in95`) и средний
  Mahalanobis²/2 (`p_nees`, идеал 1), медиана / среднее по bag:

  | прогон | GNSS | `p_in95` | `p_nees` |
  |---|---|---|---|
  | `F3_final` (первая версия, без петель и коррекции) | первые 10 с | 0.998 / 0.956 | 0.245 / 1.05 |
  | `F4_final` (текущий код) | первые 10 с | 0.995 / 0.947 | 0.151 / 1.55 |
  | `F4_final_burst` (текущий код) | 30 с + пачка 1.5 с каждые 120 с | 0.98 / 0.929 | 0.431 / 4.69 |

  Ковариация в типичном прогоне консервативна, но есть хвост. В `F4_final` `p_in95` ниже 0.8
  у трёх bag: 30618_4d487b0d (0.73, al_rmse 7.63 м), 30639_92226df0 (0.58, 17.56 м) и
  30639_9c362687 (0.63, 12.98 м). Там систематическая along-track ошибка больше опубликованной
  σ_along (`results/F4_final/per_bag.json`). У 4d487b0d и 9c362687 часы GNSS сдвинуты на ±1 с
  на 18–60 с (такие сдвиги есть в 7 из 60 оценочных bag; `results/robustness/summary.md`,
  раздел 4, пункт «Флаг slip на 60 чистых заездах»). В `F4_final_burst` среднее `p_nees`
  4.69: после GNSS-якоря σ_along мала, и промахи пачек стоят дорого. Отдельно: часть прогонов S2T при s ≈ 500–1200 м идёт по
  не нанесённому на карту параллельному пути в 4.2 м справа, а σ_cross на карте 0.8 м,
  поэтому там эталон выпадает из эллипса поперёк пути (раздел 9).
- **Нечисловые значения.** NaN и inf в выходы не попадают. Выход с нечисловой скоростью
  отбрасывается целиком. Выход с нечисловой позой публикуется только как скорость.
  Нечисловые σ²_along и σ²_v публикуются как 1e6, отсутствующие σ²_cross и σ²_yaw заменяются
  на 0.8² и 0.03². Все такие случаи считаются в диагностике.

`VelocitySensor` на `/result/velocity`: тот же stamp, `frame_id = base_link`,
`velocity` = та же скорость в м/с, что в `twist.linear.x`. Чекер организаторов читает поле
`velocity` (параметр `result_velocity_field` по умолчанию).

### 5.5 Диагностика

`/backup_odometry/diagnostics` публикуется раз в секунду. Одна запись
`backup_odometry: estimator`, уровень OK/WARN/ERROR с пояснением. ERROR: «N callback
exceptions». WARN: «no input yet», «no input for X s» (нет входов дольше 1 с), «no pose yet
(initial alignment done|pending)», «wheel slip/slide suspected», «N non-finite outputs
dropped». Основные ключи:

| ключ | смысл |
|---|---|
| `mode` | `<этап>:<колёса>`. Этап: `init`, `init_provisional`, `nav` (выставка завершена), `nav_trn` (работает привязка по уклонам), `nav_gnss` (положение идёт от GNSS-якоря, 5.3), `no_gnss` (счисление без выставки). Колёса: `both`, `disagree` (тележки расходятся, подозрение на боксование или юз), `single_front`, `single_rear`, `model_only` |
| `slip`, `init_done`, `gnss_subscribed`, `has_pose` | флаги. `init_done` — GNSS больше не нужен (выставка завершена и коррекция выключена или сработало счисление); при `gnss_aiding: true` после выставки остаётся `False`. `gnss_subscribed=True`, пока подписки на GNSS живы |
| `pipe_gnss_bursts`, `pipe_gnss_rejected`, `pipe_pos_eps`, `pipe_gnss_nu` | принятые и отброшенные GNSS-пачки, оценка масштаба положения $\varepsilon_{pos}$, невязка последней пачки вдоль пути, м |
| `velocity_mps`, `sigma_along_m`, `output_frame`, `vehicle_id` | текущая оценка и настройка |
| `cb_p50_ms`, `cb_p95_ms`, `cb_max_ms`, `cb_max_ms_total` | время обработки callback: за окно 1 с и максимум за прогон |
| `input_idle_s`, `msgs_front/rear/cmd/master/rover/out_vel/out_pos/errors/stamp_fallback/nonfinite` | счётчики |
| `pipe_s_route`, `pipe_off_map_m` | координата вдоль маршрута (с учётом `output_along_offset_m`) и расстояние за пределами карты; есть только после выставки |
| `pipe_s_odo` | одометр фильтра: путь, пройденный с первого сообщения колёс, м |
| `pipe_delta`, `pipe_delta_std`, `pipe_eps` | поправка TRN вдоль пути, её σ и оценка масштаба |
| `pipe_gain_tr`, `pipe_gain_br`, `pipe_bias` | адаптивные коэффициенты тяги и торможения, смещение модели ускорения |
| `pipe_v_front`, `pipe_v_rear`, `pipe_slip_ratio_front/rear`, `pipe_adhesion_used`, `pipe_a` | скорости тележек, продольное скольжение (v_тележки − v)/max(v, 1), используемое сцепление \|a\|/g, ускорение |

`/backup_odometry/slip` (Bool, latched) публикуется только при смене состояния флага
боксования или юза.

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
| `--start S` | начать с S-й секунды bag. Выставке нужен GNSS в начале прогона |
| `--vehicle ID` | `vehicle_id`. По умолчанию берётся из префикса имени bag (`30618_`, `30639_`) |
| `--sim-time` | нода с `use_sim_time:=true`, воспроизведение с `--clock 100` |
| `--dev` / `--src DIR` | взять текущий `src/` (или DIR) и пересобрать пакет в контейнере вместо копии из образа |
| `--measure` | запустить рядом `tools/latency_probe.py` и `tools/resource_monitor.py` |
| `--image NAME` | образ (по умолчанию `tram_backup_odometry:humble`) |

- Всё выполняется в одном контейнере с `--network none` (DDS работает через loopback).
- Bag монтируется только для чтения, `<out_dir>` монтируется в `/out`. `<out_dir>/result_bag`
  не должен существовать заранее.

Результаты в `<out_dir>`:

- `result_bag/` — rosbag2 с `/result/velocity`, `/result/position`, `/backup_odometry/diagnostics`, `/backup_odometry/slip`;
- `node.log`, `play.log`, `record.log`;
- `check.txt` — сверка выходов с входным bag. Для каждого выходного топика: число сообщений,
  доля stamp, равных stamp входов (ожидается 100 %), частота по stamp, покрытие сетки 0.1 с,
  `frame_id` и `child_frame_id`, число дублей и убываний stamp, последний статус диагностики;
- с `--measure` дополнительно `latency.{txt,json,csv}` и `resources.{txt,json,csv}`.

### 6.2 `scripts/measure.sh`

```bash
scripts/measure.sh <bag_dir> [out_dir] [опции run_bag.sh]     # = run_bag.sh --measure --duration 120
scripts/measure.sh ../dataset/data/30618_27e994fc results/rt/27e994fc_full_final --duration 1905   # весь bag, 31 мин
```

`out_dir` по умолчанию: `${TMPDIR:-/tmp}/tram_measure_<bag>_<время>`.

- **`latency_probe.py`** подписывается на входы и выходы ноды. Задержка считается как
  разность времени прихода выхода и входа с тем же header stamp (по стенным часам). Отчёт даёт
  p50/p95/p99/max по всем выходам и отдельно после первых 2 с: каждый bag начинается с пачки
  примерно из 150 устаревших сообщений, и её очередь даёт десятки миллисекунд. Также
  выводятся частота и покрытие сетки 0.1 с.
- **`resource_monitor.py`** через psutil с частотой 2 Гц снимает CPU процесса ноды в ядрах и
  RSS в МиБ.

Наши замеры: Docker Desktop на Apple M-series (10 ядер), `ros2 bag play -r 1`, `vehicle_id`
из имени bag. Оба замера в таблице сняты на образе версии F3 (26.09). В ней ещё не было петель,
коррекции по GNSS, выхода в `base_link` и задержки скорости. Короткий замер: 30618_0e41eac3,
первые 120 с, `results/rt/0e41eac3_120s_final/`. Длинный: весь 31-минутный bag
30618_27e994fc, 1900 с, `results/rt/27e994fc_full_final/`. Замеры ещё более ранней сборки
(память дедупликации stamp 16 вместо 64, без режима `enu`) оставлены для сравнения в
`results/rt/0e41eac3_120s/` и `results/rt/27e994fc_full/`.

Замер текущей версии снят под чекером организаторов на всём тестовом bag (22 мин,
`results/checker_ros_30618_88aea4d9_final/`, таблица в 7.1). Callback ноды: mean 0.693 мс,
p99 2.00 мс, max 6.32 мс. CPU 0.071 ядра в среднем, RSS не больше 64.1 МиБ. Задержка
после первых 2 с, p50 / p99 / max: 1.17 / 3.03 / 22.8 мс (скорость) и 1.90 / 3.78 / 24.9 мс
(положение). С учётом стартовой пачки bag максимум 32.1 / 32.4 мс (`latency.txt`).

| показатель | 0e41eac3, 120 с | 27e994fc, 1900 с | требование ТЗ |
|---|---|---|---|
| задержка вход → `/result/velocity` после 2 с: p50 / p99 / max | 0.74 / 1.73 / 5.5 мс | 0.45 / 1.53 / 28.0 мс | ≤ 100 мс |
| задержка вход → `/result/position` после 2 с: p50 / p99 / max | 1.41 / 2.73 / 7.2 мс | 0.88 / 2.37 / 28.2 мс | ≤ 100 мс |
| максимум с учётом стартовой пачки | 62 мс | 28 мс | пик ≤ 250 мс |
| выходов / частота по stamp | 3645 / 29.9 Гц | 55 948 / 29.4 Гц | ≥ 10 Гц (реком. 20–50) |
| stamp выходов = stamp входов; покрытие сетки 0.1 с | 100 %; 100 % | 100 %; 100 % | |
| повторные stamp в `/result/velocity` | 0 | 0 | |
| убывания stamp в `/result/velocity` (`check.txt`) | 1177 | 14 749 | |
| CPU ноды: среднее / p95 / max | 0.060 / 0.080 / 0.26 ядра | 0.044 / 0.080 / 0.26 ядра | ≤ 2 ядра |
| RSS: старт / max | 62.0 / 63.4 МиБ | 62.1 / 63.8 МиБ (среднее 63.7) | ≤ 0.5 ГБ |
| callback ноды: mean / p99 / max | 0.53 / 1.50 / 5.5 мс | 0.37 / 1.12 / 17.1 мс | |
| исключения / нечисловые выходы | 0 / 0 | 0 / 0 | |

- **Память и стабильность.** За 31 минуту RSS выходит на 63.7–63.8 МиБ и дальше не растёт.
  Ошибок и нечисловых выходов нет, нода завершается штатно (`process has finished cleanly`).
- **Пики задержки.** На 27e994fc максимум 28 мс — единичный пик на 1550-й секунде (максимум
  callback ноды за прогон 17.1 мс); после первых 2 с задержку выше 10 мс имеют 18 из 111 610
  выходов обоих топиков. Стартовая пачка на этом bag даёт до 24 мс.
- **Метрика «относительно первого прихода stamp».** Это консервативная граница: задержка
  считается от первого входа с тем же stamp. На образе F3 её максимум 62 мс на
  0e41eac3 и 28 мс на 27e994fc. На предыдущей сборке было 69 и 231 мс: задний канал в
  стартовой пачке присылает свой backlog раньше переднего, сообщение передней тележки с тем
  же stamp приходит примерно через 0.22 с, после 25–51 других выходов, и сборка с памятью 16
  публиковала его повторно.
- **Дубли stamp.** Предыдущая сборка: 25 дублей на 0e41eac3 и 4 на 27e994fc, все в первые
  0.3 с после первого выхода. В финальном коде память 64 (`core/pipeline.py`, `STAMP_MEMORY`),
  дублей 0 в обоих замерах.

---

## 7. Проверка точности: чекер организаторов и офлайн-оценка

Два независимых способа:

- 7.1: **чекер организаторов** `hackathon_solution_checker` на их тестовом bag с эталоном
  `/localization/kinematic_state`. Эта метрика совпадает со скрытой проверкой;
- 7.2–7.5: **офлайн-оценка по всему датасету** (122 bag) против GNSS-эталона. Она нужна,
  потому что в датасете нет `/localization/kinematic_state`.

### 7.1 Проверка чекером организаторов

**Что считает чекер** (`../check_code/check-code/src/checker_ros/hackathon_solution_checker/metrics.py`):

- **Эталон:** `/localization/kinematic_state` (`nav_msgs/Odometry`, 50 Гц, система `map`,
  точка `base_link` на pathgraph, z = z pathgraph).
- **Сопоставление:** два отдельных `message_filters.ApproximateTimeSynchronizer`: эталон с
  `/result/velocity` и эталон с `/result/position`. Параметры: очередь 100, slop 0.05 с,
  сопоставление по header stamp.
- **Ошибка скорости:** поле `velocity` нашего `VelocitySensor` минус `twist.twist.linear.x`
  эталона.
- **Ошибка положения:** по x, y, z и 3D-расстояние. Для каждой считаются RMSE, max и n.
- **Отчёт:** печатается каждые 5 с и один раз при завершении (SIGINT).
- **Параметры чекера:** `reference_topic`, `velocity_topic`, `position_topic`,
  `result_velocity_field`, `sync_tolerance_sec`, `sync_queue_size`, `report_period_sec`.
  Мы запускаем его со значениями по умолчанию.

#### (а) Офлайн-эмуляция: `tools/checker_sim.py`

Запускается хостовым Python без ROS и Docker, по несколько секунд на вариант. Тестовый bag
читается напрямую (`bagio` + разбор `nav_msgs/Odometry`) и прогоняется через тот же `Pipeline`,
что в ноде.

Как устроена эмуляция:

- сообщения подаются в порядке времени записи;
- выход «приходит» в чекер через 1 мс после времени записи своего входа;
- синхронизатор эмулируется так: при каждом приходе берётся ближайший по stamp элемент другой
  очереди в пределах slop, и оба элемента удаляются из очередей;
- метрики те же, что у чекера, плюс медиана, p95 и RMSE по минутам.

```bash
export OMP_NUM_THREADS=1 PYTHONPATH=src/tram_backup_odometry
PY=/opt/miniconda3/envs/ml/bin/python        # любой Python 3 с numpy/scipy/PyYAML
$PY tools/checker_sim.py --gnss-start-only 35 --out results/checker_30618_88aea4d9   # все варианты (~1 с CPU на вариант)
$PY tools/checker_sim.py --variants final                                             # только итоговая конфигурация
$PY tools/checker_sim.py --variants final,no_gnss_aiding --set gnss_sigma_rtk_m=2.0   # свои переопределения Params
```

Опции:

| опция | по умолчанию | смысл |
|---|---|---|
| `--bag` | `../check_code/check-code/bags/30618_88aea4d9` | каталог rosbag2 с `/localization/kinematic_state` |
| `--vehicle-id` | `30618` | `Params.vehicle_id`; `''` = неизвестный вагон, K = 3.595 |
| `--variants` | `all` | `all`, `final` или список через запятую из таблицы ниже |
| `--gnss-start-only N` | — | дополнительный вариант `final_gnss_first_Ns_only`: итоговая конфигурация, но GNSS подаётся только первые N с после первого fix |
| `--set k=v ...` | — | переопределения `Params` для всех вариантов |
| `--out DIR` | — | записать `summary.json` и `summary.md` |

Варианты:

| вариант | переопределения | что показывает |
|---|---|---|
| `final` | нет (значения по умолчанию) | итоговая конфигурация: `base_link`, z pathgraph, задержка скорости 0.09 с, петли, коррекция по GNSS |
| `no_gnss_aiding` | `gnss_aiding=False` | GNSS только в окне выставки, дальше чистое счисление |
| `no_loops` | `loops_enable=False` | без геометрии разворотных петель: мост G1 до карты и экстраполированный хвост после неё |
| `no_velocity_delay` | `output_velocity_delay_s=0` | вклад задержки скорости |
| `master_antenna_point` | `output_along_offset_m=0`, `output_z_offset_m=3.10` | прежняя выходная точка: master-антенна, z антенны |
| `pre_checker_version` | всё перечисленное выше выключено | поведение версии до уточнений организаторов |

Результат на `30618_88aea4d9` (`results/checker_30618_88aea4d9/summary.md`; команда из первой
строки блока выше). Во всех вариантах эталон содержит 65 579 сообщений, выходов 38 476,
пар 38 439 и по скорости, и по положению:

| вариант | v RMSE, м/с | 3D RMSE, м | 3D медиана, м | 3D p95, м | 3D max, м | z RMSE, м | пачки GNSS: принято/отклонено |
|---|---|---|---|---|---|---|---|
| `final` | **0.0335** | **1.11** | 0.50 | 1.98 | 14.0 | 0.04 | 49/1 |
| `no_gnss_aiding` | 0.0335 | 7.16 | 4.20 | 6.55 | 50.7 | 0.09 | 0/0 |
| `no_loops` | 0.0335 | 23.09 | 0.89 | 28.66 | 173.8 | 0.25 | 46/4 |
| `no_velocity_delay` | 0.0566 | 1.11 | 0.50 | 1.98 | 14.0 | 0.04 | 49/1 |
| `master_antenna_point` | 0.0335 | 10.26 | 10.22 | 11.47 | 16.5 | 3.07 | 49/1 |
| `pre_checker_version` | 0.0566 | 24.24 | 6.52 | 30.50 | 169.3 | 3.15 | 0/0 |
| `final_gnss_first_35s_only` | 0.0335 | 7.02 | 4.11 | 5.67 | 50.8 | 0.08 | 29/1 |

3D RMSE по минутам записи, м (из того же `summary.md`):

| вариант | 0 с | 60 с | 120 с | 420 с | 540 с | 900 с | 1200 с | 1260 с (последняя минута) |
|---|---|---|---|---|---|---|---|---|
| `final` | 1.2 | 0.9 | 1.0 | 1.2 | 1.7 | 0.3 | 0.2 | 3.7 |
| `no_gnss_aiding` | 0.5 | 0.2 | 0.4 | 2.1 | 6.2 | 4.3 | 2.9 | 30.2 |
| `final_gnss_first_35s_only` | 1.2 | 0.9 | 1.0 | 1.6 | 5.3 | 4.2 | 3.2 | 30.3 |
| `no_loops` | 18.6 | 20.5 | 11.0 | 1.2 | 1.7 | 0.3 | 4.8 | 112.7 |

Что видно из таблиц:

- **Скорость.**
  - Скорость не зависит ни от GNSS, ни от петель: v RMSE 0.0335 м/с во всех вариантах с
    задержкой.
  - Задержка 0.09 с снижает v RMSE с 0.0566 до 0.0335 м/с (MAE с 0.0378 до 0.0195, max с
    0.380 до 0.324 м/с; `summary.json`).
- **Выходная точка и z.**
  - Прежняя точка (master-антенна, z + 3.10) даёт почти постоянную ошибку около 10.2 м
    (медиана 10.22 м). Её составляют смещение 9.873 м вдоль пути до `base_link` и 3.07 м по z.
  - Переход на `base_link` и z pathgraph даёт основной выигрыш: 10.26 → 1.11 м.
- **Петли.**
  - Тестовый bag начинается на разворотной петле. С `loops.json` выставка идёт на ветви петли
    (`init_kind` = `loop`), без него через мост G1 (`bridge`).
  - Без петель ошибка первых двух минут 18.6–20.5 м, а на последней минуте 112.7 м.
- **Коррекция по GNSS.**
  - В этом bag GNSS есть на всём маршруте. Коррекция снижает 3D RMSE с 7.16 до 1.11 м.
  - Если GNSS есть только в первые 35 с, результат почти как без коррекции (7.02 м).
  - Итог на скрытой проверке зависит от того, будет ли там GNSS в середине маршрута
    (вопрос в разделе 9).
- **Последняя минута.**
  - Последняя минута записи проходит по хвосту после петли. Без GNSS там 30 м, с GNSS 3.7 м.
  - Ветвь хвоста без GNSS не выбрать, по умолчанию берётся ветвь A (5.3).
- **Итог:** `pre_checker_version` против `final` на этом bag: v RMSE 0.0566 → 0.0335 м/с,
  3D RMSE 24.24 → 1.11 м.

Эмуляция — это не сам чекер. Порядок прихода сообщений в ROS зависит от планировщика, а
задержку выхода мы считаем постоянной (1 мс). Сверка с настоящим чекером приведена ниже.

#### (б) Настоящий чекер в Docker: `scripts/run_checker.sh`

```bash
./scripts/build.sh                                                                 # образ tram_backup_odometry:humble
./scripts/run_checker.sh ../check_code/check-code/bags/30618_88aea4d9 /tmp/chk_88aea4d9 --measure
cat /tmp/chk_88aea4d9/checker_final.txt
./scripts/run_checker.sh ../check_code/check-code/bags/30618_88aea4d9 /tmp/chk_120 --duration 120   # короткая проверка
```

Использование: `scripts/run_checker.sh <bag_dir> <out_dir> [опции] [-- аргументы launch]`.

| опция | по умолчанию | смысл |
|---|---|---|
| `--rate R` | 1.0 | скорость `ros2 bag play`; чекер рассчитан на реальное время |
| `--duration S` | весь bag | остановить воспроизведение через S секунд |
| `--vehicle ID` | из префикса имени bag (`30618_`, `30639_`), иначе не задаётся | аргумент launch `vehicle_id` |
| `--measure` | выкл. | параллельно запустить `tools/latency_probe.py` и `tools/resource_monitor.py`, как в `run_bag.sh` |
| `--checker DIR` | `../check_code/check-code/src/checker_ros` | исходники пакета чекера |
| `--image NAME` | `tram_backup_odometry:humble` (или переменная окружения `IMAGE`) | Docker-образ из `scripts/build.sh` |

Что делает скрипт (один контейнер, `--network none`; bag и исходники чекера монтируются
только для чтения, `<out_dir>` монтируется как `/out`):

1. Копирует чекер и собирает только `hackathon_solution_checker` (`colcon build
   --packages-select`) поверх `/ws/install`, то есть против нашего `tram_vehicle_msgs`, который
   совпадает с пакетом датасета. Если в `package.xml` нет `<maintainer>`, добавляет
   заглушку, как в `docker/Dockerfile`. Лог сборки пишется в `checker_build.log`.
2. Запускает ноду (`ros2 launch tram_backup_odometry backup_odometry.launch.py
   use_sim_time:=false vehicle_id:=...`) и ждёт строку `ready:`. Затем запускает чекер
   `ros2 run hackathon_solution_checker metrics` и ждёт строку `Comparing`. Дальше стартуют
   `ros2 bag record` для `/result/*`, `/localization/kinematic_state` и диагностики и, с
   `--measure`, пробы задержки и ресурсов.
3. Через 3 с запускает `ros2 bag play -d 1 -r R`. После конца воспроизведения ждёт 3 с,
   останавливает пробы, посылает чекеру SIGINT (он печатает итоговый отчёт), затем
   останавливает запись и ноду.

Результаты в `<out_dir>`:

- **`checker_final.txt` / `checker_final.json`:** итоговые RMSE, max и n чекера из отчёта при
  завершении. Если его нет, берётся последний 5-секундный отчёт, и это явно помечается;
- **логи:** `checker.log` (все периодические отчёты), `checker_build.log`, `node.log`,
  `play.log`, `record.log`;
- **`result_bag/`:** выходы ноды и воспроизведённый эталон; время записи = время прихода;
- **с `--measure`:** `latency.{txt,json,csv}` и `resources.{txt,json,csv}`.

Ограничения скрипта:

- `<out_dir>/result_bag` не должен существовать, так что для повторного запуска нужен новый
  каталог.
- Launch-файл объявляет только `params_file`, `output_frame`, `vehicle_id`, `use_sim_time`,
  `maps_dir` и `log_level`. Прочие аргументы после `--` (например, `gnss_aiding:=false`)
  `ros2 launch` **молча игнорирует**. Любой другой параметр передаётся через частичный YAML в
  `<out_dir>` (нода объявляет все параметры со значениями по умолчанию, 4.1):

  ```bash
  mkdir -p /tmp/chk_noaid
  printf 'backup_odometry:\n  ros__parameters:\n    gnss_aiding: false\n' > /tmp/chk_noaid/noaid.yaml
  ./scripts/run_checker.sh ../check_code/check-code/bags/30618_88aea4d9 /tmp/chk_noaid -- params_file:=/out/noaid.yaml
  ```

#### Результат настоящего чекера на `30618_88aea4d9`

Команда: `scripts/run_checker.sh ../check_code/check-code/bags/30618_88aea4d9
results/checker_ros_30618_88aea4d9_final --measure`. Условия: Docker Desktop на Apple
M-series, итоговый образ (`.work/build_final3.log`), `-r 1`, весь bag (1309.6 с по
`metadata.yaml`), `vehicle_id` = 30618, на хосте нет параллельных тяжёлых расчётов (27.09,
17:30–17:52). Итоговый отчёт чекера (`results/checker_ros_30618_88aea4d9_final/checker_final.txt`,
отчёт при завершении, до него 263 периодических):

| метрика чекера | RMSE | max | n | офлайн-эмуляция (`final`) |
|---|---|---|---|---|
| скорость `velocity`, м/с | **0.0324** | 0.302 | 38 437 | 0.0335 / 0.324, n 38 439 |
| x, м | 0.904 | 11.02 | 38 437 | 0.907 |
| y, м | 0.645 | 8.67 | 38 437 | 0.647 |
| z, м | 0.037 | 0.288 | 38 437 | 0.037 |
| distance (3D), м | **1.111** | 14.02 | 38 437 | 1.115 / 14.02 |

- **Эмуляция и чекер совпадают.** Расхождение по положению меньше 0.01 м, по скорости
  0.0011 м/с. Число пар отличается на 2. Предварительный прогон на образе до последней
  правки (`results/checker_ros_30618_88aea4d9/`) дал 0.0322 м/с и 1.116 м.
- **Лог ноды** (`node.log`): `vehicle_id='30618' k=3.5970 output_frame=map`. Первая поза
  получена через 5.4 с после `ready` по стенным часам: сюда входят 3 с ожидания скрипта и
  `-d 1`. Строки `GNSS subscriptions destroyed` нет: коррекция включена. При завершении:
  входов front 12 216, rear 12 225, cmd 26 249, fix master 438, rover 435; выходов velocity
  38 476, position 38 473; ошибок, подмен stamp и нечисловых выходов 0; нода завершилась
  штатно.
- **Реальное время** (`--measure`, `latency.txt`, `resources.txt`):

  | показатель | значение |
  |---|---|
  | задержка вход → `/result/velocity`: p50 / p95 / p99 / max | 1.17 / 2.45 / 3.03 / 22.8 мс (после 2 с); max 32.1 мс за весь прогон |
  | задержка вход → `/result/position`: p50 / p95 / p99 / max | 1.90 / 3.23 / 3.78 / 24.9 мс (после 2 с); max 32.4 мс за весь прогон |
  | частота по stamp; покрытие сетки 0.1 с; дубли stamp | 29.3 Гц; 99.99 % / 99.98 %; 0 |
  | callback ноды: mean / p50 / p95 / p99 / max | 0.693 / 0.668 / 1.585 / 1.995 / 6.32 мс |
  | CPU ноды: среднее / p95 / max | 0.071 / 0.100 / 0.259 ядра |
  | RSS: старт / среднее / max | 62.2 / 64.0 / 64.1 МиБ |

  Ни один из 76 949 выходов обоих топиков не превысил 100 мс; больше 10 мс — 64 выхода,
  почти все в стартовой пачке bag. В предварительном прогоне
  (`results/checker_ros_30618_88aea4d9/`) было 10 выходов выше 100 мс (max 202–204 мс):
  они совпали по времени с `tools/run_cv.py --jobs 6`, который шёл на том же хосте, при
  callback ноды не больше 13.4 мс.
- **Версия кода.** Замер снят на итоговом образе из `.work/build_final3.log` (тесты
  `120 passed, 11 skipped`).

### 7.2 Офлайн-оценка по датасету: кэш и индекс

Офлайн-инструменты вызывают **тот же класс `Pipeline`**, что и ROS-нода. Сообщения подаются
в порядке записи bag с теми же header stamp. Поэтому `tram_backup_odometry.replay` выдаёт то
же, что нода при `ros2 bag play`, пока порядок доставки совпадает с порядком записи.
ROS для этого не нужен. Команды выполняются из корня решения:

```bash
export PY=python3            # любой Python 3 с numpy, scipy, matplotlib, PyYAML
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
```

Датасет нужен только двум инструментам. `build_cache.py` читает bag из `../dataset/data`
(другой путь: `--data`). `bag_index.py` берёт время старта и длительность из
`../dataset/data/<bag>/metadata.yaml`; если файла нет, использует время записи первого и
последнего сообщения из кэша. Все остальные инструменты работают только по кэшу
`.work/cache/<bag>.pkl` и индексу `.work/bag_index.json`.

```bash
$PY tools/build_cache.py                  # все bag -> .work/cache/<bag>.pkl (параллельно; существующие пропускаются)
$PY tools/build_cache.py 0e41eac3 --force # выбранные bag (полное имя или уникальный суффикс), перезаписать
$PY tools/build_cache.py --list           # что есть в кэше
$PY tools/bag_index.py                    # -> .work/bag_index.json
```

- **`build_cache.py`** разбирает sqlite и CDR без ROS (`tram_backup_odometry.bagio.load_bag`).
  Файлы пишутся атомарно (временный файл, затем rename). Опции: `--data`, `--out`, `--jobs`
  (по умолчанию min(8, число ядер − 2)), `--force`, `--list`. Весь кэш (122 файла) занимает
  около 0.5 ГБ.
- **`bag_index.py`** строит индекс. Для каждого bag: машина, день, группа = машина_день
  (единица leave-one-group-out), fold, длительность, покрытие RTK, направление T2S/S2T,
  дубликаты (md5 скорости передней тележки) и `eval_ok` (уникальный, ≥ 60 с RTK-эталона).
  На текущих данных: 122 bag, 97 уникальных, 60 `eval_ok`, 6 групп.

### 7.3 Прогон и сводка: `tools/run_cv.py`

```bash
$PY tools/run_cv.py --tag F4_check --bags all --antenna all --jobs 6   # ~22 с на 6 процессах M-series (по умолчанию ядер − 2)
$PY tools/run_cv.py --compare F4_final F4_check                         # сравнение двух прогонов
$PY tools/run_cv.py --tag burst --bags all --antenna all --gnss-limit 30 --gnss-burst 120 1.5   # GNSS в середине маршрута
$PY tools/run_cv.py --tag x --bags 'eval&dir:S2T' --set trn_enable=false --plots worst
```

Финальная таблица получена командой `tools/run_cv.py --tag F4_final --jobs 6 --bags all --antenna all`
(аргументы сохранены в `results/F4_final/summary.json`, поле `argv`; `wall_s` = 22.0 с).
Основные числа (медианы по 60 `eval_ok` bag, эталон master): v_rmse 0.0295 м/с, al_rmse 1.98 м
(T2S 1.58, S2T 2.5), e3d 1.66 м, дрейф в конце 0.047 % пути. Против rover-эталона e3d
12.2 м, против середины базы 5.7 м. Подробнее в [RESULTS.md](RESULTS.md).

**Эталон CV — master-антенна, а не `base_link`.** В датасете нет
`/localization/kinematic_state`, поэтому эталоном служат RTK-фиксы master. `run_cv.py`
(`build_params`, им же пользуется `plots.py`) перед `--set` принудительно выставляет точку
master-антенны: `output_along_offset_m=0`, `output_z_offset_m=3.10` и
`output_velocity_delay_s=0` (скорость GNSS не отстаёт от колёс). Остальные параметры берутся
по умолчанию, в том числе `gnss_aiding` и `loops_enable`. Проверка в точке `base_link` против
эталона организаторов описана в 7.1.

- **Выборки bag:** `all`, `eval`, `unique`, `nogps`, `noeval`, `vehicle:30639`, `group:<g>`,
  `fold:<k>`, `dir:T2S`, `@file`, идентификаторы bag (достаточно суффикса-хэша). Запятая
  означает объединение, `&` пересечение.
- **Эталон:** `--antenna master|rover|mid|all`. Первая антенна основная, остальные идут в
  сводку дополнительно.
- **Параметры:** значения `Params` по умолчанию, поверх них `--params yaml`, затем
  `--fold-params DIR` (`fold_<k>.yaml` вне фолда), затем `--set k=v`.
- **GNSS:** подаётся только первые `--gnss-limit` = 10 с после первого fix. Это худший
  случай для коррекции: GNSS есть только на старте, как было в ТЗ до уточнений. С
  `--gnss-burst PERIOD DUR` после этого окна каждые PERIOD с подаётся ещё пачка длиной DUR с,
  то есть имитируется редкий GNSS в середине маршрута. `vehicle_id` берётся из имени bag.
- **Результат** пишется в `results/<tag>/{summary.md, summary.json, per_bag.json}`.
  - `summary.md` содержит медианы, средние, p90 и max по 60 `eval_ok` bag, разбивку по
    направлению, машине и группе, сравнение с rover- и mid-эталоном, худшие bag;
  - отдельный блок «робастность и тайминг» по всем 122 bag: исключения, NaN,
    отрицательные скорости, stamp не из входов, частота, CPU на сообщение;
  - опции `--plots worst|all|<bags>` и `--save-est` (`results/<tag>/est/<bag>.npz`).

Уже посчитанные прогоны лежат в `results/`:

| тег | что это |
|---|---|
| `F4_final` | итоговая конфигурация (петли, коррекция по GNSS, TRN), GNSS первые 10 с: медианы al_rmse 1.98 м, e3d 1.66 м, v_rmse 0.0295 м/с |
| `F4_final_burst` | то же, `--gnss-limit 30 --gnss-burst 120 1.5` (пачка GNSS 1.5 с каждые 120 с): al_rmse 0.834 м (среднее 1.97), e3d 0.82 м, дрейф 0.0125 % |
| `F3_final` | первая версия до уточнений организаторов (без петель и коррекции по GNSS): al_rmse 3.03 м, e3d 4.74 м |
| `B1_naive` | воспроизводимый наивный baseline (`--estimator baseline_naive:run_bag`, `tools/baseline_naive.py`): средняя скорость колёс / K по тому же пути, та же выставка и выходная СК, без EKF, модели тяги, детектора юза, TRN, петель и коррекции. Медиана al_rmse 2.81 м, среднее 5.91 м, p90 12.7 м (у `F4_final` 1.98 / 3.15 / 7.75 м); v_rmse 0.0541 м/с |
| `B1_naive_kf` | тот же baseline с безмодельным кинематическим фильтром Калмана [s, v, a] (`baseline_naive:run_bag_kf`): v_rmse 0.0296 м/с, al_rmse 3.13 м (медианы) |
| `AF_*` | абляции от `F4_final` по одному механизму: `noadapt`, `trn_off`, `tail_straight`, `nobias`, `noinflate`, `nolook`, `nopairs`, `nosc`, `scalefb`, `noloops` (`loops_enable=false`), `noaiding` (`gnss_aiding=false`) |
| `G_limit0`, `G_limit05`, `G_limit1`, `G_limit3`, `G_limit5`, `G_limit30` | `F4_final` с GNSS 0 (одна эпоха), 0.5, 1, 3, 5, 30 с после первого fix |
| `_deliv_novid` | версия F3 без `vehicle_id`, K = 3.595 |
| `checker_30618_88aea4d9` | `tools/checker_sim.py`: эмуляция чекера организаторов на их тестовом bag (7.1) |
| `checker_ros_30618_88aea4d9_final` | `scripts/run_checker.sh --measure`: настоящий чекер в Docker на итоговом образе, реальное время (7.1) |
| `checker_ros_30618_88aea4d9` | предварительный прогон того же чекера (образ до последней правки, хост нагружен CV) |
| `robustness` | результаты `tools/robustness.py` (7.5) |
| `rt/` | замеры реального времени (раздел 6) |

### 7.4 Один bag вручную

```bash
# replay (тот же Pipeline, что в ноде) -> est.npz; внутри Docker: ros2 run tram_backup_odometry replay ...
# точка выхода = master-антенна, без задержки скорости: так же, как в tools/run_cv.py
PYTHONPATH=src/tram_backup_odometry $PY -m tram_backup_odometry.replay .work/cache/30618_0e41eac3.pkl \
    --out /tmp/est.npz --set output_along_offset_m=0 output_z_offset_m=3.10 output_velocity_delay_s=0 \
    [publish_on=cmd ...]
# оценка против GNSS-эталона
$PY tools/evaluate.py 30618_0e41eac3 --est /tmp/est.npz [--antenna rover|mid] [--clean]
$PY tools/evaluate.py 30618_0e41eac3 --self-test   # эталон в роли оценки -> все ошибки 0 (без bag: по всем bag кэша)
$PY tools/evaluate.py 30618_0e41eac3 --self-test interp   # эталон, интерполированный в stamp входов (нижняя граница)
# рисунок: скорость, ошибки, along-track, карта с зумами на начало и конец
$PY tools/plots.py --tag mytag --bags 30618_0e41eac3,40ffd323   # -> results/mytag/plots/<bag>.png
```

Переопределения точки выхода в `--set` обязательны для сравнения с `evaluate.py`: его эталон —
master-антенна, а по умолчанию пакет публикует `base_link` (+9.873 м вдоль пути, $z$ pathgraph) и
скорость с задержкой 0.09 с под чекер организаторов. Без них along-track смещён на 9.873 м, а к
ошибке скорости добавляется задержка. `tools/run_cv.py` ставит эти значения сам (`build_params`).
CLI `replay` подаёт GNSS весь прогон, а CV — только первые 10 с, поэтому число одного bag как в
CV даёт `$PY tools/run_cv.py --tag mytag --bags 30618_0e41eac3`.

`replay` сам определяет `vehicle_id` по имени bag. Для файла из кэша имя берётся из
`<bag>.pkl`: у `.work/cache/30618_0e41eac3.pkl` префикс `30618` распознаётся, так как
учитываются первые 5 символов имени.

**Методика `evaluate.py`** (подробно в docstring модуля):

- **Эталон положения:** fix master-антенны со `status == 2`, переведённые в систему `map`;
  z = GNSS altitude.
- **Эталон скорости:** `hypot(vx, vy)` из `/sensing/gnss/master/vel` в моменты, где рядом
  (±0.05 с) есть fix со status 2.
- **Сопоставление:** для каждого эталонного отсчёта берётся ближайший stamp оценки в пределах
  0.05 с. Отсчёт без позы считается пропуском.
- **Метрики скорости:** RMSE, MAE, bias, bias по режимам (разгон, торможение, стоянка, выбег).
- **Метрики положения:**
  - e2d и e3d;
  - along- и cross-track относительно полилинии маршрута (только точки не дальше 5 м от
    карты и в её пределах);
  - дрейф в конце в % пройденного пути;
  - дрейф на карте;
  - согласованность ковариации (`p_in95`, `p_nees`);
  - сводный `score_loss` для ранжирования вариантов.

### 7.5 Робастность и отчётные графики

```bash
$PY tools/robustness.py --list                     # список сценариев отказов
$PY tools/robustness.py                            # всё: faults, clean, real, plots (--jobs 4 по умолчанию)
$PY tools/robustness.py --part faults --bags 30618_073f08d1 --scenarios drop_both_30 --jobs 1
$PY tools/report_plots.py                          # все рисунки отчёта -> docs/plots/*.png
$PY tools/report_plots.py --only realtime,ablations
```

- **`tools/inject_faults.py`** — синтетические отказы на разобранном bag: пропуски и
  заморозка тележек, нули, выбросы, шум, ошибка масштаба, боксование, скачки stamp,
  пропуск и заморозка команды, NaN. `apply(D, specs, seed=0)` возвращает изменённую копию,
  которую можно передать в `replay.run_bag(bag, data=D)`.
- **`tools/robustness.py`** — набор проверок робастности через тот же `Pipeline`:
  - `--part faults`: 12 фиксированных оценочных bag (по 6 T2S и S2T, обе машины) прогоняются
    чистыми и с каждым сценарием. Сценарии: молчание одной или обеих тележек 5–60 с, в том
    числе при торможении; нули, заморозка и NaN; выбросы, шум, ошибка масштаба 5 %;
    боксование и юз одной или обеих тележек; скачки stamp ±0.5 с; пропуск и заморозка
    команды. Эталон всегда строится по чистым данным. Метрики: отсутствие падений,
    конечность выходов, stamp = входные, частота, ошибка скорости в окне отказа, время
    восстановления, смещение вдоль пути против чистого прогона и против наивного среднего
    тележек;
  - `--part clean`: частота флага боксования на 60 чистых bag (ложные срабатывания),
    используемое сцепление, адаптация коэффициентов тяги;
  - `--part real`: реальные аномальные bag (боксование, выпадения тележек 30639, скачки
    stamp, артефакт команды);
  - `--part plots`: рисунки `docs/plots/robust_*.png`.

  Результаты пишутся в `results/robustness/{summary.md, summary.json, per_bag.json}`.
  Опции: `--part`, `--bags`, `--scenarios`, `--jobs` (по умолчанию 4), `--seed`,
  `--gnss-limit`, `--out`, `--plots` (каталог рисунков), `--list`.
- **`tools/report_plots.py`** — рисунки для отчёта в `docs/plots/`: пример скорости,
  распределение ошибки скорости, along-track вдоль маршрута, траектории с зумами начала и
  конца, метрики по bag, абляции, реальное время, ковариация. Агрегаты берутся из
  `results/<--tag>/` (по умолчанию `F4_final`; абляции: `B1_naive`, `F4_final`, `AF_*`,
  `G_limit*`), реальное время из `--rt-short` и `--rt-full`
  (по умолчанию замеры ранней сборки `results/rt/0e41eac3_120s` и
  `results/rt/27e994fc_full`; для замеров образа F3 передайте
  `--rt-short results/rt/0e41eac3_120s_final --rt-full results/rt/27e994fc_full_final`). Опции: `--only`
  (список из `speed_example, speed_error, along_track, trajectory, per_bag, ablations,
  realtime, covariance`), `--tag`, `--jobs` (по умолчанию ядер − 2), `--out`, `--gnss-limit`,
  выбор представительных bag (`--t2s-bag`, `--s2t-bag`, `--speed-bag`, `--speed-t0`,
  `--cov-bag`), `--cache`.

---

## 8. FAQ

**Нода не публикует `/result/position`, только скорость.**
Поза появляется сразу после первого пригодного GNSS fix (предварительная выставка,
`init_provisional`). Если bag запущен не с начала (`--start-offset`) или GNSS нет вовсе,
через 10 с по header-времени после первого сообщения колёс включается счисление в системе
`odom` (x = пройденный путь). Состояние видно в диагностике: `mode`, `init_done`, `has_pose`.

**Сколько GNSS нужно?**
Для выставки хватает стартового окна: окончательная выставка занимает 3 с (`init_window_s`).
По прогонам с GNSS только в начале (медианы по 60 bag, эталон master; `results/<тег>/summary.md`):

| прогон | GNSS после первого fix | al_rmse, м: медиана / среднее | e3d, м | `p_in95`: медиана / среднее |
|---|---|---|---|---|
| `G_limit0` | одна эпоха | 2.15 / 3.17 | 1.77 | 0.993 / 0.919 |
| `G_limit05`, `G_limit1`, `AF_noaiding` | 0.5 с, 1 с; 10 с без коррекции | 2.15 / 3.12 | 1.74 | 0.993 / 0.929 |
| `G_limit3` | 3 с | 2.04 / 3.07 | 1.68 | 0.994 / 0.941 |
| `G_limit5` | 5 с | 1.96 / 3.15 | 1.66 | 0.995 / 0.948 |
| `F4_final` | 10 с | 1.98 / 3.15 | 1.66 | 0.995 / 0.947 |
| `G_limit30` | 30 с | 1.97 / 3.16 | 1.61 | 0.994 / 0.945 |
| `F4_final_burst` | 30 с + пачка 1.5 с каждые 120 с | 0.834 / 1.97 | 0.82 | 0.98 / 0.929 |

Скорость от GNSS не зависит: v_rmse 0.0295 м/с во всех строках. Без GNSS в середине
маршрута коррекция почти ничего не даёт. `F4_final` против `AF_noaiding` (оба с GNSS 10 с):
медиана al_rmse 1.98 против 2.15 м, среднее 3.15 против 3.12 м. al_rmse лучше в 35 из 60 bag,
хуже в 23, в 2 равно. Наибольшие проигрыши:

- 30639_9c362687: 12.98 против 8.84 м;
- 30639_92226df0: 17.56 против 15.88 м;
- 30618_27e994fc: 9.97 против 9.02 м.

9c362687 и 27e994fc входят в 7 bag со сдвигами часов GNSS на ±1 с
(`results/robustness/summary.md`, раздел 4). Причину проигрыша по каждому bag мы отдельно не
разбирали. Выигрыш коррекции проявляется, когда GNSS есть в середине маршрута:
`F4_final_burst` 0.834 м, а на тестовом bag организаторов 7.16 → 1.11 м (7.1).

**Используется ли GNSS после выставки?**
Да, по умолчанию и только для положения (`gnss_aiding: true`). Организаторы разрешили
использовать GNSS в середине маршрута для положения без штрафа. Как это устроено:

- Подписки на `/sensing/gnss/{master,rover}/fix` остаются на весь прогон.
  `ros2 topic info /sensing/gnss/master/fix -v` показывает подписчика `backup_odometry`, в
  диагностике `gnss_subscribed=True`.
- Фиксы после выставки идут только в якорь положения вдоль пути (5.3). В фильтр скорости
  они не попадают, и поток скорости побитово одинаков с коррекцией и без неё (тест
  `test_bursts_correct_along_track_drift_and_never_touch_speed`).
- С `gnss_aiding: false` поведение как в первой версии: подписки уничтожаются сразу после
  выставки, в логе строка `GNSS subscriptions destroyed`, `gnss_subscribed=False`. Параметр
  задаётся через `params_file` или `-p` (4.1); аргумент launch `gnss_aiding:=false`
  игнорируется.

Офлайн-приор из GNSS есть один: геометрия разворотных петель `maps/loops.json`. Её строит
`tools/build_loops.py` по RTK-трекам **обучающих** bag. Тестовый bag организаторов
используется только для проверки. Оговорка: в CV (`run_cv.py`) те же bag служат и
обучением петель, и оценкой, так что ошибки вне карты в `F4_final` занижены (утечка). На
тестовом bag, который в построение не входил, петли дают 23.09 → 1.11 м (7.1). Кластеры
остановок и другие приоры по GNSS не используются.

**Влияет ли `use_sim_time` или скорость воспроизведения на результат?**
Оценка идёт по header stamp входов, часы ноды в расчёте не участвуют. Поэтому
`use_sim_time` true и false дают одинаковый результат. Воспроизведение с `-r` > 1 тоже даёт
тот же результат, пока нода успевает: в среднем 0.4–0.7 мс на callback при 30 Гц (раздел 6.2).

**Что будет при сбоях входов?**
- Выбросы, заморозки, «залипший» ноль, пропуски одной или обеих тележек и
  рассогласование тележек (боксование, юз) обрабатываются в `wheel_fusion`.
- Скачки и задержки stamp обрабатываются в `preprocess.TimeGuard`.
- Без колёс скорость ведёт модель тяги и торможения.
- Исключения в callback локализованы: счётчик `errors`, уровень ERROR в диагностике,
  traceback в логе не чаще раза в 5 с. Нода продолжает работу.

**Где посмотреть логи, метрики и задержку?**
- Лог ноды: stdout `ros2 launch` или `node.log` у `run_bag.sh` и `run_checker.sh`. При
  завершении нода печатает сводку `shutdown: ...` с числом входов и выходов и перцентилями
  callback.
- Онлайн: `/backup_odometry/diagnostics`.
- Задержка и ресурсы: `scripts/measure.sh` (раздел 6) или `run_checker.sh --measure`.
- Метрика организаторов: `scripts/run_checker.sh` → `checker_final.txt`, офлайн
  `tools/checker_sim.py` (7.1).
- Точность по датасету: `tools/run_cv.py` и `tools/evaluate.py` (7.2–7.4).

**Почему `vehicle_id` важен и что если его не передать?**
По уточнению организаторов скрытая проверка идёт только на вагоне 30618, поэтому launch по
умолчанию передаёт `vehicle_id:=30618`. В `params.yaml` значение пустое: при запуске через
`ros2 run ... --params-file` передавайте `-p vehicle_id:=30618`.
`run_bag.sh` и `run_checker.sh` берут его из префикса имени bag, у `checker_sim.py` по
умолчанию стоит `--vehicle-id 30618`. Во входных топиках нет идентификатора машины. Откалиброванные масштабы колёс различаются:
K = 3.597 у 30618 и 3.59 у 30639, то есть датчики 30639 при той же скорости дают примерно на
0.2 % меньшее сырое значение. Без `vehicle_id` (или с неизвестным номером) используется
компромисс K = 3.595.
Сравнение `results/F3_final` (с `vehicle_id`) и `results/_deliv_novid` (`vehicle_id=unknown`),
медианы по 60 оценочным bag. Оба прогона сняты на версии F3, до петель и коррекции по GNSS.
Скорость от этих изменений не зависит, положение сейчас точнее:

| метрика | 30618 (49 bag): с id / без | 30639 (11 bag): с id / без |
|---|---|---|
| v_rmse, м/с | 0.0292 / 0.0294 | 0.0305 / 0.0328 |
| v_bias_trans, м/с | 0.0121 / 0.0131 | 0.0119 / 0.0191 |
| al_rmse, м (среднее) | 2.97 / 3.18 (3.68 / 3.74) | 3.81 / 5.42 (5.61 / 5.58) |
| drift_pct, % | 0.301 / 0.318 | 0.330 / 0.286 |

- **30618:** с `vehicle_id` лучше почти по всем метрикам: v_rmse лучше в 46 из 49 bag, drift
  в 48 из 49.
- **30639:** калиброванный K улучшает скорость (v_rmse лучше в 8 из 11 bag). В группе
  2026-08-26 (8 bag) средняя ошибка скорости −0.006 м/с против −0.013 м/с без него. По
  положению картина смешанная: медиана al_rmse заметно лучше с `vehicle_id` (3.81 против
  5.42 м), al_rmse лучше в 6 из 11 bag, среднее почти равно (5.61 против 5.58 м), а drift
  лучше с общим K в 10 из 11 bag. Калибровка 30639 опирается на 11 bag двух дней. В группе
  2026-05-05 (3 bag) скорость завышена на +0.02…+0.05 м/с при любом K.
- **Рекомендация:** передавать `vehicle_id:=30618`. На версии F3 общая медиана al_rmse по
  60 bag: 3.03 м с id и 3.19 м без, e3d 4.74 и 5.05 м, v_rmse 0.0295 и 0.0305 м/с.

---

## 9. ВОПРОСЫ К ЖЮРИ

В первой версии было шесть открытых вопросов о правилах оценки. Организаторы на них ответили
и дали чекер с тестовым bag. Ниже для каждого вопроса приведены ответ, что сделано в пакете и
как это проверено. В конце раздела перечислены вопросы, которые остаются открытыми. Каждое
соглашение по-прежнему переключается параметром без пересборки (4.1). Оговорка: всё, кроме
`output_frame`, `vehicle_id` и `maps_dir`, задаётся через `params_file` или `-p`, а не
аргументом launch.

| вопрос | ответ (организаторы / код чекера) | по умолчанию сейчас | как вернуть или переключить |
|---|---|---|---|
| точка эталона | `base_link` на pathgraph, 9.873 м впереди master-антенны | `output_along_offset_m` = 9.873 | 0 (master), 6.22 (середина базы), 12.44 (rover) |
| z | ошибка 3D, z = z pathgraph | `output_z_offset_m` = 0.0 | 3.10 = высота master-антенны |
| система координат | эталон в `map` | `output_frame: map` | `utm` / `enu` (+ `enu_origin_*`) / `start` |
| участки вне карты | чекер сравнивает все сопоставленные точки, без ворот по карте | петли `maps/loops.json`, дальше хвост; ковариация растёт | `loops_enable: false` (мост и хвост первой версии) |
| сопоставление по времени | header stamp, `ApproximateTimeSynchronizer`, slop 0.05 с, очередь 100 | stamp выхода = stamp входа; скорость на момент stamp − 0.09 с | `output_velocity_delay_s`, `publish_on: cmd` |
| номер вагона | скрытая проверка только на 30618 | launch по умолчанию `vehicle_id` = 30618 (K = 3.597); в `params.yaml` пусто, поэтому при `ros2 run` или своём `params_file` передайте `-p vehicle_id:=30618` | `vehicle_id:=30618` / `30639` |
| GNSS в середине маршрута | можно для положения, не для скорости | `gnss_aiding: true` | `gnss_aiding: false` |
| `tram_vehicle_msgs` | берутся из датасета | `src/tram_vehicle_msgs` совпадает с пакетом датасета | — |

1. **Эталонная точка. Отвечено: `base_link` на pathgraph.**
   - Сделано: по умолчанию публикуется точка на 9.873 м впереди master-антенны по дуге пути
     (`output_along_offset_m`).
   - Проверка: на тестовом bag 3D RMSE 1.11 м против 10.26 м у прежней точки master-антенны
     (`results/checker_30618_88aea4d9/summary.md`, вариант `master_antenna_point`).
   - Для CV по датасету эталон — фиксы master, поэтому `run_cv.py` публикует точку master
     (7.3). Там e3d 1.66 м против master, 12.2 м против rover и 5.7 м против середины базы
     (`results/F4_final`, `--antenna all`).
2. **z и 3D. Отвечено: ошибка 3D, z эталона = z pathgraph.**
   - Сделано: `output_z_offset_m` = 0.0, z RMSE на тестовом bag 0.04 м (при z антенны
     3.07 м).
   - Значение `z_offset_m` = 3.10 м нода по-прежнему использует внутри при выставке: по нему
     высота GNSS переводится в z старта и моста.
3. **Участки вне карты. Отвечено кодом чекера: сопоставляются все точки, отдельного учёта нет.**
   - Каждый прогон начинается на конечной перед разворотной петлёй. По уточнению организаторов
     RTK в окне старта гарантирован. Значит, первые сотни метров и хвост после конца карты
     входят в метрику с тем же весом.
   - Сделано: геометрия петель восстановлена по обучающим заездам (`maps/loops.json`, 5.3),
     выставка идёт прямо на ветку петли.
   - На тестовом bag без петель 3D RMSE 23.09 м, с петлями 1.11 м (`no_loops` против
     `final`).
   - В CV средняя e2d вне карты на bag: медиана 1.33 м, p90 4.87 м, максимум 23.0 м
     (`results/F4_final`). В первой версии было 11.2 / 24.6 / 104 м. Цифры CV занижены
     утечкой: петли построены по тем же bag (раздел 8).
   - Прежнее замечание про параллельный путь в 4.2 м справа от pathgraph (часть S2T при
     s ≈ 530–1140 м) касается GNSS-эталона CV. Эталон чекера по словам организаторов лежит на
     pathgraph, и там, как и наш выход, остаётся на линии карты.
4. **Сопоставление по времени. Отвечено кодом чекера: header stamp, slop 0.05 с.**
   - Stamp выхода равен header stamp входа (5.2), поэтому время записи и порядок прихода на
     сопоставление почти не влияют.
   - Немонотонные stamp допустимы: у каждого синхронизатора своя очередь. В эмуляции
     сопоставлены 38 439 из 38 476 выходов.
   - Эталонная скорость `twist.linear.x` отстаёт от своего stamp. Поэтому скорость
     публикуется на момент stamp − `output_velocity_delay_s` (0.09 с): v RMSE 0.0566 →
     0.0335 м/с на тестовом bag.
5. **Номер вагона. Отвечено: только 30618.**
   - K = 3.597 выбирается при `vehicle_id:=30618`; это умолчание launch-файла. В `core/config.py`
     и `params.yaml` значение пустое (K = 3.595), поэтому при `ros2 run … --params-file`
     передавайте `-p vehicle_id:=30618`; `run_bag.sh` и `run_checker.sh` подставляют его сами.
   - Цифры с `vehicle_id` и без него (версия F3) приведены в разделе 8.
6. **Система координат. Отвечено: эталон в `map`.**
   - По умолчанию `output_frame: map`. Это система карт pathgraph: UTM 37N минус (300000,
     6100000).
   - `utm`, `start` и `enu` оставлены для других потребителей (5.3). Оси ENU повёрнуты
     относительно `map` на сближение меридианов (около 1.25–1.27°), поэтому путать `start` и
     `enu` нельзя.
7. **GNSS в середине маршрута. Отвечено: можно для положения.**
   - Сделано: коррекция положения вдоль пути по пачкам фиксов (5.3), скорость от неё не
     зависит.
   - На тестовом bag коррекция снижает 3D RMSE с 7.16 до 1.11 м.
8. **`tram_vehicle_msgs`. Отвечено: из датасета.** Наш `src/tram_vehicle_msgs` совпадает с
   `../dataset/tram_vehicle_msgs` (`diff -r` пуст), и чекер собирается против него (7.1).

**Что остаётся открытым:**

- **Будет ли GNSS в середине маршрута в скрытых прогонах, и как часто?** От этого зависит
  большая часть ошибки положения. На тестовом bag с GNSS на всём маршруте 3D RMSE 1.11 м, с
  GNSS только в первые 35 с 7.02 м, без коррекции 7.16 м
  (`results/checker_30618_88aea4d9/summary.md`). По CV коррекция с GNSS только на старте
  почти ничего не даёт (раздел 8).
- **Задержка эталонной скорости одинакова во всех скрытых bag?** `output_velocity_delay_s`
  = 0.09 с подобрано по одному тестовому bag. Если эталонная скорость строится иначе,
  задержку нужно сменить или обнулить. Положение от неё не зависит.
- **Какой веткой петли у Таллинской (T) идёт трамвай в конце скрытых прогонов?** Без GNSS в
  хвосте мы берём ветку A. На тестовом bag последняя минута записи проходит по хвосту: 3D
  RMSE 3.7 м с GNSS и 30.2 м без него. Разворот ветки T/B ни разу не записан, вместо него
  стоит заглушка R 18 м (5.3, `tools/build_loops.py`).
- **Допустима ли геометрия петель по GNSS обучающих заездов?** Это офлайн-приор по GNSS
  обучающих данных, скрытые bag в нём не участвуют. `loops_enable: false` его выключает.
