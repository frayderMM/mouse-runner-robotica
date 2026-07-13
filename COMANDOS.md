# Comandos — granprix_bot en el robot

Chuleta rápida de copiar/pegar. Para el detalle de cada cosa ver
`FLUJO_DE_TRABAJO.md` (raíz del repo) y `robot/README.md`.

---

## 1. Conectarse al robot

```bash
ssh root@10.42.0.1
```

## 2. Entrar al contenedor Docker

```bash
docker start quizzical_ellis
docker exec -it -e DISPLAY=:0 quizzical_ellis /bin/bash
```
> `-e DISPLAY=:0` solo hace falta si vas a abrir algo gráfico (RViz,
> matplotlib, etc.) en la VNC del robot.

## 3. Traer los cambios (git pull) dentro del contenedor

```bash
cd /root/yahboomcar_ws/src/simulador-pista
git fetch origin && git reset --hard origin/main
```
> Primera vez (todavía no clonado):
> ```bash
> cd /root/yahboomcar_ws/src
> git clone https://github.com/frayderMM/mouse-runner-robotica.git simulador-pista
> ln -s /root/yahboomcar_ws/src/simulador-pista/robot/granprix_bot /root/yahboomcar_ws/src/granprix_bot
> ```

## 4. Compilar

```bash
cd /root/yahboomcar_ws/src/simulador-pista
git fetch origin && git reset --hard origin/main

cd /root/yahboomcar_ws
colcon build --packages-select granprix_bot
source install/setup.bash
```

## 5. Verificar tópicos del robot (antes de lanzar)

```bash
ros2 topic list
ros2 topic info /scan
ros2 topic info /odom_raw
```

## 6. Lanzar

**Ronda 1 — Exploración:**
```bash
ros2 launch granprix_bot explorar.launch.py
```

**Ronda 2 — Speed run** (después de correr la Ronda 1 al menos una vez):
```bash
ros2 launch granprix_bot speedrun.launch.py
```

Ctrl+C para terminar en cualquier momento (frena el robot y cierra el nodo).

## 7. Ver en vivo mientras corre (otra terminal / otro `docker exec`)

```bash
ros2 topic echo /robot_event    # cada decision (SENSADO, DECISION, GIRO_FIN, ...)
ros2 topic echo /robot_state    # estado actual de la maquina de estados
ros2 topic echo /cmd_vel
ros2 topic echo /odom_raw
```

## 8. Ver los logs de la corrida (después de Ctrl+C)

Dentro del contenedor:
```bash
cat ~/capytown_resultados/eventos_*.csv | column -s, -t   # el mas reciente, formateado como tabla
cat ~/capytown_resultados/mapa_descubierto.yaml           # despues de la Ronda 1
```

Sacar los logs del contenedor al escritorio del robot (para abrirlos
con algo gráfico, ej. LibreOffice Calc, desde la VNC):
```bash
docker cp quizzical_ellis:/root/capytown_resultados ~/Desktop/capytown_resultados
```

## 9. Recalibrar (editar en el PC, no en el robot)

Cualquier ajuste de parámetros va en
`robot/granprix_bot/config/granprix_bot_params.yaml`, se edita en el
PC, se commitea/pushea, y se repiten los pasos 3-4 en el robot. Nunca
editar el YAML directo ahí adentro.
