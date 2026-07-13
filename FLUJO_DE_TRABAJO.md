# Flujo de trabajo — Reto Final

Mismo flujo que se usó en RC3: se edita en el PC, se sube a GitHub, y en
el robot se descarga y se compila ahí.

---

## Resumen

```
PC (VSCode)  →  git commit + push  →  GitHub  →  robot: git pull + colcon build
```

Los parámetros y el código **siempre se editan en el PC**, nunca directo
en el robot.

---

## 0. Este repo — simulador de pista (nuevo, independiente del robot)

Esta carpeta (`simulador_pista`) es su **propio repo**, separado del repo
del reto en el robot (`Reto-Final-ROBOTICA-Yahboom-ROSMASTER-`, que ya no
se toca — ver `robot/README.md`). Contiene dos cosas:

- El simulador en Python puro (`main.py`, `sim/`) — corre en el PC, sin ROS2.
- `robot/granprix_bot/` — paquete ROS2 (`ament_python`) que implementa la
  misma lógica para correr de verdad en el robot. Este **sí** se compila y
  lanza en el robot (`colcon build`, `ros2 launch`) — ver `robot/README.md`
  para el detalle completo (calibración, cómo avanza celda por celda,
  comandos exactos).

```
https://github.com/frayderMM/mouse-runner-robotica
```

Ya hecho (primera vez):
```bash
git init
git add -A
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/frayderMM/mouse-runner-robotica.git
git push -u origin main
```

Ciclo normal para este repo:
```bash
python main.py pistas/pista_ejemplo.txt   # probar localmente
git add .
git commit -m "mensaje del cambio"
git push origin main
```
> `__pycache__/` y `*.pyc` están en `.gitignore`, no se suben. Las imágenes
> generadas en `pistas/` (`.png`) sí se suben porque son parte de los
> resultados/ejemplos del simulador.

### Compilar y correr `robot/granprix_bot` en el robot

```bash
# En el robot, dentro del contenedor Docker (ver Paso 3 de la sección 2):
cd /root/yahboomcar_ws/src
git clone https://github.com/frayderMM/mouse-runner-robotica.git simulador-pista   # primera vez
# o, si ya está clonado:
cd simulador-pista && git fetch origin && git reset --hard origin/main && cd ..

ln -sf /root/yahboomcar_ws/src/simulador-pista/robot/granprix_bot /root/yahboomcar_ws/src/granprix_bot
cd /root/yahboomcar_ws
colcon build --packages-select granprix_bot
source install/setup.bash

ros2 launch granprix_bot explorar.launch.py    # Ronda 1
ros2 launch granprix_bot speedrun.launch.py    # Ronda 2 (despues de la 1)
```

---

## 1. Primera vez — crear el repo (ya hecho)

**En el PC**, dentro de la carpeta `Reto Final` (ya ejecutado):
```bash
echo "# Reto-Final-ROBOTICA-Yahboom-ROSMASTER-" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/frayderMM/Reto-Final-ROBOTICA-Yahboom-ROSMASTER-.git
git push -u origin main
```
> Repo real: `https://github.com/frayderMM/Reto-Final-ROBOTICA-Yahboom-ROSMASTER-`.
> Solo se subió `README.md` en el primer commit — el resto de los `.md` de
> notas se queda local por ahora (mismo criterio que en RC3); los paquetes
> ROS2 del reto se agregan y suben más adelante cuando existan.

**En el robot, primera vez (clonar):**
```bash
cd /root/yahboomcar_ws/src
git clone https://github.com/frayderMM/Reto-Final-ROBOTICA-Yahboom-ROSMASTER-.git reto-final
cd /root/yahboomcar_ws
colcon build --packages-select capytown_interfaces capytown_granprix
source install/setup.bash
```
> La carpeta en el robot se llama **`reto-final`** (ya no `capytown-reto33`,
> ese nombre era de RC3). Los paquetes del reto son **`capytown_interfaces`**
> (mensajes) y **`capytown_granprix`** (nodos, launch, config) — compilar
> `capytown_interfaces` primero o listar ambos juntos como arriba.

---

## 2. Ciclo normal de trabajo (todas las veces siguientes)

### Paso 1 — Editar en el PC
Cambios de código o de parámetros (`.yaml`) en VSCode.

### Paso 2 — Commit y push (PC)
```bash
git add .
git commit -m "mensaje del cambio"
git push origin main
```

### Paso 3 — Entrar al robot y al contenedor Docker
```bash
ssh root@10.42.0.1
docker start quizzical_ellis
docker exec -it -e DISPLAY=:0 quizzical_ellis /bin/bash
```
> `-e DISPLAY=:0` es necesario para que las ventanas gráficas (matplotlib
> de `lidar_viz.py`, RViz, etc.) se muestren en el VNC del robot.

### Paso 4 — Pull y build (dentro del contenedor)

Para `granprix_bot` (este repo, `simulador_pista` — lo que se usa ahora):
```bash
cd /root/yahboomcar_ws/src/simulador-pista && git fetch origin && git reset --hard origin/main
cd /root/yahboomcar_ws && colcon build --packages-select granprix_bot && source install/setup.bash
```

Para el repo original `reto-final` (`capytown_interfaces`/`capytown_granprix`,
ya no se toca — ver sección 0):
```bash
cd /root/yahboomcar_ws/src/reto-final && git fetch origin && git reset --hard origin/main
cd /root/yahboomcar_ws && colcon build --packages-select capytown_interfaces capytown_granprix && source install/setup.bash
```

### Paso 5 — Verificar tópicos del robot (antes de lanzar)
```bash
ros2 topic list
ros2 topic info /scan
ros2 topic info /odom_raw
```

### Paso 6 — Lanzar y probar en pista

Para `granprix_bot`:
```bash
ros2 launch granprix_bot explorar.launch.py    # Ronda 1
ros2 launch granprix_bot speedrun.launch.py    # Ronda 2 (despues de la 1)
```
> Ver `robot/README.md` para la lista completa de argumentos
> (`params_file`, `usar_dashboard`) y la guía de calibración por nodo.

Para el repo original `reto-final`:
```bash
ros2 launch capytown_granprix granprix_bringup.launch.py ronda:=1 usar_camara:=true
```
> Ver `README.md` (raíz de ese repo) para la lista completa de argumentos
> (`ronda`, `usar_camara`, `params_file`) y la guía de calibración por
> nodo (sección 5).

### Paso 7 — Si hay que ajustar algo
Volver al Paso 1. Nunca editar el YAML directo en el robot: siempre en el
PC, commit, push, y repetir el pull en el robot.

---

## Notas

- El robot se conecta por **TigerVNC** (tiene entorno gráfico, no solo
  terminal) — útil para ver matplotlib, RViz, etc. directamente ahí.
- IP del robot: `10.42.0.1`, usuario `root`.
- Contenedor Docker: **`quizzical_ellis`** (no `friendly_pike`, ese nombre
  quedó obsoleto de una etapa anterior del proyecto).
- Workspace en el robot: `/root/yahboomcar_ws`. Carpetas del repo dentro de
  `src/`: **`simulador-pista`** (este repo, `granprix_bot` — activo) y
  **`reto-final`** (repo original, ya no se toca).
- Solo se sube a GitHub el código de los paquetes ROS2. Archivos de
  trabajo del PC (notas, imágenes, informes) pueden quedarse solo
  local si no hace falta que el robot los vea.
- Después de cada commit, hacer `git push` directo sin pedir confirmación
  — así se trabajó en RC3.

```
ros2 launch capytown_granprix granprix_bringup.launch.py usar_camara:=false
```
