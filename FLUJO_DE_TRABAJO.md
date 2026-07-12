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
```bash
ros2 launch capytown_granprix granprix_bringup.launch.py ronda:=1 usar_camara:=true
```
> Ver `README.md` (raíz del repo) para la lista completa de argumentos
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
- Workspace en el robot: `/root/yahboomcar_ws`. Carpeta del repo dentro de
  `src/`: **`reto-final`**.
- Solo se sube a GitHub el código de los paquetes ROS2. Archivos de
  trabajo del PC (notas, imágenes, informes) pueden quedarse solo
  local si no hace falta que el robot los vea.
- Después de cada commit, hacer `git push` directo sin pedir confirmación
  — así se trabajó en RC3.
```
ros2 launch capytown_granprix granprix_bringup.launch.py usar_camara:=false
```