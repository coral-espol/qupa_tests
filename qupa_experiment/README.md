# Qupa Experiment

Implementación ROS 2 del experimento de asignación distribuida de tareas basado en Brutschy et al. (2012), con aprendizaje social modulado por vecinos detectados vía cámara omnidireccional.

Los robots arrancan inmóviles esperando la señal del timer. Al recibirla, comienzan a explorar y registrar datos. Cuando el tiempo configurado expira, los nodos se cierran solos.

---

## Contenido

- [0. Pre Requisitos](#0-pre-requisitos)
- [1. Instalación](#1-instalación)
- [2. Compilación](#2-compilación)
- [3. Flujo Del Experimento](#3-flujo-del-experimento)
- [4. Salida De Datos](#4-salida-de-datos)
- [5. Configuración](#5-configuración)
- [6. Topics Y Servicios](#6-topics-y-servicios)
- [7. Estados Del Robot](#7-estados-del-robot)
- [8. Diagnóstico Rápido](#8-diagnóstico-rápido)
- [Mantenedor](#mantenedor)

---

## 0. Pre Requisitos

- ROS 2 Humble instalado
- Workspace de ROS 2 en `~/qupa_ws`
- `hardware.launch.py` y `camera.launch.py` corriendo en cada robot físico antes de lanzar el experimento:

```bash
ros2 launch qupa_hardware hardware.launch.py
ros2 launch qupa_hardware camera.launch.py
```

Considera el uso de `tmux` para correr ambos nodos. Todas las máquinas deben compartir el mismo `ROS_DOMAIN_ID` para descubrirse mutuamente vía DDS. Revisa los logs en consola para verificar que la conexión es correcta.

---

## 1. Instalación

Clona los dos repositorios necesarios dentro del workspace:

```bash
mkdir -p ~/qupa_ws/src && cd ~/qupa_ws/src

git clone https://github.com/coral-espol/qupa.git
git clone https://github.com/coral-espol/qupa_tests.git
```

---

## 2. Compilación

```bash
cd ~/qupa_ws
colcon build --packages-select qupa_experiment
source install/setup.bash
```

---

## 3. Flujo Del Experimento

### Paso 1 — Nodos De Comportamiento

Los robots arrancan en estado WAITING (LEDs blancos, sin moverse):

```bash
ros2 launch qupa_experiment experiment_multi.launch.py \
    namespaces:=q0,q1,q2,q3 \
    data_log_dir:=/home/estudiante/datos/experimento_1
```

### Paso 2 — Timer

```bash
ros2 launch qupa_experiment timer.launch.py duration_s:=600.0
```

### Paso 3 — Iniciar

```bash
ros2 service call /experiment/start std_srvs/srv/Trigger
```

Todos los robots arrancan simultáneamente y comienzan a explorar. Al llegar a `duration_s` segundos, los nodos se cierran solos.

### Paso 4 — Detener Antes De Tiempo

```bash
ros2 service call /experiment/stop std_srvs/srv/Trigger
```

---

## 4. Salida De Datos

Cada robot genera un CSV en `data_log_dir/<namespace>.csv`:

```
tick,greedy,robot,m,p_x,planned_wticks,task,search_ticks,x,y,seed
53.241,false,q0,0.000000,0.500000,60.000,TYPE_A,3.120,0,0,0
127.883,false,q0,1.000000,0.731059,57.300,TYPE_A,5.440,0,0,0
```

| Columna          | Descripción                                               |
|------------------|-----------------------------------------------------------|
| `tick`           | Segundos desde el inicio del experimento                  |
| `greedy`         | `true` si el robot siempre acepta, omitiendo la sigmoide  |
| `robot`          | Namespace del robot                                       |
| `m`              | Memoria de especialización signed (−m_max … +m_max)       |
| `p_x`            | Probabilidad de aceptación al momento de la decisión      |
| `planned_wticks` | Tiempo de servicio asignado a la tarea (segundos)         |
| `task`           | Tipo de tarea aceptada (`TYPE_A` / `TYPE_B`)              |
| `search_ticks`   | Duración de la búsqueda hasta encontrar esta tarea (s)    |
| `x`, `y`         | No disponible en esta configuración (fijo `0`)            |
| `seed`           | No usada en esta configuración (fijo `0`)                 |

---

## 5. Configuración

| Archivo                  | Contenido                                      |
|--------------------------|------------------------------------------------|
| `config/experiment.yaml` | Modelo de especialización, tiempos, navegación |
| `config/timer.yaml`      | Duración del experimento (`duration_s`)        |

### Parámetros Clave

```yaml
# config/experiment.yaml
refractory_s: 3.0          # tiempo muerto entre tareas (segundos)
task_timing:
  base_work_s:  60.0       # tiempo de servicio base
  min_work_s:    8.0       # tiempo mínimo tras especialización completa
specialization:
  m_max: 12                # límite del contador de especialización
greedy_mode: false         # true = siempre acepta, omite la sigmoide
```

```yaml
# config/timer.yaml
duration_s: 600.0          # duración total del experimento
```

Tras editar cualquier YAML, reconstruye con `colcon build --packages-select qupa_experiment`.

---

## 6. Topics Y Servicios

### Por Robot

| Tipo           | Nombre                                           | Dirección |
|----------------|--------------------------------------------------|-----------|
| Subscriber     | `scan` (`sensor_msgs/LaserScan`)                 | entrada   |
| Subscriber     | `floor/color` (`std_msgs/String`)                | entrada   |
| Subscriber     | `camera/detections` (`qupa_msgs/DetectionArray`) | entrada   |
| Publisher      | `cmd_vel` (`geometry_msgs/Twist`)                | salida    |
| Service client | `leds/set` (`qupa_msgs/LEDCommand`)              | salida    |

### Globales

| Tipo      | Nombre                                    | Descripción                          |
|-----------|-------------------------------------------|--------------------------------------|
| Publisher | `/experiment/running` (`std_msgs/Bool`)   | `true` mientras el experimento corre |
| Service   | `/experiment/start` (`std_srvs/Trigger`)  | Inicia el temporizador               |
| Service   | `/experiment/stop` (`std_srvs/Trigger`)   | Detiene el experimento               |

---

## 7. Estados Del Robot

| Estado  | LEDs                              | Comportamiento                    |
|---------|-----------------------------------|-----------------------------------|
| WAITING | Blanco                            | Parado, esperando señal de inicio |
| EXPLORE | Naranja (parpadeo patrol)         | Navegando, buscando parches       |
| EXECUTE | Azul (TYPE_A) / Verde (TYPE_B)    | Ejecutando tarea, quieto          |
| EXIT    | Apagado                           | Saliendo del parche               |

---

## 8. Diagnóstico Rápido

```bash
# Estado del experimento
ros2 topic echo /experiment/running

# Datos del sensor de piso de un robot
ros2 topic echo /q0/floor/color

# Velocidades publicadas por un robot
ros2 topic echo /q0/cmd_vel

# Nodos activos
ros2 node list | grep experiment
```

---

## Mantenedor

**David Torres**  
Ing. Mecatrónico — ESPOL  
Técnico de Laboratorio, Collaborative Robotics and Artificial Intelligence Laboratory (CoRAL)  
davatorr@espol.edu.ec
