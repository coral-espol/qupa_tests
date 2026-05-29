# Qupa Experiment

Implementación ROS 2 del experimento de asignación distribuida de tareas basado en Brutschy et al. (2012), con aprendizaje social modulado por vecinos detectados vía cámara omnidireccional.

Los robots arrancan inmóviles esperando la señal del timer. Al recibirla, comienzan a explorar y registrar datos. Cuando el tiempo configurado expira, los nodos se cierran solos.

---

## Contenido

- [Pre Requisitos](#pre-requisitos)
- [Build](#build)
- [Flujo Del Experimento](#flujo-del-experimento)
- [Salida De Datos](#salida-de-datos)
- [Configuración](#configuración)
- [Topics Y Servicios](#topics-y-servicios)
- [Estados Del Robot](#estados-del-robot)
- [Diagnóstico Rápido](#diagnóstico-rápido)
- [Mantenedor](#mantenedor)

---

## Pre Requisitos

`hardware.launch.py` y `camera.launch.py` deben estar corriendo en cada robot físico antes de lanzar el experimento:

```bash
ros2 launch qupa_hardware hardware.launch.py
ros2 launch qupa_hardware camera.launch.py
```
Considera el uso de `tmux` para correr ambos nodos.  
Todas las máquinas deben compartir el mismo `ROS_DOMAIN_ID` para descubrirse mutuamente vía DDS. Adicional a esto revisen los logs en las consolas para verificar si esta correcto 

---

## Build

```bash
cd ~/experiment_ws
colcon build --packages-select qupa_experiment
source install/setup.bash
```

---

## Flujo Del Experimento

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

## Salida De Datos

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

## Configuración

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

## Topics Y Servicios

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

## Estados Del Robot

| Estado    | LEDs                        | Comportamiento                        |
|-----------|-----------------------------|---------------------------------------|
| WAITING   | Blanco                      | Parado, esperando señal de inicio     |
| EXPLORE   | Naranja (parpadeo patrol)   | Navegando, buscando parches           |
| EXECUTE   | Azul (TYPE_A) / Verde (TYPE_B) | Ejecutando tarea, quieto           |
| EXIT      | Apagado                     | Saliendo del parche                   |

---

## Diagnóstico Rápido

```bash
# Estado del experimento
ros2 topic echo /experiment/running

# Logs de un robot (busca [JOB], [SKIP], [DONE])
ros2 topic echo /q0/cmd_vel

# Datos del sensor de piso
ros2 topic echo /q0/floor/color

# Nodos activos
ros2 node list | grep experiment
```

---

## Mantenedor

**David Torres**  
Ing. Mecatrónico — ESPOL  
Técnico de Laboratorio, Collaborative Robotics and Artificial Intelligence Laboratory (CoRAL)  
davatorr@espol.edu.ec