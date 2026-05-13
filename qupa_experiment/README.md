# qupa_experiment

Implementación ROS 2 del experimento de asignación distribuida de tareas
basado en Brutschy et al. (2012), con aprendizaje social modulado por
vecinos detectados vía cámara.

Cada robot ejecuta una copia de `experiment_node` bajo su propio namespace.
La lógica es idéntica para todos los robots; lo único que cambia es el
namespace, que separa los topics y servicios por robot.

---

## Pre-requisitos

Antes de lanzar el experimento, `hardware.launch.py` debe estar corriendo
en cada robot físico (provee `scan`, `floor/color`, `cmd_vel`, `leds/set`,
`uv_led/set`):

```bash
# En cada robot
ros2 launch qupa_hardware hardware.launch.py namespace:=qupa_3A
```

---

## Lanzar un solo robot

```bash
# Default namespace (qupa_C0)
ros2 launch qupa_experiment experiment.launch.py

# Namespace explícito
ros2 launch qupa_experiment experiment.launch.py namespace:=qupa_3B
```

El nodo se publicará bajo `/qupa_3B/experiment_node` y consumirá los topics
`/qupa_3B/scan`, `/qupa_3B/floor/color`, `/qupa_3B/camera/detections`, etc.

---

## Lanzar varios robots a la vez

`experiment_multi.launch.py` toma una lista de namespaces separados por
coma y arranca una instancia del mismo nodo para cada uno. Útil cuando se
quiere coordinar el experimento desde una sola PC.

```bash
# Default: qupa_3A,qupa_3B
ros2 launch qupa_experiment experiment_multi.launch.py

# Lista explícita
ros2 launch qupa_experiment experiment_multi.launch.py \
    namespaces:=qupa_3A,qupa_3B,qupa_F4
```

> **Nota:** cada robot debe tener su propio hardware corriendo (cada Pi
> ejecuta `hardware.launch.py` con su namespace). Este launch solo levanta
> los nodos de comportamiento, que se conectan a los topics del namespace
> correspondiente vía DDS. Asegúrate de que **todas las máquinas
> compartan el mismo `ROS_DOMAIN_ID`** para que se descubran mutuamente.

---

## Variante clásica

`experiment_classic.launch.py` lanza la versión sin aprendizaje social
(`experiment_node_classic.py`), reproduciendo el modelo Brutschy original
+1/−1 con sigmoide simple.

```bash
ros2 launch qupa_experiment experiment_classic.launch.py namespace:=qupa_F4
```

---

## Configuración

Los parámetros del modelo están en
[config/experiment.yaml](config/experiment.yaml).
Los launch files cargan ese YAML y sobrescriben solo los parámetros de
kinemática del robot. Para cambiar el comportamiento (timings, sigmoid,
aprendizaje social), edita el YAML y reconstruye:

```bash
colcon build --packages-select qupa_experiment
```

---

## Topics y servicios

| Tipo          | Nombre (relativo al namespace)         | Dirección |
|---------------|-----------------------------------------|-----------|
| Subscriber    | `scan` (`sensor_msgs/LaserScan`)        | in        |
| Subscriber    | `floor/color` (`std_msgs/String`)       | in        |
| Subscriber    | `camera/detections` (`std_msgs/String`) | in        |
| Publisher     | `cmd_vel` (`geometry_msgs/Twist`)       | out       |
| Service client| `leds/set` (`qupa_msgs/LEDCommand`)     | out       |

---

## Diagnóstico rápido

```bash
# ¿Está vivo el nodo?
ros2 node list | grep experiment

# ¿Está publicando velocidades?
ros2 topic echo /qupa_3A/cmd_vel

# ¿Llegan datos del piso?
ros2 topic echo /qupa_3A/floor/color

# ¿Tiene suscriptor el cmd_vel? (debe ser ≥ 1, el motor_driver)
ros2 topic info /qupa_3A/cmd_vel
```
