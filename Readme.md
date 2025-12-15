# Week 13–14: Launch file (Python) – turtlesim mimic

Selle nädala eesmärk on koostada ROS 2 **launch file**, mis käivitab korraga mitu node’i ning seadistab namespace’id, argumendid ja topic remap’id.

## Õpiväljundid
- lood `launch/` kataloogi ja kirjutad Python launch faili
- käivitad süsteemi käsuga `ros2 launch`
- mõistad, milleks on `namespace`, `arguments` vs `ros_arguments` ning `remappings`
- testid, et **mimic** paneb `turtlesim2` jäljendama `turtlesim1` liikumist

## Eeldused
- ROS 2 Humble keskkond töötab (Docker/Devcontainer).
- `turtlesim` on installitud (enamasti ROS 2 desktop installiga kaasas).
- GUI aknad peavad saama avaneda (turtlesim on graafiline).

---

## Ülesanne A: loo launch kataloog (kohustuslik)

Vali asukoht, kuhu sa selle ülesande failid lisad (soovitus: sinu repo sees, näiteks `ros2_ws/src/<mingi_pakett>/` või otse repo juures).

Loo `launch/` kataloog:
```bash
mkdir -p launch
```

---

## Ülesanne B: kirjuta launch fail (kohustuslik)

Loo fail:
```
launch/turtlesim_mimic_launch.py
```

Kopeeri sisse **täpselt** järgmine kood:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim',
            ros_arguments=['--log-level', 'warn']
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

---

## Ülesanne C: käivita launch fail (kohustuslik)

Mine `launch/` kataloogi ja käivita launch:

```bash
cd launch
ros2 launch turtlesim_mimic_launch.py
```

Kui kõik on korras, näed logis midagi sellist:
- launch logging INFO
- 2× `turtlesim_node` protsessi
- 1× `mimic` protsess

---

## Ülesanne D: pane turtlesim1 liikuma ja kontrolli “mimic” tööd (kohustuslik)

Ava **uus terminal** (jäta launch terminal tööle) ja käivita:

```bash
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```

Ootus:
- `turtlesim1` hakkab liikuma
- `turtlesim2` peaks tegema **sama trajektoori** (mimic töötab)

Peatamiseks:
- `Ctrl + C` pub terminalis
- `Ctrl + C` launch terminalis

---

## Ülesanne E: introspektsioon rqt_graph abil (soovituslik, +lisapunktid)

Kui süsteem töötab, ava uus terminal ja käivita:

```bash
ros2 run rqt_graph rqt_graph
```

Tee ekraanipilt või kirjuta 2–4 lauset:
- milline node publishib `/turtlesim1/turtle1/cmd_vel`
- kuhu `mimic` subscribe’b ja kuhu publishib

---

## Märkus (paketis kasutamine)
Launch faili saab käivitada ka paketist:
```bash
ros2 launch <package_name> <launch_file_name>
```

Kui sa paned launch faili oma ROS paketisse, on hea lisada `package.xml` faili:
```xml
<exec_depend>ros2launch</exec_depend>
```

---

## Esitamine (kohustuslik)
Sinu GitHub repo peab sisaldama:
- `launch/turtlesim_mimic_launch.py`
- README “Tulemused” sektsioon täidetud

Git:
```bash
git status
git add -A
git commit -m "Week 13-14: turtlesim mimic launch file"
git push
```
