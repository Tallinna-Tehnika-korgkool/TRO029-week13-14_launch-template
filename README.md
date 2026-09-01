# Nädal 13–14: Launch-failid (Python) – `py_pubsub` käivitamine

Selle nädala eesmärk on koostada ROS 2 **launch-fail**, mis käivitab
korraga mitu sinu enda kirjutatud node'i. Kasutame selleks eelmises
harjutuses loodud `py_pubsub` paketi publisheri (`talker`) ja
subscriberi (`listener`) noode.

## Õpiväljundid
- Lood uue ROS 2 paketi launch-failide hoidmiseks.
- Kirjutad Pythonis launch-faili, mis käivitab kaks node'i (`talker` ja `listener`).
- Käivitad kogu süsteemi ühe käsuga: `ros2 launch`.
- Mõistad, kuidas lisada olemasoleva paketi node'e launch-faili.

## Eeldused
- ROS 2 Humble keskkond töötab (vt nädal 01–02).
- Sinu workspace'is (`/workspace/ros2_ws/src`) on olemas `py_pubsub`
  pakett (nädal 09–10).

---

## Ülesanne A: Loo uus launch-pakett (kohustuslik)

```bash
cd /workspace/ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 my_launch_pkg
cd my_launch_pkg
mkdir launch
```

## Ülesanne B: Kirjuta launch-fail (kohustuslik)

Loo `launch` kausta fail `pubsub_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_pubsub',      # Pakett, kust node pärineb
            executable='talker',      # Node'i käivitatav nimi (setup.py-st)
            name='my_talker'          # Node'ile antav unikaalne nimi
        ),
        Node(
            package='py_pubsub',
            executable='listener',
            name='my_listener'
        )
    ])
```

## Ülesanne C: Tee launch-fail paketile nähtavaks (kohustuslik)

Selleks, et `ros2 launch` käsk leiaks `pubsub_launch.py` faili üles,
tuleb `setup.py` failis selle asukoht deklareerida.

Ava `/workspace/ros2_ws/src/my_launch_pkg/setup.py` ja muuda selliseks:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'my_launch_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='Launch package for py_pubsub demo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

*Ära unusta `maintainer` ja muid välju enda andmetega täita.*

## Ülesanne D: Ehita ja käivita (kohustuslik)

```bash
cd /workspace/ros2_ws
colcon build --packages-select my_launch_pkg py_pubsub
source install/setup.bash
ros2 launch my_launch_pkg pubsub_launch.py
```

Kui kõik töötab, näed ühes terminalis nii **"Publishing..."** kui ka
**"I heard..."** teateid vaheldumisi. See näitab, et mõlemad node'id
käivitusid edukalt.

Peatamiseks vajuta **Ctrl + C**.

## Ülesanne E: Kontroll (valikuline)

Jäta launch-fail ühes terminalis tööle. Ava **uus terminal**
(`docker exec -it <container-name> bash` host-arvutis) ja proovi:

```bash
cd /workspace/ros2_ws
source install/setup.bash

ros2 node list
# /my_talker
# /my_listener

ros2 topic list
# /topic
```

## Kontrollpunktid

- [ ] `my_launch_pkg/launch/pubsub_launch.py` eksisteerib ja defineerib mõlemad node'id.
- [ ] `setup.py` `data_files` sisaldab launch-kausta kaasamise rida.
- [ ] `colcon build --packages-select my_launch_pkg py_pubsub` läbib veata.
- [ ] `ros2 launch my_launch_pkg pubsub_launch.py` käivitab mõlemad node'id
      (näed nii "Publishing" kui "I heard" ridu).
- [ ] `ros2 node list` näitab `/my_talker` ja `/my_listener`.

## Esitamine

Paki `ros2_ws/src/my_launch_pkg` (ja vajadusel `ros2_ws/src/py_pubsub`,
kui seal midagi muutsid) ZIP-failiks ja lae üles Moodle'isse selle
nädala ülesande juurde.
