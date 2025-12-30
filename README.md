# Week 13–14: Launch-failid (Python) – `py_pubsub` käivitamine

Selle nädala eesmärk on koostada ROS 2 **launch-fail**, mis käivitab korraga mitu sinu enda kirjutatud node'i. Kasutame selleks eelmistes harjutustes loodud `py_pubsub` paketi publisheri (`talker`) ja subscriberi (`listener`) noode.


## Õpiväljundid
- Lood uue ROS 2 paketi launch-failide hoidmiseks.
- Kirjutad Pythonis launch-faili, mis käivitab kaks node'i (`talker` ja `listener`).
- Käivitad kogu süsteemi ühe käsuga: `ros2 launch`.
- Mõistad, kuidas lisada olemasoleva paketi node'e launch-faili.

## Eeldused
- ROS 2 Humble keskkond töötab (IDX/Docker).
- Sinu workspace'is (`/workspace/ros2_ws/src`) on olemas `py_pubsub` pakett (loodud 9.–10. nädalal).

---

## Ülesanne A: Loo uus launch-pakett

Hea tava on hoida launch-faile kas spetsiaalses paketis või selle paketi sees, mille node'e käivitatakse. Loome selguse huvides uue paketi.

1.  Mine `src` kausta:
    ```bash
    cd /workspace/ros2_ws/src
    ```

2.  Loo uus Pythoni pakett nimega `my_launch_pkg`:
    ```bash
    ros2 pkg create --build-type ament_python --license Apache-2.0 my_launch_pkg
    ```

3.  Loo uue paketi sisse `launch` kaust:
    ```bash
    cd my_launch_pkg
    mkdir launch
    ```

---

## Ülesanne B: Kirjuta Launch-fail

See fail kirjeldab, millised node'id ja mis parameetritega käivitada.

1.  Loo `launch` kausta fail nimega `pubsub_launch.py`:
    ```bash
    # Veendu, et oled /workspace/ros2_ws/src/my_launch_pkg/launch kaustas
    touch pubsub_launch.py
    ```

2.  Ava loodud fail ja kopeeri sinna järgmine sisu:

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

---

## Ülesanne C: Tee launch-fail paketile nähtavaks

Selleks, et `ros2 launch` käsk leiaks meie `pubsub_launch.py` faili üles, peame `setup.py` failis selle asukoha deklareerima.

1.  Ava `/workspace/ros2_ws/src/my_launch_pkg/setup.py` fail.

2.  Muuda see fail selliseks (lisa `import` ja `data_files`): 

    ```python
    import os
    from glob import glob
    from setuptools import setup

    package_name = 'my_launch_pkg'

    setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        # Lisa see osa
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            # Lisa see rida, et launch-failid kaasataks
            (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='Your Name',
        maintainer_email='you@email.com',
        description='TODO: Package description',
        license='Apache-2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
            ],
        },
    )
    ```
    *Ära unusta `maintainer` ja muid välju enda andmetega täita.*

---

## Ülesanne D: Ehita ja käivita

1.  Mine workspace'i juurkausta:
    ```bash
    cd /workspace/ros2_ws
    ```

2.  Ehita oma uus launch-pakett (ja vajadusel ka `py_pubsub`):
    ```bash
    colcon build --packages-select my_launch_pkg py_pubsub
    ```

3.  Tee ehitatud paketid terminalis kättesaadavaks:
    ```bash
    source install/setup.bash
    ```

4.  Käivita launch-fail:
    ```bash
    ros2 launch my_launch_pkg pubsub_launch.py
    ```

    Kui kõik töötab, näed ühes terminalis nii **"Publishing..."** kui ka **"I heard..."** teateid vaheldumisi. See näitab, et mõlemad node'id käivitusid edukalt.

    Peatamiseks vajuta **Ctrl + C**.

---

## Ülesanne E: Kontroll (valikuline)

Jäta launch-fail ühes terminalis tööle. Ava **uus terminal** ja proovi järgmiseid käske, et näha, mis süsteemis toimub:

```bash
# Enne source'i oma workspace!
cd /workspace/ros2_ws
source install/setup.bash

# Vaata aktiivseid node'e (peaksid nägema /my_talker ja /my_listener)
ros2 node list

# Vaata aktiivseid topiceid (peaksid nägema /topic)
ros2 topic list
```

---

## Esitamise kord

Harjutuse edukaks esitamiseks **pead töötama oma isiklikus GitHub Classroomi repos**, mis on loodud selle nädala jaoks.

1.  **Ava ülesande link Moodle'ist:**
    - Mine kursuse Moodle'i lehele.
    - Leia sealt selle nädala (Week 13-14) ülesande juurest link oma isikliku GitHub Classroomi repositooriumi loomiseks.

2.  **Klooni enda isiklik repo GitHubist**, mille nimi sisaldab sinu GitHubi kasutajanime.
    Näide:
    ```bash
    git clone https://github.com/Tallinna-Tehnika-korgkool/TRO029-week13-14_launch-<sinu_kasutajanimi>
    ```

3.  **Tee kõik ülesanded selles kloonitud repos.**

4.  **Lisa ja `commit`'i oma muudatused** (`ros2_ws` kaust koos sinu paketiga).
    ```bash
    git add .
    git commit -m "docs: Complete week 13-14 exercise"
    git push
    ```

5.  Kui `push` on tehtud, käivituvad **automaatsed testid** (GitHub Actions) sinu repo **Actions** vahekaardil.
    - ✅ **Roheline** ✔️ tähendab, et kõik on korras.
    - ❌ **Punane** ✖️ tähendab, et midagi on puudu või valesti – paranda ja tee uus `commit` ja `push`.

> **NB!** Kui töötad väljaspool oma isiklikku Classroom repo't, siis testid ei tööta ja harjutust ei loeta esitatuks.
