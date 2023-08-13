Dies ist der branch der MAMUT-Micro-ROS-Demos für das OpenCR-board.
Es handelt sich um ein selbst geschriebenes Paket, um den Openmanipulator-X mithilfe des OpenCR-Boards und des Robot Visualizers (rviz) zu kontrollieren.
**Es mag bereits solche Pakete geben!**
Das Packet wurde in erster linie erstellt, um mit ROS ein wenig zu lernen und ist deshalb noch relativ minimalistisch.

Der Zweck dieses Repos ist Momentan die Darstellung eines Beispiels, wie einzelne Aspekte des ROS-Frameworks kombiniert werden können. Durch eine .urdf (universal robot description file) Datei kann der Zusammenhang einzelner Roboterteile zueinander beschrieben werden. Die Roboterteile können als STL mesh gegeben (wie in diesem Fall, diese befinden sich im subordner "meshes" des rviz-teils) oder durch die urdf-Datei durch Primitive wie Zylinder und Quader beschrieben werden. Rviz, das diese urdf-Datei ausliest, gibt den aktuellen status eines Freiheitsgerades (joints, welche nicht die Bezeichnung "fixed" hat) also sogenannte jointstate-msg aus. Diese message besteht aus:

-Name des Joints 

-Position 

-Geschwindigkeit 

-Kraft 


Es sei gesagt, dass die Nodes mit Ubuntu 22.04 und ROS2 Humble Hawksbill erstellt wurden und vorausgesetzt, dass dieses installiert ist.
Es kann nicht gerantiert werden, dass dieses auf anderen Plattformen funktioniert.

**Funktion**:

Der Rviz-Node startet neben der Grafischen Darstellung auch den "joint-state-publisher", welcher den Status des Freiheitsgrades ausgibt. 
Interessanterweise kann micro-ROS diesen nicht selbst verarbeiten. Darum wurde als kurzfristige Lösung entschieden, einen zwischen-Node namens "py_jointsub" zu erstellen, 
der die Message aufniommt und in geringerer Frequenz publisht.
Zuletzt wird mit der Arduino IDE (anders nicht möglich) ein Sketch auf das OpenCR-Board geflasht und der micro-ros-agent schließt das Board quasi an das Netzwerk an.

**Installation**:

zunächst in die bash, in den ROS-Workspace gehen(falls nicht vorhanden, einfach neuen Ordner erstellen) und das Repository klonen:

    git clone -b OpenCR_Manip https://github.com/Bbenno-UX/MAMUT_ROS_Greifarm.git
    cd branch_opencr

**RVIZ-Node:**

RViZ:
Für den Fall, dass nur die Light-version von ROS2 installiert wurde, müssen die Abhängigkeiten installiert werden.
ROS-Packages geben normalerweise die Abhängigkeiten in der Datei "package.xml" für die leichte Installation mit an
Dies geht mit rosdep:
    sudo apt update && rosdep update
    rosdep install --from-paths rviz_Greifarm -y
Wenn dies erledigt ist, kann mithilfe von:
    colcon build --packages-select rviz_Greifarm
installiert werden
für das Ausführen muss ROS wissen, wo sich das Package befindet:
    source install/setup.bash
zum Finalen ausführen gibt es das Kommando:

    ros2 launch rviz_Greifarm display.launch.py model:=rviz_Greifarm/urdf/Eurobot_konzept_idee.urdf
    
**py_jointsub:**

Der Node hat keine nennenswerten Abhängigkeiten, einfach:
    colcon build --packages-select py_jointsub
    source install/setup.bash
    ros2 run py_jointsub listner

**OpenCR-Board**:

Um das OpenCR-Board ans laufen zu bringen, braucht die Arduino IDE zugriff auf die URL. Das geht mit:
File>Preferences
bei "Additional boards manager URLS" den Link:

https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json

hereinladen. Wenn schon ein Link drin ist, bei mit einem "," separieren.
Dann: Tools>Boards>Boards-Manager
Dort nach dem OpenCR board suchen und installieren.
Außerdem muss die Arduino-Bibliothek "Dynamixel2Arduino" installiert werden. 

Dann am besten den Repository-Sketch in den Arduino-Pfad verschieben

    cp -r Arduino/opencr_manip ${ARDUINO_ORDNER}/opencr_manip

WICHTIG:
Das OpenCR-Board ist sehr eigen. Es wird empfohlen, vor dem Upload:

-Button SW2 drücken

-RESET Button drücken

-RESET Button loslassen

-Button SW2 loslassen

Wenn das Board geflasht ist, dann muss der micro-ros-agent die Verbindung zum ROS-Netzwerk herstellen.

**Micro-Ros:**

Es sei gesagt, dass Micro-ros probleme machen kann, wenn man es in einen Workspace mit restlichen Rosnodes schmeisst, weswegen ein seperater Workspace zu erstellen empfohlen wird.
Folgender Link beinhaltet eine beschreibung, wie dieser zu erstellen ist:

https://micro.ros.org/docs/tutorials/core/first_application_linux/

Ist der Agent gebaut, ist in den Micro-Ros-Workspace zu navigieren:

    cd microros_ws
    source install/setup.bash

Vorher sollte geguckt werden, dass die Variable "ROS_DOMAIN_ID" nicht gesetzt ist, kann mit:
    echo $ROS_DOMAIN_ID
überprüft werden, es sollte eine leere Zeile zurückgegeben werden
Nun den micro-ros-agent starten mit:

    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
/dev/ttyACM0 durch aktuellen Port ersetzten, kann durch ein- und ausstecken, jeweils gefolgt vom Befehl:
    ls /dev/tty*
nachgeguckt werden.

**launch multipler Nodes:**

Es kann, wenn alle Packages im selben Ordner liegen, auch alles gleichzeitig durch ein Kommando gestartet werden:
    ros2 launch rviz_Greifarm display.launch_all.py model:=rviz_Greifarm/urdf/open_manipulator_x.urdf
Die Datei "display.launch_all.py" soll zeigen, wie so etwas gemacht werden kann.
Aufgrund der micro-ros-konflikte wird dies jedoch nicht empfohlen.

