Dies ist eine Demo für einen Greifarm, welcher durch das ROS2 Visualization tool (rviz2)
angesteuert wird.
Das Projekt ist unvollständig und wird vorraussichtlich sowohl um eine Fahrbasis als auch durch Sensoren ergänzt. 
Der Zweck dieses Repos ist Momentan die Darstellung eines Beispiels, wie einzelne Aspekte des ROS-Frameworks kombiniert werden können.
Durch eine .urdf (universal robot description file) Datei kann der Zusammenhang einzelner Roboterteile zueinander beschrieben werden. 
Die Roboterteile können als STL mesh gegeben (wie in diesem Fall, diese befinden sich im subordner "meshes" des rviz-teils) oder durch die urdf-Datei durch Primitive wie Zylinder und Quader beschrieben werden.
Rviz, das diese urdf-Datei ausliest, gibt den aktuellen status eines Freiheitsgerades (joints, welche nicht die Bezeichnung "fixed" hat) also sogenannte 
jointstate-msg aus.
Diese message besteht aus:
	-Name des Joints
	-Position
	-Geschwindigkeit
	-Kraft
alle als Datenfeld gegeben. Die Position in dem Datenfeld gibt die "nummer" des joints wieder, position[1] ist die position des zweiten joints.
Diese Message kann von einem micro-ROS Node, der auf einem Mikrocontroller wie dem Raspberry Pi Pico läuft, "gehört" werden.
Zu den Grundprinzipien wie Publishe rund Subscriber wird auf:
https://answers.ros.org/question/185205/what-are-publishers-and-subscribers/
verwiesen und empfohlen, diese entweder in Python oder C++ einmal auszutesten.

vorab sei empfohlen, benötigte Packages zu installieren:

    sudo apt install build-essential cmake gcc-arm-none-eabi libnewlib-arm-none-eabi doxygen git python3


Als Mikrocontroller wurde an der stelle der Raspberry Pi Pico genutzt, was heißt, dass das PICO-SDK hierfür installiert sein muss. Falls nicht vorhanden, kann die Datei:
./microros_pkg/micro_ros_raspberrypi_pico_sdk2/build/pico_micro_ros_example.uf2 
auf einen Pico geladen werden.

Zur installation des Pico-SDKs wird auf: 
https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf 
verwiesen.

Wichtig ist es hier, die Umgebungsvariable:
"PICO_SDK_PATH"
zu setzen.
Dies kann durch einen Befehl wie: 
    export PICO_SDK_PATH=(Installationspfad)/pico/pico-sdk
getätigt werden.

Ist die .uf2 Datei für den Flash des Pico nun geladen, muss der micro-ros-agent verbindung zum pico aufnehmen, damit dieser mit dem Rest des ROS-Netzwerks
kommunizieren kann.

Der micro-ros-agent ist nicht standardmäßig installiert. Er kann entweder installiert werden durch: 

    sudo snap install micro-ros-agent

ODER durch den micro-ros-setup erstellt werden:
https://micro.ros.org/docs/tutorials/core/first_application_linux/
WICTHIG: für Turtlebot-Nutzer wird empfohlen, die Umgebungsvariable "ROS_DOMAIN_ID" nicht zu setzen, sonst findet ROS den microros-node nicht
AUSSERDEM: für das OpenCR board wird der micro-ros-setup empfohlen. 
genutzt wurde bislang nur ubuntu. Für ubuntu muss noch das Kommando:

    sudo snap set core experimental.hotplug=true && sudo systemctl restart snapd

ausgeführt werden. 

**Pico:**

Um zu schauen, dass der Pico verbunden ist, hilft das Kommando:

    snap interface serial-port

Ist er verbunden, muss der micro-ros-agent mit dem Port verbunden werden:

    snap connect micro-ros-agent:serial-port snapd:pico

Jetzt kann endlich der Node gestartet werden mit:

    micro-ros-agent serial --dev /dev/ttyACM0 baudrate=115200
    
**OPENCR-Board**:

Das repo wurde nicht für das OpenCR-Board erstellt. Da es in vergleichbarer Form jedoch später auf diesem laufen soll, wird auf as Board eingegangen.
für einen Start mit dem durch den micro-ros-setup muss dieser verfügbar sein, das geht durch das Kommando:
    source (microros_agent_pfad)/install/setup.bash

dann starten durch das Kommando:
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

**Kommunikation der Nodes:**

Durch das Öffnen eines neuen Terminals kann der erfolgreiche Start mit :

    ros2 node list

geprüft werden.

für den rviz-teil wird empfohlen, in den ros-workspace zu gehen und das kommando:

    colcon build --packages-select rviz_Greifarm

auszuführen
zuletzt ist zu sourcen und in den Package-Pfad zu gehen:
    source (rviz_package_pfad)/install/setup.bash
    cd rviz_Greifarm 
    
und dann final das Kommando:

    ros2 launch rviz_Greifarm display.launch.py model:=urdf/Eurobot_konzept_idee.urdf

auszuführen.  Nun kann durch rviz der Greifarm kontrolliert werden.

