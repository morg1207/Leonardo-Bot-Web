# LEONARDO-BOT-WEB

## 1 Instalar y configurar

## 1.1 Install vue

```sh
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
\. "$HOME/.nvm/nvm.sh"
nvm install 22
```

## 1.2 Install ROSBRIDGE package
```sh
sudo apt install ros-humble-rosbridge-suite
```

## 1.3 Clonar el repositorio

```sh
cd ~
git clone https://github.com/morg1207/Leonardo-Bot-Web.git
```

## 1.4 Instalar las depedencias npm

```sh
cd ~/Leonardo-Bot-Web
# Ejecutar "npm install" dentro de la carpeta del repositorio
npm install
```

### 2. Ejecutar


## 2.1 Compilar y ejecutar el servidor
Terminal 1
```sh
# Ejecutar el servidor web
npm run dev -- --host
```
## 2.2 Ejecutar rosbridge 

Terminal 2
```sh
# Ejecuta el enlace del websocket
source /opt/ros/${ROS_DISTRO}/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```