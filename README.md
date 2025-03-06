# LEONARDO-BOT-WEB

## 1.1 Install vue

```sh
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
\. "$HOME/.nvm/nvm.sh"
nvm install 22
```
## 1.2 Clone repository

```sh
cd ~
git clone https://github.com/morg1207/Leonardo-Bot-Web.git
```

## 1.3 Project Setup

```sh
cd ~/Leonardo-Bot-Web
npm install
```

### Compile and Hot-Reload for Development

```sh
npm run dev
```

### Compile and Minify for Production

```sh
npm run build
```

## 1.4 Install ROSBRIDGE package
```sh
sudo apt install ros-humble-rosbridge-suite
```

## 1.5 Launch server

Terminal 1
```sh
source /opt/ros/humnle/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```