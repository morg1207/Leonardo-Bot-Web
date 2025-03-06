# leo

This template should help get you started developing with Vue 3 in Vite.

## Recommended IDE Setup

[VSCode](https://code.visualstudio.com/) + [Volar](https://marketplace.visualstudio.com/items?itemName=Vue.volar) (and disable Vetur).

## Customize configuration

See [Vite Configuration Reference](https://vite.dev/config/).

## Project Setup

```sh
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

### Lint with [ESLint](https://eslint.org/)

```sh
npm run lint
```


### 1.5 Install ROSBRIDGE package
```sh
sudo apt install ros-humble-rosbridge-suite
```

## 3. Launch server

Terminal 1
```sh
source /opt/ros/humnle/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```