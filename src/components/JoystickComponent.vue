<template>
  <div class="card mb-2 w-100">
    <div class="card-header">
      <h4>Joystick Control</h4>
    </div>
    <div class="card-body">
      <button class="btn btn-primary"
              :class="{ 'btn-disabled': !joystickConfigured, 'btn-active': joystickControlEnabled }"
              @click="toggleJoystickControl"
              :disabled="!joystickConfigured">
        {{ joystickControlEnabled ? 'Disable Joystick Control' : 'Enable Joystick Control' }}
      </button>

      <div class="joystick-content mt-3" v-if="joystickControlEnabled">
        <div id="dragstartzone" @mousedown="startDrag" @mousemove="doDrag"></div>
        <div id="dragCircle" :style="dragCircleStyle"></div>
      </div>

      <div class="joystick-values mt-3">
        <h5>Joystick Values</h5>
        <hr />
        <p v-if="joystickControlEnabled">Vertical: {{ joystick.vertical.toFixed(3) }}</p>
        <p v-if="joystickControlEnabled">Horizontal: {{ joystick.horizontal.toFixed(3) }}</p>
        <p v-else>Joystick control is disabled.</p>
      </div>
    </div>
  </div>
</template>
<script>
import ROSLIB from "roslib";


export default {
  name: "JoystickComponent",
  data() {
    return {
      // values
      ros: null,
      // publisher
      pubInterval: null,
      joystick: {
        vertical: 0,
        horizontal: 0,
      },
      // joystick grapth
      dragging: false,
      x: "no",
      y: "no",
      dragCircleStyle: {
        margin: "0px",
        top: "0px",
        left: "0px",
        display: "none",
        width: "75px",
        height: "75px",
      },
      topic_cmd_vel: "turtle1/cmd_vel",
      joystickControlEnabled: false,
      joystickConfigured: false,
      shelf_detected: false
    };
  },

  methods: {

    toggleJoystickControl()  {
      if (!this.joystickConfigured) return;

      this.joystickControlEnabled = !this.joystickControlEnabled;
      if (this.joystickControlEnabled) {
        this.activateJoystickControl();
      } else {
        this.deactivateJoystickControl();
      }
    },

    activateJoystickControl(){
      this.joystickControlEnabled=true;
      this.pubInterval = setInterval(this.publishCmdVel, 100);
      console.log("Publicando mensajes");
    },
    deactivateJoystickControl(){
      this.joystickControlEnabled=false;
      clearInterval(this.pubInterval)
      console.log("Joystick desactivado");
    },
    
    joystickConfig(ros) {
      this.joystickConfigured = true
      console.log("Joystick configurado");
      this.ros = ros;
      //this.pubInterval = setInterval(this.publishCmdVel, 100);
    },
    joystickClose() {
      this.joystickConfigured = false
      console.log("Joystick close");
      clearInterval(this.pubInterval)


    },

    paramConfig(config_web) {
      console.log(config_web);
      this.topic_name = config_web.topic_cmd_vel.value;
    },
    //enviar velocidades
    publishCmdVel: function () {
      let topic = new ROSLIB.Topic({
        ros: this.ros,
        name: this.topic_cmd_vel,
        messageType: "geometry_msgs/msg/Twist",
      });
      let message = new ROSLIB.Message({
        linear: { x: this.joystick.vertical, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: -1 * this.joystick.horizontal },
      });
      console.log("Publicando mensaje");
      topic.publish(message);
      
    },

    // funciones para el joystick
    startDrag() {
      this.dragging = true;
      this.x = this.y = 0;
    },
    stopDrag() {
      this.dragging = false;
      this.x = this.y = "no";
      this.dragCircleStyle.display = "none";
      this.resetJoystickVals();
    },
    doDrag(event) {
      if (this.dragging) {
        this.x = event.offsetX;
        this.y = event.offsetY;
        let ref = document.getElementById("dragstartzone");
        this.dragCircleStyle.display = "inline-block";

        let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2;
        //let maxTop = minTop + 200;
        let top = this.y + minTop;
        this.dragCircleStyle.top = `${top}px`;

        let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2;
        //let maxLeft = minLeft + 200;
        let left = this.x + minLeft;
        this.dragCircleStyle.left = `${left}px`;
        this.setJoystickVals();
      }
    },
    setJoystickVals() {
      this.joystick.vertical = -1 * (this.y / 200 - 0.5);
      this.joystick.horizontal = +1 * (this.x / 200 - 0.5);
    },
    resetJoystickVals() {
      this.joystick.vertical = 0;
      this.joystick.horizontal = 0;
    },
  },
  mounted() {
    // page is ready
    window.addEventListener("mouseup", this.stopDrag);
    this.interval = setInterval(() => {
      if (this.ros != null && this.ros.isConnected) {
        this.ros.getNodes(
          (data) => {
            console.log(data);
          },
          (error) => {
            console.log(error);
          }
        );
      }
    }, 10000);
  },
};
</script>

<style scoped>
.joystick-content {
  display: flex;
  flex-direction: column;
  align-items: center;
}

.joystick-values {
  margin-top: 20px;
}

.btn-disabled {
  opacity: 0.5; /* Botón opaco cuando está deshabilitado */
  cursor: not-allowed; /* Cursor no permitido cuando está deshabilitado */
}

.btn-active {
  background-color: #007bff; /* Color de fondo cuando está activo */
  border-color: #007bff; /* Color del borde cuando está activo */
  color: #fff; /* Color del texto cuando está activo */
}
#dragCircle {
  position: absolute;
  z-index: 9;
  border: 1px solid transparent;
  border-radius: 50%;
  background-color: rgba(0, 0, 0, 30%);
  -moz-user-select: -moz-none;
  -khtml-user-select: none;
  -webkit-user-select: none;
}

#dragstartzone {
  position: relative;
  display: inline-block;
  width: 200px;
  height: 200px;
  border: 1px solid #333;
  border-radius: 50%;
  z-index: 10;
  -moz-user-select: -moz-none;
  -khtml-user-select: none;
  -webkit-user-select: none;
}
#dragCircle:hover {
  background-color: lightcoral;
}

.joystick-column {
  display: flex;
  justify-content: center; /* Centra el contenido horizontalmente */
  align-items: center; /* Centra el contenido verticalmente */
  height: 100%; /* Asegura que la columna ocupe todo el espacio disponible */
  margin-top: 00px; /* Espacio superior uniforme */
}
</style>