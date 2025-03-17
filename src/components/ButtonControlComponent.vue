<template>
  <div class="control-panel">
    <!-- Section Title -->
    <h2 class="title text-center text-2xl font-bold text-blue-400 mb-4">Robot Control</h2>

    <!-- Directional Controls -->
    <div v-if="controlEnabled" class="control-content mb-4 p-4 bg-gray-700 rounded-xl shadow-lg">
      <!-- Contenedor flexible para botones y sliders -->
      <div class="flex flex-col lg:flex-row items-center gap-4">
        <!-- Botones de dirección -->
        <div class="button-controls flex flex-col items-center gap-2">
          <button 
            @touchstart="move('up')" 
            @touchend="stop" 
            @mousedown="move('up')" 
            @mouseup="stop" 
            class="control-button1 p-2">
            <i class="fas fa-arrow-up"></i> <!-- Icono de flecha hacia arriba -->
          </button>
          <div class="horizontal-buttons flex gap-2">
            <button 
              @touchstart="move('left')" 
              @touchend="stop" 
              @mousedown="move('left')" 
              @mouseup="stop" 
              class="control-button1 p-2">
              <i class="fas fa-arrow-left"></i> <!-- Icono de flecha hacia la izquierda -->
            </button>
            <button 
              @touchstart="move('stop')" 
              @touchend="stop" 
              @mousedown="move('stop')" 
              @mouseup="stop" 
              class="control-button1 p-2">
              <i class="fas fa-stop"></i> <!-- Icono de stop -->
            </button>
            <button 
              @touchstart="move('right')" 
              @touchend="stop" 
              @mousedown="move('right')" 
              @mouseup="stop" 
              class="control-button1 p-2">
              <i class="fas fa-arrow-right"></i> <!-- Icono de flecha hacia la derecha -->
            </button>
          </div>
          <button 
            @touchstart="move('down')" 
            @touchend="stop" 
            @mousedown="move('down')" 
            @mouseup="stop" 
            class="control-button1 p-2">
            <i class="fas fa-arrow-down"></i> <!-- Icono de flecha hacia abajo -->
          </button>
        </div>

        <!-- Sliders para velocidad horizontal y de rotación -->
        <div class="flex flex-col gap-4 w-full max-w-xs"> <!-- Ancho máximo para los sliders -->
          <!-- Slider de velocidad horizontal -->
          <div class="flex flex-col items-center">
            <label for="horizontal-speed" class="text-sm font-bold text-gray-300 mb-1">
              Horizontal Speed:
              <span class="speed-value bg-blue-500 text-white px-2 py-1 rounded-md">
                {{ (horizontalSpeed / 100).toFixed(2) }}
              </span>
            </label>
            <input
              type="range"
              id="horizontal-speed"
              v-model="horizontalSpeed"
              min="0"
              max="100"
              class="slider-vel-x w-full">
          </div>

          <!-- Slider de velocidad de rotación -->
          <div class="flex flex-col items-center">
            <label for="rotation-speed" class="text-sm font-bold text-gray-300 mb-1">
              Rotation Speed:
              <span class="speed-value bg-blue-500 text-white px-2 py-1 rounded-md">
                {{ (rotationSpeed / 100).toFixed(2) }}
              </span>
            </label>
            <input
              type="range"
              id="rotation-speed"
              v-model="rotationSpeed"
              min="0"
              max="100"
              class="slider-vel-z w-full h-3 bg-gray-400 rounded-lg">
          </div>
        </div>
      </div>
    </div>

    <!-- Arm Height Control -->
    <div v-if="controlEnabled" class="control-content p-4 bg-gray-700 rounded-xl shadow-lg mt-6">
      <div class="container">
        <span class="label">Arm Height (cm): </span>
        <div class="flex gap-4 mt-4">
          <div class="relative w-32">
            <input type="text" id="pos_z" class="input-field" :value="height" readonly />
            <label for="pos_z" class="floating-label">cm</label>
          </div>
        </div>
        <input
          type="range"
          v-model="height"
          min="0"
          max="100"
          @input="updateSlider"
          class="slider-arm w-full h-3 bg-gray-400 rounded-lg mt-4">
        
        <!-- Paint Test Button and Status -->
        <div class="flex items-center gap-2">
          <button 
            @click="togglePaintTest" 
            class="control-button-paint toggle-button-paint"
            :class="{ 'active': isPaintTestEnabled }">
            {{ isPaintTestEnabled ? "Disable Paint Test" : "Enable Paint Test" }}
          </button>
          <div v-if="isPaintTestEnabled" class="flex items-center gap-2">
            <span class="text-yellow-400 font-bold text-2xl">🪣 Painting...</span>
            <div class="w-4 h-4 rounded-full bg-blue-500 text-3xl"></div> <!-- Círculo de color -->
          </div>
        </div>
      </div>
    </div>

    <!-- Toggle Manual Control Button -->
    <div class="control-content bg-gray-700 rounded-lg shadow-lg mt-6">
      <button 
        @click="toggleManualControl" 
        class="control-button toggle-button py-2 px-4"
        :class="{ 'active': controlEnabled }">
        {{ controlEnabled ? "Disable Manual Control" : "Enable Manual Control" }}
      </button>
    </div>

    <!-- Routine Control Button -->
    <div v-if="!controlEnabled" class="p-2 bg-gray-700 rounded-lg shadow-lg mt-6">
      <button 
        @click="toggleRoutineControl" 
        class="control-button toggle-button">
        {{ controlRoutineEnabled ? "Disable Painting Routine" : "Enable Painting Routine" }}
      </button>
    </div>
  </div>    
</template>
<script>
import ROSLIB from "roslib";

export default {
  name: "ButtonControlComponent",
  props: {
    ros: {
      type: Object,
      required: true,
    },
  },
  data() {
    return {
      cmdVel: null,
      controlEnabled: false,  // Manual control enabled by default
      controlRoutineEnabled: false,  // Manual control enabled by default
      isPaintTestEnabled: false,  // Paint test enabled by default
      pubInterval: null,  // Publication interval in ms
      timeInterval: 100,  // Publication interval in ms

      horizontalSpeed: 50, // Valor inicial de la velocidad horizontal
      rotationSpeed: 50,   // Valor inicial de la velocidad de rotación

      ros: null,
      // ROS Topic names
      cmdVelTopic: "/turtle1/cmd_vel",
      armTopic: "/arm_controller",
      paintTestTopic: "/test_paint",

      // Control variables
      height: 0.0,
      linear_x: 0,
      angular_z: 0,
    };
  },
  methods: {
    /**
     * Move the robot in the specified direction.
     * @param {string} direction - The direction to move ('up', 'down', 'left', 'right', 'stop').
     */
    init(ros) {
      console.log("Inicializando ButtonControlComponent");
      this.ros = ros;
    },
    move(direction) {
      console.log(`Moving in direction: ${direction}`);
      if (!this.controlEnabled) return;  // Do nothing if control is disabled

      switch (direction) {
        case "up":
          this.linear_x = this.horizontalSpeed / 100;
          break;
        case "down":
          this.linear_x = -this.horizontalSpeed / 100;
          break;
        case "left":
          this.angular_z = this.rotationSpeed / 100;
          break;
        case "right":
          this.angular_z = -this.rotationSpeed / 100;
          break;
        case "stop":
          this.linear_x = 0.0;
          this.angular_z = 0.0;
          break;
      }

      if (this.pubInterval !== null) return;
      this.pubInterval = setInterval(this.publishCmdVel, this.timeInterval);
      console.log("Publishing velocity command");
    },

    /**
     * Stop the robot's movement.
     */
    stop() {
      if (!this.controlEnabled) return;  // Do nothing if control is disabled
      this.linear_x = 0.0;
      this.angular_z = 0.0;

      this.publishCmdVel();
      clearInterval(this.pubInterval);
      this.pubInterval = null;
      console.log("Stopping the robot");
    },

    /**
     * Publish the velocity command to the ROS topic.
     */
    publishCmdVel() {
      if (!this.ros) {
        console.error("Error: ROS is not initialized.");
        return;
      }
      let topic = new ROSLIB.Topic({
        ros: this.ros,
        name: this.cmdVelTopic,
        messageType: "geometry_msgs/Twist",
      });
      let message = new ROSLIB.Message({
        linear: { x: this.linear_x, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: this.angular_z },
      });
      console.log("Publishing velocity message");
      topic.publish(message);
    },

    /**
     * Toggle the paint test mode.
     */
    togglePaintTest() {
      this.isPaintTestEnabled = !this.isPaintTestEnabled;
      this.publishPaintTest();
    },

    /**
     * Publish the paint test command to the ROS topic.
     */
    publishPaintTest() {
      if (!this.ros) {
        console.error("Error: ROS is not initialized.");
        return;
      }
      let topic = new ROSLIB.Topic({
        ros: this.ros,
        name: this.paintTestTopic,
        messageType: "std_msgs/Bool",
      });
      let message = new ROSLIB.Message({
        data: this.isPaintTestEnabled,
      });
      console.log("Paint test state: ", this.isPaintTestEnabled);
      topic.publish(message);
    },

    /**
     * Update the arm height slider and publish the value to the ROS topic.
     */
    updateSlider() {
      let sliderElement = document.querySelector('.slider');
      let valPercent = (this.height / sliderElement.max) * 100;
      sliderElement.style.background = `linear-gradient(to right, #3264fe ${valPercent}%, #d5d5d5 ${valPercent}%)`;

      console.log(`Current height: ${this.height}`);
      this.publishHeight();
    },

    /**
     * Publish the arm height to the ROS topic.
     */
    publishHeight() {
      if (!this.ros) {
        console.error("Error: ROS is not initialized.");
        return;
      }
      let topic = new ROSLIB.Topic({
        ros: this.ros,
        name: this.armTopic,
        messageType: "std_msgs/Float64",
      });
      let message = new ROSLIB.Message({
        data: Number(this.height),
      });
      console.log("Publishing height message");
      topic.publish(message);
    },

    /**
     * Toggle manual control mode.
     */
    toggleManualControl() {
      this.controlEnabled = !this.controlEnabled;
      if (!this.controlEnabled) {
        this.isPaintTestEnabled = false;
        this.publishPaintTest();
        console.log("Manual control disabled");
      } else {
        console.log("Manual control enabled");
      }
    },

    /**
     * Toggle routine control mode.
     */
    toggleRoutineControl() {
      console.log("Routine control toggled");
      // Add routine control logic here
    },
  },
};
</script>

<style scoped>
@import url('https://fonts.googleapis.com/css2?family=Roboto:wght@700&display=swap');
@import "tailwindcss";
@import '@fortawesome/fontawesome-free/css/all.min.css';
.control-panel {
  flex-direction: column;
  width: 100%;
  max-width: 100%;
  margin: 0 auto;
  padding: 2rem;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  background-color: #222;
  color: white;
  border-radius: 10px;
}

.control-content {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
}

.button-controls {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  margin-left: auto;
  margin-right: auto;
}

.horizontal-buttons {
  display: flex;
  justify-content: center;
  margin: 0.5rem 0;
}

.control-button {
  background: #2563eb;
  color: white;
  border: none;
  border-radius: 10px;
  padding: 1rem;
  margin: 1rem;
  font-size: 1.0rem;
  cursor: pointer;
  transition: background 0.1s;
  min-width: 50px;
  display: block;
  margin-left: auto;
  margin-right: auto;
}

.control-button-paint {
  background: #13cc76;
  color: white;
  border: none;
  border-radius: 10px;
  padding: 1rem;
  margin: 1rem;
  font-size: 1.0rem;
  cursor: pointer;
  transition: background 0.1s;
  min-width: 50px;
  display: block;
  margin-left: auto;
  margin-right: auto;
}

.control-button1 {
  background: #2563eb;
  color: white;
  border: none;
  border-radius: 10px;
  padding: 1rem;
  margin: 1rem;
  font-size: 1.5rem;
  cursor: pointer;
  transition: background 0.1s;
  min-width: 50px;
}

.control-button:hover {
  background: #3b82f6;
}

.label {
  font-weight: bold;
  margin-bottom: 10px;
  display: block;
  color: #93c5fd;
  font-size: 16px;
}

.input-field {
  block-size: 40px;
  width: 100%;
  text-align: center;
  background: transparent;
  color: #fff;
  border: 1px solid #3b82f6;
  border-radius: 8px;
  padding: 10px;
  font-size: 14px;
  outline: none;
  appearance: none;
}

.input-field:focus {
  border-color: #2563eb;
}

.floating-label {
  position: absolute;
  left: 10px;
  top: 10px;
  font-size: 14px;
  color: #93c5fd;
  background-color: #222;
  padding: 0 5px;
  transition: all 0.3s ease-in-out;
}

.input-field:focus + .floating-label,
.input-field:not(:placeholder-shown) + .floating-label {
  top: -5px;
  left: 12px;
  font-size: 12px;
  color: #60a5fa;
}

.position-indicators, .orientation-indicator {
  margin-top: 15px;
}

.toggle-button {
  background-color: #3264fe;
  margin-top: 1rem;
}
.toggle-button.active {
  background-color: #D9534F;
}

.toggle-button-paint {
  background-color: #40AD55;
  margin-top: 1rem;
}

.toggle-button-paint.active {
  background-color: #D9534F;
}

.speed-value {
  display: inline-block;
  min-width: 40px; /* Ancho mínimo para evitar cambios de tamaño */
  text-align: center;
  font-size: 14px;
  font-weight: bold;
  transition: background-color 0.3s ease; /* Transición suave para el fondo */
}

.speed-value:hover {
  background-color: #3182ce; /* Cambia el color al pasar el mouse */
}


.slider {
  -webkit-appearance: none;
  appearance: none;
  height: 8px;
  background: #4a5568;
  outline: none;
  border-radius: 4px;
}

.slider::-webkit-slider-thumb {
  -webkit-appearance: none;
  appearance: none;
  width: 16px;
  height: 16px;
  background: #4299e1;
  border-radius: 50%;
  cursor: pointer;
}

@media (max-width: 768px) {
  .control-panel {
    flex-direction: column;
  }

  .control-content {
    flex-direction: column;
  }

  .control-button {
    padding: 1rem;
    font-size: 1.2rem;
  }
}

@media (min-width: 768px) {
  .control-panel {
    width: 80%;
  }
}
</style>