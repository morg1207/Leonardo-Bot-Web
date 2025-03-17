<template>
    <div class="control-panel">
  
  
      <!-- Control de botones de dirección y slider -->
  
       <!-- Título de la sección -->
      <h2 v-if="controlEnabled" class="title text-center text-2xl font-bold text-blue-400 mb-6">Control robot</h2>
  
      <div v-if="controlEnabled" class="control-content mb-6 p-4 bg-gray-700 rounded-xl shadow-lg">
        <!-- Botones de dirección -->
        <div class="button-controls">
          <button @touchstart="move('up')" @touchend="stop" @mousedown="move('up')" @mouseup="stop" class="control-button1">⬆️</button>
          <div class="horizontal-buttons">
            <button @touchstart="move('left')" @touchend="stop" @mousedown="move('left')" @mouseup="stop" class="control-button1">⬅️</button>
            <button @touchstart="move('stop')" @touchend="stop" @mousedown="move('stop')" @mouseup="stop" class="control-button1">⏹️</button>
            <button @touchstart="move('right')" @touchend="stop" @mousedown="move('right')" @mouseup="stop" class="control-button1">➡️</button>
          </div>
          <button @touchstart="move('down')" @touchend="stop" @mousedown="move('down')" @mouseup="stop" class="control-button1">⬇️</button>
        </div>
      </div>
  
      <div v-if="controlEnabled" class="position-idicatoros p-4 bg-gray-700 rounded-xl shadow-lg mt-6">
        <!-- Slider de altura del brazo -->
        <div class="container">
  
          <!-- Mostrar el valor del slider -->
          <span class="label">Altura del brazo(cm): </span>
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
            class="slider w-full h-3 bg-gray-200 rounded-lg appearance-none cursor-pointer range-lg dark:bg-gray-700">   
          <!-- Button paint-->    
          <button 
            @click="testPaintControl" 
            class="control-button-paint toggle-button-paint"
            :class="{ 'active': testPaintEnabled }">
            {{ testPaintEnabled ? "Desactivar Test de Pintura" : "Activar Test de Pintura" }}
          </button> 
          <p v-if="testPaintEnabled" class="mt-4 text-yellow-400 font-bold">🖌️ Pintando...</p>
  
        </div>
      </div>
  
      <div lass="control-content mb-6 p-4 bg-gray-700 rounded-xl shadow-lg">
        <!-- Botón para activar/desactivar el control -->
        <button 
          @click="toggleControl" 
          class="control-button toggle-button">
          {{ controlEnabled ? "Desactivar Control Manual" : "Activar Control Manual" }}
        </button>
      </div>
  
      <div v-if="!controlEnabled" lass="mb-6 p-4 bg-gray-700 rounded-xl shadow-lg">
        <button 
          @click="rutineTaskControl" 
          class="control-button toggle-button">
          {{ controlEnabled ? "Activar rutina de pintado " : "Desactivar rutina de pintado" }}
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
        controlEnabled: false,  // Control habilitado por defecto
        testPaintEnabled: false,  // Control habilitado por defecto
        pubInterval: null,  // Intervalo de publicación en ms
        timeInterval: 100,  // Intervalo de publicación en ms
  
        //Topic names
        cmdVelTopic: "/turtle1/cmd_vel",
        armTopic: "/arm_controller",
        testPaintTopic: "/test_paint",
  
        
        height: 0.0,
        linear_x: 0,
        angular_z: 0,
        ros: null,
      };
    },
    methods: {
      move(direction) {
        console.log(`Moviendo en dirección ${direction}`);
        if (!this.controlEnabled) return;  // Si el control está desactivado, no hacer nada
  
        switch (direction) {
          case "up":
            this.linear_x = 1.0;
            break;
          case "down":
            this.linear_x = -1.0;
            break;
          case "left":
            this.angular_z = 1.0;
            break;
          case "right":
            this.angular_z = -1.0;
            break;
          case "stop":
            this.linear_x = 0.0;
            this.angular_z = 0.0;
            break;
        }
        if(this.pubInterval != null){
          return;
        }
        this.pubInterval = setInterval(this.publishCmdVel, this.timeInterval);
        console.log("Publicando mensaje de velocidad");
      },
      stop() {
        if (!this.controlEnabled) return;  // Si el control está desactivado, no hacer nada
        this.linear_x = 0.0;
        this.angular_z = 0.0;
  
        this.publishCmdVel;
         this.publishCmdVel;
        clearInterval(this.pubInterval);
        this.pubInterval = null;
        console.log("Deteniendo el robot");
      },
      init(ros) {
        console.log("Inicializando ButtonControlComponent");
        this.ros = ros;
      },
  
      publishCmdVel() {
        if (!this.ros) {
          console.error("Error: ROS no está inicializado.");
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
        console.log("Publicando mensaje");
        topic.publish(message);
      },
  
      publishTestPaint() {
        if (!this.ros) {
          console.error("Error: ROS no está inicializado.");
          return;
        }
        let topic = new ROSLIB.Topic({
          ros: this.ros,
          name: this.testPaintTopic,
          messageType: "std_msgs/Bool",
        });
        let message = new ROSLIB.Message({
          data: this.testPaintEnabled,
        });
        console.log("El estado del mensaje de pintura es: ", this.testPaintEnabled);
        topic.publish(message);
      },
  
      publishHeight() {
        if (!this.ros) {
          console.error("Error: ROS no está inicializado.");
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
        console.log("Publicando mensaje de altura");
        topic.publish(message);
      },
  
      toggleControl() {
        this.controlEnabled = !this.controlEnabled;  // Cambiar el estado del control
        if(!this.controlEnabled){
          this.testPaintEnabled  = false;
          this.publishTestPaint();
          console.log("Control manual desactivado");
        }
        else{
          console.log("Control manual activado");
        }
      },
      testPaintControl() {
        this.testPaintEnabled = !this.testPaintEnabled;  // Cambiar el estado del control
        this.publishTestPaint();
      },
  
      updateSlider() {
        // Actualiza el fondo del slider con un gradiente dinámico
        let sliderElement = document.querySelector('.slider');
        let valPercent = (this.height / sliderElement.max) * 100;
        sliderElement.style.background = `linear-gradient(to right, #3264fe ${valPercent}%, #d5d5d5 ${valPercent}%)`;
  
  
        console.log(`Altura actual: ${this.height}`);
        // Publica el valor del slider (altura)
        this.publishHeight();
      },
    },
  };
  </script>
  
  <style scoped>
  @import url('https://fonts.googleapis.com/css2?family=Roboto:wght@700&display=swap');
  @import "tailwindcss";
  
  .control-panel {
    flex-direction: column; /* Apilamos los elementos en columna */
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
    flex-direction: column;  /* Los controles de dirección y el slider en columna */
    justify-content: center;
    align-items: center;
  }
  
  .button-controls {
    display: flex;
    flex-direction: column; /* Botones de dirección en columna */
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
    transition: background 0.3s;
    min-width: 50px;
  
    /* Centrado con flexbox */
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
    transition: background 0.3s;
    min-width: 50px;
  
    /* Centrado con flexbox */
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
    transition: background 0.3s;
    min-width: 50px;
  
  }
  
  
  .control-button:hover {
    background: #3b82f6;
  }
  
  .slider {
    width: 100%; /* Ajusta el tamaño del slider */
    height: 10%; /* Ajusta el tamaño del slider */
    margin: 20px auto;
  }
  
  
  
  /* Etiquetas */
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
    border: 1px solid #3b82f6; /* Azul */
    border-radius: 8px;
    padding: 10px;
    font-size: 14px;
    outline: none;
    appearance: none;
  }
  
  .input-field:focus {
    border-color: #2563eb; /* Azul más fuerte */
  }
  
  .floating-label {
    position: absolute;
    left: 10px;
    top: 10px;
    font-size: 14px;
    color: #93c5fd; /* Azul claro */
    background-color: #222;
    padding: 0 5px;
    transition: all 0.3s ease-in-out;
  }
  
  .input-field:focus + .floating-label,
  .input-field:not(:placeholder-shown) + .floating-label {
    top: -5px;
    left: 12px;
    font-size: 12px;
    color: #60a5fa; /* Azul más oscuro */
  }
  
  .position-indicators, .orientation-indicator {
    margin-top: 15px;
  }
  #slider-value {
    font-size: 1.5rem;
    color: #ffffff;
    text-align: center;
  }
  
  /*Buton css*/ 
  
  .toggle-button {
    background-color: #3264fe;
    margin-top: 1rem;
  }
  
  .toggle-button-paint {
    background-color:#40AD55;
    margin-top: 1rem;
  }
  
  .toggle-button-paint.active {
    background-color: #D9534F; /* Rojo cuando está activado */
  }
  
  @media (max-width: 768px) {
    .control-panel {
      flex-direction: column;  /* En pantallas pequeñas, apilamos los elementos */
    }
  
    .control-content {
      flex-direction: column;  /* Botones y slider en columna en pantallas pequeñas */
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
