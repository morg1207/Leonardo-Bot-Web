
<script setup>

  import RobotConfigComponent from "./components/RobotConfigComponent.vue";
  import JoystickComponent from "./components/JoystickComponent.vue";
  import LogsComponent from "./components/LogsComponent.vue";
</script>

<template>
  <div id="app" class="app">
    <div class="grid-container">
      <!-- Bloque 1: Conexión -->
      <div class="grid-item">
        <div class="card">
          <RobotConfigComponent @my-event="connect_ros" @my-event-error-conection="error_conection" @my-event-disconnect="disconnect"></RobotConfigComponent>
        </div>
      </div>
      
      <!-- Bloque 2: Joystick -->
      <div class="grid-item">
        <div class="card center-content">
          <JoystickComponent ref="JoystickComponentRef" v-bind:usuario="usuario"></JoystickComponent>
        </div>
      </div>
      
      <!-- Bloque 3: Logs -->
      <div class="grid-item">
        <div class="card">
          <LogsComponent ref="LogsComponentRef" v:bind:usuario="usuario"></LogsComponent>
        </div>
      </div>
      
      <!-- Bloque 4: Otro contenido -->
      <div class="grid-item">
        <div class="card center-content">
          <p>Nuevo Contenido Aquí</p>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  name: "App",
  components: {

    RobotConfigComponent,
    JoystickComponent,
    LogsComponent,
  },
  data() {
    return {
      ros: null,
    };
  },
  methods: {

    connect_ros(ros,webpage_adress) {
      this.ros = ros;
      console.log("Evento de conexion recibido");

      this.$refs.JoystickComponentRef.joystickConfig(ros);
      this.$refs.LogsComponentRef.logsInit();
    },
    error_conection(){
      this.$refs.LogsComponentRef.logsErrorConection();
    },
    disconnect(){
      this.$refs.LogsComponentRef.logsDisconnect();
      //close button

      this.$refs.JoystickComponentRef.joystickClose();

    },
  },
};
</script>



<style lang="scss">
@import url('https://fonts.googleapis.com/css2?family=Raleway:wght@400;700&display=swap');

* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
  font-family: "Raleway", sans-serif;
}

.app {
  min-height: 100vh;
  display: flex;
  justify-content: center;
  align-items: center;
  background-color: #f1f1f1;
  padding: 20px;
}

.grid-container {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  grid-template-rows: repeat(2, 1fr);
  gap: 20px;
  width: 90%;
  max-width: 1000px;
}

.grid-item {
  display: flex;
  justify-content: center;
  align-items: center;
}

.card {
  width: 100%;
  height: 100%;
  background: white;
  padding: 20px;
  border-radius: 10px;
  box-shadow: 0px 4px 6px rgba(0, 0, 0, 0.1);
  text-align: center;
}

.center-content {
  display: flex;
  justify-content: center;
  align-items: center;
}

@media (max-width: 768px) {
  .grid-container {
    grid-template-columns: 1fr;
    grid-template-rows: auto;
  }
}
</style>
