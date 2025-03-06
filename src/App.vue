<template>
  <!-- Franja superior fija -->
  <div :style="headerStyle">
    <Titulo @my-event-titulo="connect_ros" @my-event-disconnect="disconnect_ros" />
  </div>

  <!-- Contenedor principal debajo del título -->
  <div :style="mainContainerStyle">
    <!-- Contenedor responsivo de botones alineados horizontalmente -->
    <div :style="buttonContainerStyle">
      <!-- Primer botón -->
      <div :style="buttonWrapperStyle">
        <ButtonControlComponent ref="ButtonControlComponentRef1" style="width: 100%; height: 100%;" v-if="isConnected"/>
      </div>
      
      <!-- Segundo botón (StatusPanel en lugar del segundo botón) -->
      <div :style="buttonWrapperStyle">
        <StatusPanelComponent v-if="isConnected"/>
      </div>
      
      <!-- Tercer botón -->
      <div :style="buttonWrapperStyle">
        <MapComponent
            ref="MapComponentRef"
            v:bind:usuario="usuario"
            v-if="isConnected"/>
      </div>
    </div>

    <!-- Sección de expansión del control manual (solo para cuando esté activado) -->
    <div v-if="isControlExpanded" :style="expandedControlStyle">
      <!-- Aquí va el contenido adicional que se expande -->
    </div>
  </div>
</template>

<script>
import Titulo from "./components/Titulo.vue";
import ButtonControlComponent from "./components/ButtonControlComponent.vue";
import StatusPanelComponent from "./components/StatusPanelComponent.vue";
import MapComponent from "./components/MapComponent.vue";

export default {
  name: "App",
  components: {
    Titulo,
    ButtonControlComponent,
    StatusPanelComponent,
    MapComponent

  },
  data() {
    return {
      ros: null,
      isConnected: false,
      isControlExpanded: false, // Indica si el control está expandido
    };
  },
  methods: {
    connect_ros(ros) {
      this.isConnected = true;
      this.ros = ros;
      console.log("App - Conexión ROS establecida");

      this.$nextTick(() => {
        if (this.$refs.ButtonControlComponentRef1 && this.$refs.ButtonControlComponentRef2 && this.$refs.ButtonControlComponentRef3) {
          this.$refs.ButtonControlComponentRef1.init(this.ros);
        }
        this.$refs.MapComponentRef.setMapViewer(this.ros);
        this.$refs.MapComponentRef.initCamera(this.ros);
        this.$refs.ButtonControlComponentRef1.init(this.ros);
      });

    },
    disconnect_ros() {
      this.isConnected = false;
      this.ros = null;
      console.log("App - Conexión ROS cerrada");
    },
    toggleControl() {
      this.isControlExpanded = !this.isControlExpanded;
    },
  },
  computed: {
    headerStyle() {
      return {
        top: "0",
        left: "0",
        width: "100vw",  // Asegura que el header ocupe todo el ancho de la pantalla
        height: "55px",
        zIndex: 1000,
        backgroundColor: "#222",  // Solo para mostrar un fondo distinto
        boxShadow: "0 4px 6px rgba(0,0,0,0.1)",
        pointerEvents: "auto",
        boxSizing: "border-box",  // Incluir bordes y padding en el cálculo del ancho
      };
    },
    mainContainerStyle() {
      return {
        marginTop: "24px",
        width: "100vw",  // Asegura que el contenedor principal ocupe todo el ancho de la pantalla
        backgroundColor: "#343434",
        padding: "1rem",
        position: "relative",
        zIndex: 10,
        overflowX: "hidden", // Evita que haya scroll horizontal
        boxSizing: "border-box",  // Incluir bordes y padding en el cálculo del ancho
      };
    },
    buttonContainerStyle() {
      return {
        display: "flex",
        flexDirection: "row",  // Alineación horizontal
        justifyContent: "center", // Centra los elementos
        gap: "1rem",
        width: "100%",
        flexWrap: "wrap", // Permite que los elementos se acomoden en varias filas si es necesario
        position: "relative",
        zIndex: 500,
        boxSizing: "border-box",
      };
    },
    buttonWrapperStyle() {
      return {
        flex: "1 1 30%",
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        minWidth: "200px", 
        position: "relative",
        zIndex: 500,
        pointerEvents: "auto",
        boxSizing: "border-box",
        alignItems: "top",
      };
    },
    expandedControlStyle() {
      return {
        marginTop: "1rem",
        width: "100%",
        padding: "1rem",
        backgroundColor: "#343434",  // Solo para mostrar un fondo distinto
        boxSizing: "border-box",
      };
    },
  },
};
</script>

<style scoped>
* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
  background-color: #343434; /* Color oscuro */
}

body {
  overflow-x: hidden;  
  font-family: Arial, sans-serif;
  background-color: #343434; /* Color oscuro */

}

#app {
  width: 100vw;  
}
</style>
