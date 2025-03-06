<template>
  <div class="flex flex-col items-center w-screen p-4 bg-gray-100 min-h-screen overflow-x-hidden">
    <!-- Header -->
    <header class="header" @mouseover="showHint = true" @mouseleave="showHint = false">
      <div class="flex items-center justify-between w-full max-w-4xl px-4">
        <span class="text-xl font-semibold tracking-wide">LEONARDO BOT</span>
        <button @click="handleConnection" class="btn-connect">
          {{ ros ? 'Desconectar' : 'Conectar' }}
        </button>
      </div>
    </header>

    <div class="flex flex-col items-center w-full max-w-4xl mt-4"> 
      <transition name="slide">
        <div v-if="showModal && !isConnected" class="modal">
          <div class="modal-content">
            <RobotConfigComponent 
              @my-event="connect_ros" 
              @my-event-error-conection="error_conection" 
              @my-event-disconnect="disconnect"
            />
            <button @click="toggleModal" class="btn close">Cerrar</button>
          </div>
        </div>
      </transition>
    </div>
  </div>
</template>

<script>
import RobotConfigComponent from './RobotConfigComponent.vue';

export default {
  name: "Titulo",
  components: {
    RobotConfigComponent,
  },
  data() {
    return {
      showModal: false,
      isConnected: false,
      ros: null,
    };
  },

  methods: {
    toggleModal() {
      this.showModal = !this.showModal;
    },
    handleConnection() {
      if (this.ros) {
        console.log("Conectado");
        this.disconnect();
      } else {
        console.log("Desconectado");
        this.toggleModal();
      }
    },
    connect_ros(ros) {
      this.ros = ros;
      console.log("Titulo - Evento de conexión recibido");
      this.isConnected = true;
      this.toggleModal();

      this.$emit("my-event-titulo", this.ros);
    },
    error_conection(){
      console.log("Error de conexión");
    },
    disconnect(){
      console.log("Desconectado");
      this.isConnected = false;
      this.ros = null;
      
      this.$emit("my-event-disconnect", this.ros);
    },
  }
};
</script>

<style scoped>
@import url('https://fonts.googleapis.com/css2?family=Roboto:wght@700&display=swap');
@import "tailwindcss";

/* Estilos del header */
.header {
  width: 100%;
  height: 10%;
  background: linear-gradient(to right, #1E3A8A, #2563EB);
  color: white;
  padding: 1rem 2rem;
  text-align: center;
  font-size: 1.75rem;
  font-weight: 800;
  font-family: 'Roboto', sans-serif;
  box-shadow: 0px 4px 6px rgba(0, 0, 0, 0.1);
  cursor: pointer;
  z-index: 10;
}


.btn-connect {
  position: absolute;
  right: 1rem; /* Ajusta la separación del borde derecho */
  background: #ffffff;
  color: rgb(0, 0, 0);
  padding: 12px 24px;
  border: none;
  border-radius: 8px;
  font-weight: bold;
  cursor: pointer;
  transition: background 0.3s;
  font-size: 1rem;
  margin-left: auto;
}



.btn-connect:hover {
  background: #FBBF24;
}

/* Modal */
.modal {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background: rgba(0, 0, 0, 0.5);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 9999;
}

.modal-content {
  background: white;
  padding: 25px;
  border-radius: 12px;
  box-shadow: 0px 4px 6px rgba(0, 0, 0, 0.1);
  text-align: center;
  max-width: 400px;
  position: relative;
}

/* Botones generales */
.btn {
  margin: 10px;
  padding: 12px 24px;
  border: none;
  background: #2563EB;
  color: white;
  border-radius: 8px;
  cursor: pointer;
  transition: background 0.3s;
  font-size: 1rem;
}

.btn:hover {
  background: #3B82F6;
}

.btn.close {
  background: #555;
}

.btn.close:hover {
  background: #a30707;
}

/* Transiciones */
.slide-enter-active, .slide-leave-active {
  transition: transform 0.3s ease-in-out, opacity 0.3s;
}

.slide-enter-from, .slide-leave-to {
  transform: translateY(-20px);
  opacity: 0;
}
</style>
