<template>
  <div class="card mb-3 w-full p-6 bg-white rounded-lg shadow-md">
    <div class="card-body">
      <!-- Estado de conexión -->
      <p class="status-text" :class="{ 'text-red-500': !connected, 'text-green-500': connected }">
        {{ connected ? 'Conectado!' : 'No conectado!' }}
      </p>

      <!-- Campo de entrada para la IP -->
      <div class="form-group mt-4">
        <label for="rosbridge-address" class="form-label">IP robot</label>
        <input
          id="ip_robot"
          type="text"
          v-model="ip_robot"
          class="form-input"
          placeholder="Ingrese la IP de su Leonard-Bot"
          @input="validateIp"
        />
        <!-- Mensaje de error si la IP no es válida -->
        <p v-if="!isIpValid && ip_robot" class="text-red-500 text-sm mt-1">
          Ingrese una IP válida (ejemplo: 192.168.1.18)
        </p>
      </div>

      <!-- Botones de conexión/desconexión -->
      <div class="mt-6">
        <button
          :disabled="loading || !isIpValid"
          class="btn btn-danger"
          @click="disconnect"
          v-if="connected"
        >
          Disconnect
        </button>
        <button
          :disabled="loading || !isIpValid"
          class="btn btn-success"
          @click="connect"
          v-else
        >
          Connect
        </button>
      </div>
    </div>
  </div>
</template>
<script>
import ROSLIB from "roslib";

export default {
  name: "RobotConfigComponent",

  data() {
    return {
      ros: null,
      connected: false,
      isIpValid: false, // Estado de validación de la IP
      loading: false,
      ip_robot: "",
      port: "9090",
      rosbridge_address: "",
    };
  },

  methods: {
    // Valida la IP ingresada
    validateIp() {
      const ipPattern = /^(\d{1,3}\.){3}\d{1,3}$/; // Expresión regular para validar IP
      this.isIpValid = ipPattern.test(this.ip_robot); // Actualiza el estado de validación
    },

    connect() {
      this.loading = true;
      this.rosbridge_address = "ws://" + this.ip_robot + ":9090"
      this.ros = new ROSLIB.Ros({
        url: this.rosbridge_address,
      });
      console.log("------------------------------");
      console.log("Conectando a rosbridge address");
      console.log(this.rosbridge_address);
      this.ros.on("connection", () => {
        console.log("RB- Conectado a ROS");
        this.connected = true;
        this.loading = false;
        this.$emit("my-event", this.ros);

      });
      this.ros.on("error", (error) => {
        this.loading = false;
        console.log("Error de conexión");
        this.$emit("my-event-error-conection", this.ros);
      });
      this.ros.on("close", () => {
        this.connected = false;
        this.loading = false;
        this.ros = null;
        console.log("RB - Desconectado");
        this.$emit("my-event-disconnect", this.ros);
      });
    },

    disconnect() {
    console.log("Desconectando de ROS");
    if (this.ros) {
      console.log("Desconectado");
      this.ros.close();
      console.log("1");
      this.connected = false;
      this.loading = false;
      this.ros = null;
      this.$emit("my-event-disconnect", this.ros); // Emitir evento de desconexión


    }
    },
  
  },
};
</script>

<style scoped>
@import url('https://fonts.googleapis.com/css2?family=Roboto:wght@700&display=swap');
@import "tailwindcss";
.card {
  background: white;
  border-radius: 12px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  padding: 1.5rem;
  width: 100%;
}

.card-body {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.status-text {
  font-weight: 600;
  font-size: 1rem;
}

.text-red-500 {
  color: #ef4444;
}

.text-green-500 {
  color: #22c55e;
}

.form-group {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.form-label {
  font-size: 0.875rem;
  font-weight: 500;
  color: #374151;
}

.form-input {
  padding: 0.5rem 1rem;
  border: 1px solid #d1d5db;
  border-radius: 8px;
  font-size: 0.875rem;
  outline: none;
  transition: border-color 0.3s ease;
}

.form-input:focus {
  border-color: #3b82f6;
  box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.1);
}

.btn {
  padding: 0.5rem 1rem;
  border: none;
  border-radius: 8px;
  font-weight: 600;
  font-size: 0.875rem;
  cursor: pointer;
  transition: background-color 0.3s ease;
}

.btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.btn-danger {
  background-color: #ef4444;
  color: white;
}

.btn-danger:hover {
  background-color: #dc2626;
}

.btn-success {
  background-color: #22c55e;
  color: white;
}

.btn-success:hover {
  background-color: #16a34a;
}
</style>