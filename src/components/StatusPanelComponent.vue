<template>
  <div class="status-panel">
    <!-- Título de la sección -->
    <h2 class="title text-center text-2xl font-bold text-blue-400 mb-4">Perfil del Robot</h2>

    <!-- Indicador de Voltaje -->
    <div class="voltage-indicator mb-6 p-4 bg-gray-700 rounded-xl shadow-lg">
      <span class="label">ID Robot</span>
      <div class="flex items-center justify-center">
        <div class="w-full h-12 bg-gray-600 rounded-full relative">
          <div class="absolute inset-0 flex items-center justify-center">
            <span class="text-white text-lg font-bold">Leonardo-v01</span>
          </div>
        </div>
      </div>
    </div>

    <!-- Indicador de Voltaje -->
    <div class="voltage-indicator mb-6 p-4 bg-gray-700 rounded-xl shadow-lg">
      <span class="label">Voltaje de la Batería</span>
      <div class="flex items-center justify-center">
        <div class="w-full h-12 bg-gray-600 rounded-full relative">
          <div class="absolute inset-0 flex items-center justify-center">
            <span class="text-white text-lg font-bold">{{ formattedVoltage }}V</span>
          </div>
        </div>
      </div>
    </div>

    <!-- Indicador de Cantidad de Batería -->
    <div class="battery-indicator mb-6 p-4 bg-gray-700 rounded-xl shadow-lg">
      <span class="label">Cantidad de Batería</span>
      <div class="flex flex-wrap justify-center">
        <div class="w-full">
          <div class="shadow w-full rounded border-2 border-gray-400 flex my-1 relative">
            
            <div class="border-r-8 h-10 rounded-r absolute right-0 top-0 flex border-gray-400 z-10"></div>
            <div
              class="cursor-default text-xs font-bold leading-none flex items-center justify-center m-1 py-4 text-center text-white"
              :class="batteryClass"
              :style="{ width: batteryWidth }"
            >
              <div class="absolute left-0 mx-8 text-gray-700">{{ batteryPercentage }}%</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- Datos de Posición con Floating Labels -->
    <div class="position-indicators p-4 bg-gray-700 rounded-xl shadow-lg">
      <span class="label">Posición (m) :</span>
      <div class="flex gap-4 mt-4">
        <div class="relative w-32">
          <input type="text" id="pos_x" class="input-field" :value="position.x" readonly />
          <label for="pos_x" class="floating-label">X </label>
        </div>
        <div class="relative w-32">
          <input type="text" id="pos_y" class="input-field" :value="position.y" readonly />
          <label for="pos_y" class="floating-label">Y</label>
        </div>
        <div class="relative w-32">
          <input type="text" id="pos_z" class="input-field" :value="position.z" readonly />
          <label for="pos_z" class="floating-label">Z</label>
        </div>
      </div>
      <span class="label mt-4 ">Orientación ( ° ) :</span>
      <div class="flex gap-4 mt-4 ">
        <div class="relative w-32">
          <input type="text" id="orientation_x" class="input-field" :value="orientation.x" readonly />
          <label for="orientation_x" class="floating-label">Yaw</label>
        </div>
      </div>
    </div>

    <!-- Orientación del Robot -->
    <div class="orientation-indicator p-2 bg-gray-700 rounded-xl shadow-lg mt-6">
      <span class="label">Velocidad:</span>
      <div class="flex gap-4 mt-4">
        <div class="relative w-32">
          <input type="text" id="vel_x" class="input-field" :value="vel.x" readonly />
          <label for="vel_x" class="floating-label">X (m/s)</label>
        </div>
        <div class="relative w-32">
          <input type="text" id="vel_z" class="input-field" :value="vel.z" readonly />
          <label for="vel_z" class="floating-label">Z (rad/s)</label>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  name: "StatusPanelComponent",
  data() {
    return {
      voltage: 12.6, // Voltaje de la batería
      batteryLevel: 9, // Nivel de batería de 0 a 10
      position: {
        x: 3.4,
        y: 5.2,
        z: 1.1,
      },
      orientation: {
        x: 0.0,
      },
      vel: {
        x: 0.0,
        z: 0.0,
      }
    };
  },
  computed: {
    formattedVoltage() {
      return this.voltage.toFixed(1);
    },
    batteryPercentage() {
      return this.batteryLevel * 10;
    },
    batteryWidth() {
      return `${this.batteryPercentage}%`;
    },
    batteryClass() {
      if (this.batteryLevel >= 8) return 'bg-green-400';
      if (this.batteryLevel >= 5) return 'bg-yellow-400';
      if (this.batteryLevel >= 3) return 'bg-gray-400';
      return 'bg-red-400';
    },
  },
};
</script>

<style scoped>
@import url('https://fonts.googleapis.com/css2?family=Roboto:wght@700&display=swap');
@import "tailwindcss";

.status-panel {
  background-color: #222;
  color: white;
  padding: 2rem;
  border-radius: 10px;
  width: 100%; /* Hacer que el panel ocupe todo el espacio disponible */
  margin: 0 auto; /* Centrar en la pantalla */
  align-items: top;
}

/* Título */
.title {
  font-size: 24px;
  color: #60a5fa; /* Color azul más oscuro */
}

/* Etiquetas */
.label {
  font-weight: bold;
  margin-bottom: 10px;
  display: block;
  color: #93c5fd;
  font-size: 16px;
}

/* Indicador de Voltaje */
.voltage-indicator .label {
  color: #60a5fa; /* Color azul más oscuro */
}

/* Indicador de Batería */
.battery-indicator .label {
  color: #60a5fa; /* Color azul más oscuro */
}

/* Floating Input Fields */
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

/* Responsividad para pantallas pequeñas */
@media (max-width: 768px) {
  .status-panel {
    width: 95%;
  }
  .input-field {
    font-size: 12px;
  }
}
</style>
