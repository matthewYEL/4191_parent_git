// bluetooth.js

const UART_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const UART_RX_CHARACTERISTIC = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';

let bluetoothDevice = null;
let gattServer = null;
let rxCharacteristic = null;

/**
 * Cleanly disconnects any existing BLE sessions.
 */
export async function disconnectRobot() {
  if (gattServer && gattServer.connected) {
    try {
      gattServer.disconnect();
    } catch (e) {
      console.warn("Error during disconnect:", e);
    }
  }
  bluetoothDevice = null;
  gattServer = null;
  rxCharacteristic = null;
}

/**
 * Connects to the ESP32 BLE Robot.
 */
export async function connectToRobot() {
  console.log("🔌 [BLE] Executing connectToRobot()...");
  // 1. Force cleanup of any lingering previous connection
  await disconnectRobot();

  try {
    console.log("🔌 [BLE] Triggering navigator.bluetooth.requestDevice()...");
    // 2. Request device popup
    bluetoothDevice = await navigator.bluetooth.requestDevice({
      acceptAllDevices: true,
      optionalServices: [UART_SERVICE_UUID]
    });

    console.log(`🔌 [BLE] Device selected: ${bluetoothDevice.name || 'Unnamed'}`);

    // 3. Listen for unexpected disconnections
    bluetoothDevice.addEventListener('gattserverdisconnected', () => {
      console.warn("Robot disconnected unexpectedly.");
      disconnectRobot();
    });

    // 4. Establish GATT connection
    gattServer = await bluetoothDevice.gatt.connect();
    const service = await gattServer.getPrimaryService(UART_SERVICE_UUID);
    rxCharacteristic = await service.getCharacteristic(UART_RX_CHARACTERISTIC);
    console.log("✅ [BLE] GATT setup complete and ready to send data.");

    return bluetoothDevice.name || "Turtle Robot";
  } catch (error) {
    console.error("❌ [BLE] Failed during BLE sequence:", error);
    // Ensure clean state if user cancels popup or connection times out
    await disconnectRobot();
    throw new Error(error.message || "Connection failed or canceled.");
  }
}

export async function sendData(dataString) {
  if (!rxCharacteristic) {
    throw new Error("No active Bluetooth connection!");
  }
  const encoder = new TextEncoder();
  await rxCharacteristic.writeValue(encoder.encode(dataString));
}

export function isConnected() {
  return gattServer && gattServer.connected;
}