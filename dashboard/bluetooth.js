// bluetooth.js

const UART_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const UART_RX_CHARACTERISTIC = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
const UART_TX_CHARACTERISTIC = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';

let bluetoothDevice = null;
let gattServer = null;
let rxCharacteristic = null;
let txCharacteristic = null;

// Optional callback holder to handle incoming Logo strings in your main UI code
let onDataReceivedCallback = null;

/**
 * Registers a callback function to handle incoming Logo strings from PSoC via ESP32.
 * Usage in UI script: registerDataHandler((data) => { console.log("Received:", data); });
 */
export function registerDataHandler(callback) {
  onDataReceivedCallback = callback;
}

/**
 * Cleanly disconnects any existing BLE sessions.
 */
export async function disconnectRobot() {
  // Only try to stop notifications if the GATT server is still actively connected
  if (gattServer && gattServer.connected && txCharacteristic) {
    try {
      await txCharacteristic.stopNotifications();
    } catch (e) {
      console.warn("Error stopping notifications:", e);
    }
  }

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
  txCharacteristic = null;
}

/**
 * Connects to the ESP32 BLE Robot.
 */
export async function connectToRobot() {
  console.log("🔌 [BLE] Executing connectToRobot()...");
  
  // 1. Clean up existing session
  await disconnectRobot();

  try {
    console.log("🔌 [BLE] Triggering navigator.bluetooth.requestDevice()...");
    
    // 2. Request device prompt (acceptAllDevices ensures all nearby devices show up)
    bluetoothDevice = await navigator.bluetooth.requestDevice({
      acceptAllDevices: true,
      optionalServices: [UART_SERVICE_UUID]
    });

    console.log(`🔌 [BLE] Device selected: ${bluetoothDevice.name || 'Unnamed'}`);

    // 3. Handle unexpected disconnections
    bluetoothDevice.addEventListener('gattserverdisconnected', () => {
      console.warn("Robot disconnected unexpectedly.");
      disconnectRobot();
    });

    // 4. Connect GATT Server
    gattServer = await bluetoothDevice.gatt.connect();
    const service = await gattServer.getPrimaryService(UART_SERVICE_UUID);
    
    // 5. Get Characteristics
    rxCharacteristic = await service.getCharacteristic(UART_RX_CHARACTERISTIC);
    txCharacteristic = await service.getCharacteristic(UART_TX_CHARACTERISTIC);

    // 6. Enable Notifications to receive PSoC Logo strings
    await txCharacteristic.startNotifications();
    txCharacteristic.addEventListener('characteristicvaluechanged', (event) => {
      const decoder = new TextDecoder('utf-8');
      const receivedString = decoder.decode(event.target.value).trim();
      
      console.log(`📩 [BLE Received from ESP32]: ${receivedString}`);
      
      if (onDataReceivedCallback) {
        onDataReceivedCallback(receivedString);
      }
    });

    console.log("✅ [BLE] GATT setup complete and ready.");
    return bluetoothDevice.name || "Turtleeeee";

  } catch (error) {
    console.error("❌ [BLE] Failed during BLE sequence:", error);
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