// main.js

import { connectToRobot } from './bluetooth.js';
import { initGrid, executeLogoCommands, clearTrail} from './grid.js';
import { openLastGradeModal } from './grid.js';

document.addEventListener('DOMContentLoaded', () => {
  console.log("🚀 [MAIN] Starting single-page dashboard...");

  // 1. Draw the 20x20 grid immediately on page load
  initGrid();

  // 2. Select DOM Elements
  const connectBtn = document.getElementById('connectBtn');
  const pdfBtn = document.getElementById('pdfBtn');
  const statusBadge = document.getElementById('statusBadge');

  const runLogoBtn = document.getElementById('runLogoBtn');
  const resetTrailBtn = document.getElementById('resetTrailBtn');
  const logoInput = document.getElementById('logoInput');

  const closeModalBtn = document.getElementById('closeModalBtn');
  const modalOkBtn = document.getElementById('modalOkBtn');
  const gradeModal = document.getElementById('gradeModal');
  const viewGradeBtn = document.getElementById('viewGradeBtn');

  const hideModal = () => { if (gradeModal) gradeModal.style.display = 'none'; };

  if (closeModalBtn) closeModalBtn.addEventListener('click', hideModal);
  if (modalOkBtn) modalOkBtn.addEventListener('click', hideModal);
  if (viewGradeBtn) viewGradeBtn.addEventListener('click', openLastGradeModal);

  // 3. Bluetooth Connection Listener
  connectBtn.addEventListener('click', async () => {
    console.log("👉 [MAIN] 'Connect to Robot' button clicked!");
    try {
      statusBadge.innerText = `Searching for Robot...`;
      statusBadge.className = "badge status-connecting";
      console.log("👉 [MAIN] Requesting BLE device popup...");

      const deviceName = await connectToRobot();

      console.log(`✅ [MAIN] Connected successfully to: ${deviceName}`);

      // Update UI on success
      statusBadge.innerText = `Status: Connected to ${deviceName}`;
      statusBadge.className = "badge status-connected";
      connectBtn.disabled = true;
      connectBtn.innerText = `Connected (${deviceName})`;

    } catch (error) {
      console.error("❌ [MAIN] Connection Error:", error.message);
      statusBadge.innerText = `Status: ${error.message}`;
      statusBadge.className = "badge status-disconnected";
    }
  });

  // 4. PDF Save Listener
  if (pdfBtn) {
    pdfBtn.addEventListener('click', () => {
      console.log("👉 [MAIN] Triggering Save to PDF...");
      window.print();
    });
  }

  if (connectBtn) {
    connectBtn.addEventListener('click', async () => {
      console.log("👉 [MAIN] 'Connect to Robot' button clicked!");
      try {
        if (statusBadge) {
          statusBadge.innerText = `Searching for Robot...`;
          statusBadge.className = "badge status-connecting";
        }
        console.log("👉 [MAIN] Requesting BLE device popup...");

        const deviceName = await connectToRobot();

        console.log(`✅ [MAIN] Connected successfully to: ${deviceName}`);

        if (statusBadge) {
          statusBadge.innerText = `Status: Connected to ${deviceName}`;
          statusBadge.className = "badge status-connected";
        }
        connectBtn.disabled = true;
        connectBtn.innerText = `Connected (${deviceName})`;

      } catch (error) {
        console.error("❌ [MAIN] Connection Error:", error.message);
        if (statusBadge) {
          statusBadge.innerText = `Status: ${error.message}`;
          statusBadge.className = "badge status-disconnected";
        }
      }
    });
  } else {
    console.warn("⚠️ [MAIN] connectBtn element was not found in HTML.");
  }

  if (runLogoBtn) {
    runLogoBtn.addEventListener('click', () => {
      const code = logoInput.value;
      if (!code) {
        alert("Please enter Logo commands (e.g., FD 5 RT 90 FD 3)");
        return;
      }
      executeLogoCommands(code);
    });
  }

  if (resetTrailBtn) {
    resetTrailBtn.addEventListener('click', clearTrail);
  }

});