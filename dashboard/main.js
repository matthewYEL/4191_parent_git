// main.js

import { connectToRobot, registerDataHandler } from './bluetooth.js';
import { initGrid, executeLogoCommands, clearTrail } from './grid.js';
import { openLastGradeModal, loadTemplate } from './grid.js';
import { TEMPLATES } from './templates.js';

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

  // Modal Handlers
  const hideModal = () => { if (gradeModal) gradeModal.style.display = 'none'; };

  if (closeModalBtn) closeModalBtn.addEventListener('click', hideModal);
  if (modalOkBtn) modalOkBtn.addEventListener('click', hideModal);
  if (viewGradeBtn) viewGradeBtn.addEventListener('click', openLastGradeModal);

  // 3. Single Bluetooth Connection Listener
  if (connectBtn) {
    connectBtn.addEventListener('click', async () => {
      // Prevent duplicate clicks while browser prompt is open
      if (connectBtn.disabled) return;

      console.log("👉 [MAIN] 'Connect to Robot' button clicked!");
      try {
        connectBtn.disabled = true;

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
        connectBtn.innerText = `Connected (${deviceName})`;

      } catch (error) {
        console.error("❌ [MAIN] Connection Error:", error.message);
        if (statusBadge) {
          statusBadge.innerText = `Status: ${error.message}`;
          statusBadge.className = "badge status-disconnected";
        }
        connectBtn.disabled = false; // Re-enable on error or cancellation
      }
    });
  } else {
    console.warn("⚠️ [MAIN] connectBtn element was not found in HTML.");
  }

 // 4. Handle Incoming BLE Data (PSoC / HC-05 Keyboard Commands -> Logo Box)
  registerDataHandler((incomingCommand) => {
    console.log("📩 [MAIN] Received Logo command over BLE:", incomingCommand);
    if (logoInput) {
      // Overwrite input box with only the latest incoming command
      logoInput.value = incomingCommand;
    }
  });

  // 5. PDF Save Listener
  if (pdfBtn) {
    pdfBtn.addEventListener('click', () => {
      console.log("👉 [MAIN] Triggering Save to PDF...");
      
      const containerEl = document.getElementById('gridContainer');
      const toggleEl = document.getElementById('showVowelsToggle');
      const canSeeVowels = toggleEl ? toggleEl.checked : true;

      console.log("🔍 [PDF Export] Show Vowels Toggle Checked:", canSeeVowels);

      // Store original text so we can restore it after printing
      const originalVowelTexts = [];

      if (!canSeeVowels && containerEl) {
        console.log("🙈 Hiding vowels for student PDF export...");
        containerEl.querySelectorAll('.grid-cell.cell-vowel').forEach(cell => {
          originalVowelTexts.push({ cell: cell, text: cell.innerText });
          cell.innerText = ''; // Wipe letter, keep yellow highlight
        });
      }

      // Trigger browser print dialog
      setTimeout(() => {
        window.print();

        // Restore vowel letters on screen after print dialog closes
        if (!canSeeVowels) {
          originalVowelTexts.forEach(item => {
            item.cell.innerText = item.text;
          });
        }
      }, 50);
    });
  }

  // 6. Run Logo Commands Listener
  if (runLogoBtn) {
    runLogoBtn.addEventListener('click', () => {
      const code = logoInput ? logoInput.value : '';
      if (!code) {
        alert("Please enter Logo commands (e.g., FD 5 RT 90 FD 3)");
        return;
      }
      executeLogoCommands(code);
    });
  }

  // 7. Reset Trail Listener
  if (resetTrailBtn) {
    resetTrailBtn.addEventListener('click', clearTrail);
  }

  // 8. Template Picker
  const templateSelect = document.getElementById('templateSelect');
  const loadTemplateBtn = document.getElementById('loadTemplateBtn');
  const templateInfo = document.getElementById('templateInfo');
  const fillSolutionBtn = document.getElementById('fillSolutionBtn');

  if (templateSelect) {
    TEMPLATES.forEach(t => {
      templateSelect.appendChild(new Option(t.robotOnly ? `${t.name} 🤖` : t.name, t.id));
    });

    const selectedTemplate = () => TEMPLATES.find(t => t.id === templateSelect.value);
    const showInfo = () => {
      const t = selectedTemplate();
      if (!t || !templateInfo) return;
      templateInfo.innerText = t.goal + (t.robotOnly
        ? '\n⚠️ Robot only: the simulator cannot grade repeat/while/ifelse.'
        : '');
    };
    templateSelect.addEventListener('change', showInfo);
    showInfo();

    if (loadTemplateBtn) {
      loadTemplateBtn.addEventListener('click', () => {
        const t = selectedTemplate();
        if (!t) return;
        loadTemplate(t);
        if (logoInput) logoInput.value = '';
      });
    }
    if (fillSolutionBtn) {
      fillSolutionBtn.addEventListener('click', () => {
        const t = selectedTemplate();
        if (t && logoInput) logoInput.value = t.solution;
      });
    }
  }
});

// Attach mutual exclusion logic for Include / Exclude checkboxes
document.querySelectorAll('.checkbox-row-wrapper input[type="checkbox"]').forEach(checkbox => {
  checkbox.addEventListener('change', (e) => {
    const currentBox = e.target;
    const command = currentBox.dataset.command;
    if (!command) return;

    // Find ALL checkboxes sharing this command (Include & Exclude)
    const matchingBoxes = document.querySelectorAll(`.checkbox-row-wrapper input[data-command="${command}"]`);

    matchingBoxes.forEach(box => {
      // Target the opposite box in the same row
      if (box !== currentBox) {
        // Disable opposite box if current box is checked; enable if unchecked
        box.disabled = currentBox.checked;

        // Toggle greyed-out visual class on parent label
        const parentLabel = box.closest('.print-checkbox-item');
        if (parentLabel) {
          if (currentBox.checked) {
            parentLabel.classList.add('disabled-item');
          } else {
            parentLabel.classList.remove('disabled-item');
          }
        }
      }
    });
  });
});