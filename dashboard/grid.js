// grid.js
import { sendData } from './bluetooth.js';

const COLS = ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T'];
const ROWS = 20;

// Board State Tracking
let boardState = {}; // Format: { "B14": { type: "start", symbol: "🚀" } }
let startCoord = null;
let destCoord = null;
let hoveredCellKey = null;
let container = null;

// DOM Element References
let coordInput, itemSelect, placeBtn, clearAllBtn, pdfBtn;
let clearStartBtn, clearDestBtn, clearObsBtn, clearVowelBtn;

let robotState = {
  colIdx: 0,   // 0 = Col A, 19 = Col T
  rowIdx: 0,   // 0 = Row 1, 19 = Row 20
  heading: 0   // 0 = UP (North), 90 = RIGHT (East), 180 = DOWN (South), 270 = LEFT (West)
};

let startDir = 0; // Default North (0=North, 90=East, 180=South, 270=West)
let startDirSelect;

// State for store and re-viewing the latest score
let lastEvaluationResult = null;

// Inside step execution loop in grid.js
let nextCol = robotState.colIdx;
let nextRow = robotState.rowIdx;

export function initGrid() {
  container = document.getElementById('gridContainer');
  if (!container) {
    console.error("❌ [GRID] Could not find #gridContainer in HTML!");
    return;
  }

  // 1. Grab UI Elements inside initGrid
  coordInput = document.getElementById('coordInput');
  itemSelect = document.getElementById('itemTypeSelect');
  placeBtn = document.getElementById('placeBtn');
  clearAllBtn = document.getElementById('clearAllBtn');
  pdfBtn = document.getElementById('pdfBtn');

  clearStartBtn = document.getElementById('clearStartBtn');
  clearDestBtn = document.getElementById('clearDestBtn');
  clearObsBtn = document.getElementById('clearObsBtn');
  clearVowelBtn = document.getElementById('clearVowelBtn');

  container.innerHTML = ''; 

  // 2. TOP ROW LABELS: Corner | A-T | Corner
  createLabelCell(container, '');
  COLS.forEach(col => createLabelCell(container, col));
  createLabelCell(container, '');

  // 3. MIDDLE ROWS: Left Label (1-20) | Grid Cells | Right Label (1-20)
  for (let r = 1; r <= ROWS; r++) {
    createLabelCell(container, r);

    COLS.forEach(col => {
      const cell = document.createElement('div');
      cell.className = 'grid-cell';
      cell.dataset.col = col;
      cell.dataset.row = r;

      cell.addEventListener('click', () => {
        if (coordInput) coordInput.value = `${col}${r}`;
      });

      cell.addEventListener('mouseenter', () => {
        hoveredCellKey = `${col}${r}`;
      });

      cell.addEventListener('mouseleave', () => {
        if (hoveredCellKey === `${col}${r}`) hoveredCellKey = null;
      });

      container.appendChild(cell);
    });

    createLabelCell(container, r);
  }

  // 4. BOTTOM ROW LABELS: Corner | A-T | Corner
  createLabelCell(container, '');
  COLS.forEach(col => createLabelCell(container, col));
  createLabelCell(container, '');

  // 5. Attach Control Listeners
  if (placeBtn) placeBtn.addEventListener('click', parseAndPlace);
  if (clearStartBtn) clearStartBtn.addEventListener('click', () => clearCategory('start'));
  if (clearDestBtn) clearDestBtn.addEventListener('click', () => clearCategory('dest'));
  if (clearObsBtn) clearObsBtn.addEventListener('click', () => clearCategory('obstacle'));
  if (clearVowelBtn) clearVowelBtn.addEventListener('click', () => clearCategory('vowels'));
  if (clearAllBtn) clearAllBtn.addEventListener('click', clearEntireGrid);

  if (coordInput) {
    coordInput.addEventListener('keypress', (e) => {
      if (e.key === 'Enter') parseAndPlace();
    });
  }

  startDirSelect = document.getElementById('startDirSelect');
  if (startDirSelect) {
    startDirSelect.addEventListener('change', (e) => {
      startDir = parseInt(e.target.value, 10);
      if (startCoord) syncRobotToStart(); // Update turtle direction immediately if placed
    });
  }

  updateUI();
}

//robot onscreen move
export function syncRobotToStart() {
  if (!startCoord) return;
  
  // Convert "B14" -> colIdx (1) and rowIdx (13)
  const colLetter = startCoord[0];
  const rowNum = parseInt(startCoord.slice(1), 10);

  robotState.colIdx = COLS.indexOf(colLetter);
  robotState.rowIdx = rowNum - 1;
  robotState.heading = startDir; // Uses selected starting direction!
  
  highlightCurrentRobotCell(robotState.colIdx, robotState.rowIdx);
}

export async function executeLogoCommands(commandString) {
  if (!startCoord) {
    alert("Please place a Start point (🚀) on the grid first!");
    return;
  }

  clearTrail();
  syncRobotToStart();

  // Create clean tracker object for this run
  const runTracker = {
    hitBorder: false,
    reachedDestination: false,
    hitObstacles: new Set(),
    visitedVowels: new Set(),
    completedVowelActions: new Set()
  };

  const tokens = commandString.trim().toUpperCase().split(/\s+/);
  let i = 0;

  while (i < tokens.length) {
    const cmd = tokens[i];
    const val = parseInt(tokens[i + 1], 10) || 0;

    if (cmd === 'FD' || cmd === 'FORWARD') {
      const hitBorder = await moveRobotForward(val, runTracker);
      if (hitBorder) break; // Stop loop on border crash
      i += 2;
    } else if (cmd === 'BK' || cmd === 'BACKWARD') {
      const hitBorder = await moveRobotForward(-val, runTracker);
      if (hitBorder) break;
      i += 2;
    } else if (cmd === 'RT' || cmd === 'RIGHT') {
      robotState.heading = (robotState.heading + val) % 360;
      highlightCurrentRobotCell(robotState.colIdx, robotState.rowIdx);
      i += 2;
    } else if (cmd === 'LT' || cmd === 'LEFT') {
      robotState.heading = (robotState.heading - val + 360) % 360;
      highlightCurrentRobotCell(robotState.colIdx, robotState.rowIdx);
      i += 2;
    } else {
      i++;
    }
  }

  // Check if final step stopped on Destination
  const finalKey = `${COLS[robotState.colIdx]}${robotState.rowIdx + 1}`;
  if (destCoord && finalKey === destCoord) {
    runTracker.reachedDestination = true;
  }

  // Evaluate final score & pop modal
  lastEvaluationResult = evaluateMissionRun(runTracker);
  showGradeModal(lastEvaluationResult);
  updateGradeBanner(lastEvaluationResult);
}

async function moveRobotForward(steps, runTracker) {
  const dir = steps >= 0 ? 1 : -1;
  const count = Math.abs(steps);

  for (let step = 0; step < count; step++) {
    markCellTrail(robotState.colIdx, robotState.rowIdx);

    let nextCol = robotState.colIdx;
    let nextRow = robotState.rowIdx;

    // 1. Calculate prospective cell without mutating state prematurely
    if (robotState.heading === 0) nextRow -= dir;       // UP
    else if (robotState.heading === 90) nextCol += dir;  // RIGHT
    else if (robotState.heading === 180) nextRow += dir; // DOWN
    else if (robotState.heading === 270) nextCol -= dir; // LEFT

    // 2. BORDER CRASH CHECK (Grid range 0 to 19)
    if (nextCol < 0 || nextCol > 19 || nextRow < 0 || nextRow > 19) {
      console.warn("🚨 BORDER CRASH DETECTED!");
      if (runTracker) runTracker.hitBorder = true;
      
      const currentCell = container.querySelector(`[data-col="${COLS[robotState.colIdx]}"][data-row="${robotState.rowIdx + 1}"]`);
      if (currentCell) currentCell.style.backgroundColor = '#ef4444';
      return true; // Signal immediately that a crash occurred!
    }

    // 3. Commit new valid position
    robotState.colIdx = nextCol;
    robotState.rowIdx = nextRow;

    // 4. Track cell interactions
    const currentKey = `${COLS[robotState.colIdx]}${robotState.rowIdx + 1}`;
    if (boardState[currentKey] && runTracker) {
      const cellType = boardState[currentKey].type;
      if (cellType === 'obstacle') runTracker.hitObstacles.add(currentKey);
      if (['A','E','I','O','U'].includes(cellType)) runTracker.visitedVowels.add(cellType);
    }

    highlightCurrentRobotCell(robotState.colIdx, robotState.rowIdx);
    await new Promise(resolve => setTimeout(resolve, 250));
  }
  return false;
}

function markCellTrail(colIdx, rowIdx) {
  const colLetter = COLS[colIdx];
  const rowNum = rowIdx + 1;
  const cell = container.querySelector(`[data-col="${colLetter}"][data-row="${rowNum}"]`);
  if (cell) {
    cell.classList.add('path-trail');
  }
}

function highlightCurrentRobotCell(colIdx, rowIdx) {
  // 1. Clear previous robot highlight from former cell
  container.querySelectorAll('.robot-current').forEach(cell => {
    cell.classList.remove('robot-current');
    
    // Restore underlying symbol (🚀, 🏁, 🧱, A-U) or leave blank
    const key = `${cell.dataset.col}${cell.dataset.row}`;
    if (boardState[key]) {
      cell.innerText = boardState[key].symbol;
    } else {
      cell.innerText = '';
    }
  });

  // 2. Locate active cell
  const colLetter = COLS[colIdx];
  const rowNum = rowIdx + 1;
  const cell = container.querySelector(`[data-col="${colLetter}"][data-row="${rowNum}"]`);
  
  if (cell) {
    cell.classList.add('robot-current');
    cell.classList.add('path-trail');

    // 3. Map heading angle to compass facing class
    // Default 🐢 faces LEFT (270°)
    let facingClass = 'turtle-facing-left';
    if (robotState.heading === 0) facingClass = 'turtle-facing-up';
    else if (robotState.heading === 90) facingClass = 'turtle-facing-right';
    else if (robotState.heading === 180) facingClass = 'turtle-facing-down';

    // Inject scaled & rotated turtle element
    cell.innerHTML = `<span class="turtle-icon ${facingClass}">🐢</span>`;
  }
}

export function clearTrail() {
  container.querySelectorAll('.grid-cell').forEach(cell => {
    cell.classList.remove('path-trail', 'robot-current');
    
    // Restore underlying placed items (🚀, 🏁, 🧱, A-U) or leave blank
    const key = `${cell.dataset.col}${cell.dataset.row}`;
    if (boardState[key]) {
      cell.innerText = boardState[key].symbol;
    } else {
      cell.innerText = '';
    }
  });
}

// Global Keyboard Delete Listener
document.addEventListener('keydown', (e) => {
  if ((e.key === 'Delete' || e.key === 'Backspace') && hoveredCellKey && document.activeElement !== coordInput) {
    if (boardState[hoveredCellKey]) {
      removeCoordinate(hoveredCellKey);
      updateUI();
    }
  }
});

function parseAndPlace() {
  if (!coordInput || !itemSelect) return;

  const raw = coordInput.value.trim().toUpperCase();
  const match = raw.match(/^([A-T])([1-9]|1[0-9]|20)$/);
  if (!match) {
    alert("Please enter a valid coordinate from A1 to T20.");
    return;
  }

  const col = match[1];
  const row = parseInt(match[2], 10);
  const key = `${col}${row}`;
  const selectedType = itemSelect.value;

  if (boardState[key]) {
    removeCoordinate(key);
  }

  // Enforce Single-Instance placement rules
  if (selectedType === 'start' && startCoord) removeCoordinate(startCoord);
  if (selectedType === 'dest' && destCoord) removeCoordinate(destCoord);

  if (['A', 'E', 'I', 'O', 'U'].includes(selectedType)) {
    const existingVowelKey = Object.keys(boardState).find(
      k => boardState[k].type === selectedType
    );
    if (existingVowelKey) removeCoordinate(existingVowelKey);
  }

  // Get Visual Config
  const config = getItemConfig(selectedType);

  // Render on Grid DOM Cell
  const cell = container.querySelector(`[data-col="${col}"][data-row="${row}"]`);
  if (cell) {
    cell.innerText = config.symbol;
    cell.className = `grid-cell ${config.className}`;
    boardState[key] = { type: selectedType, symbol: config.symbol };
  }

  // Update Core State Trackers
  if (selectedType === 'start') {
    startCoord = key;
  } else if (selectedType === 'dest') {
    destCoord = key;
  }

  updateUI();
}

function clearCategory(category) {
  Object.keys(boardState).forEach(coord => {
    const type = boardState[coord].type;
    if (
      (category === 'start' && type === 'start') ||
      (category === 'dest' && type === 'dest') ||
      (category === 'obstacle' && type === 'obstacle') ||
      (category === 'vowels' && ['A','E','I','O','U'].includes(type))
    ) {
      removeCoordinate(coord);
    }
  });

  if (category === 'start') startCoord = null;
  if (category === 'dest') destCoord = null;

  updateUI();
}

function removeCoordinate(key) {
  if (!boardState[key]) return;

  if (key === startCoord) startCoord = null;
  if (key === destCoord) destCoord = null;

  delete boardState[key];

  const cell = container.querySelector(`[data-col="${key[0]}"][data-row="${key.slice(1)}"]`);
  if (cell) {
    cell.innerText = '';
    cell.className = 'grid-cell';
  }
}

function clearEntireGrid() {
  Object.keys(boardState).forEach(coord => removeCoordinate(coord));
  startCoord = null;
  destCoord = null;
  updateUI();
}

// Inside updateUI() in grid.js

function updateUI() {
  const startDisplay = document.getElementById('startDisplay');
  const destDisplay = document.getElementById('destDisplay');
  const obsCount = document.getElementById('obsCount');
  const obsDisplay = document.getElementById('obsDisplay');
  const vowelCount = document.getElementById('vowelCount');
  const vowelDisplay = document.getElementById('vowelDisplay');
  const pdfBtn = document.getElementById('pdfBtn');

  // 1. Update text displays
  if (startDisplay) startDisplay.innerText = startCoord || 'Not Set';
  if (destDisplay) destDisplay.innerText = destCoord || 'Not Set';

  if (startDisplay && startCoord) {
    const dirMap = { 0: 'N ⬆️', 90: 'E ➡️', 180: 'S ⬇️', 270: 'W ⬅️' };
    startDisplay.innerText = `${startCoord} (${dirMap[startDir]})`;
  }

  // 2. Aggregate Obstacles
  const obstacles = Object.keys(boardState).filter(k => boardState[k].type === 'obstacle');
  if (obsCount) obsCount.innerText = obstacles.length;
  if (obsDisplay) obsDisplay.innerText = obstacles.length > 0 ? obstacles.join(', ') : 'None';

  // 3. Aggregate Vowels
  const vowels = Object.keys(boardState)
    .filter(k => ['A','E','I','O','U'].includes(boardState[k].type))
    .map(k => `${boardState[k].type}:${k}`);
  if (vowelCount) vowelCount.innerText = vowels.length;
  if (vowelDisplay) vowelDisplay.innerText = vowels.length > 0 ? vowels.join(', ') : 'None';

  // 4. Gray out PDF button if Start or Destination is missing
  if (pdfBtn) {
    const isComplete = Boolean(startCoord && destCoord);
    pdfBtn.disabled = !isComplete;
    
    // Add tooltip explaining why it's grayed out
    if (!isComplete) {
      pdfBtn.title = "Set both Start (🚀), Destination (🏁) and Starting Facing Direction to unlock PDF export.";
    } else {
      pdfBtn.removeAttribute('title');
    }
  }
}

function getItemConfig(type) {
  switch (type) {
    case 'start':    return { symbol: '🚀', className: 'cell-start' };
    case 'dest':     return { symbol: '🏁', className: 'cell-dest' };
    case 'obstacle': return { symbol: '✖', className: 'cell-obstacle' };
    default:         return { symbol: type, className: 'cell-vowel' };
  }
}

function createLabelCell(parent, text) {
  if (!parent) return; 
  const label = document.createElement('div');
  label.className = 'grid-label';
  label.innerText = text;
  parent.appendChild(label);
}

export function evaluateMissionRun(simulationResult) {
  // 1. Immediate Fail: Boundary Breach
  if (simulationResult.hitBorder) {
    return {
      passed: false,
      score: 0,
      breakdown: {
        itemWeight: "0.0",
        clearedObs: "0/0",
        vowels: "0/0"
      },
      reason: "CRASH: Robot hit the grid border and stopped!"
    };
  }

  // 2. Count Total Graded Grid Elements
  const totalObstacles = Object.keys(boardState).filter(k => boardState[k].type === 'obstacle').length;
  const totalVowels = Object.keys(boardState).filter(k => ['A','E','I','O','U'].includes(boardState[k].type)).length;
  const totalItems = totalObstacles + totalVowels;

  // Handle Sets properly (.size instead of .length)
  const hitObsCount = simulationResult.hitObstacles.size || 0;
  const visitedVowelsCount = simulationResult.visitedVowels.size || 0;
  const completedActionsCount = simulationResult.completedVowelActions.size || 0;

  // Edge case: If teacher places NO obstacles and NO vowels
  if (totalItems === 0) {
    const passed = simulationResult.reachedDestination;
    return {
      passed: passed,
      score: passed ? 100 : 0,
      breakdown: {
        itemWeight: "0.0",
        clearedObs: "N/A",
        vowels: "N/A"
      },
      reason: passed ? "Mission Successful!" : "Failed to reach destination."
    };
  }

  // 3. Dynamic Weight Calculation
  const itemWeight = 100 / totalItems;

  // 4. Calculate Obstacle Score
  const clearedObstacles = Math.max(0, totalObstacles - hitObsCount);
  const obstacleScore = clearedObstacles * itemWeight;

  // 5. Calculate Checkpoint Score
  const touchWeight = itemWeight * 0.5;
  const actionWeight = itemWeight * 0.5;

  const vowelTouchScore = visitedVowelsCount * touchWeight;
  const vowelActionScore = completedActionsCount * actionWeight;
  const vowelScore = vowelTouchScore + vowelActionScore;

  // 6. Aggregate Final Score
  const rawScore = obstacleScore + vowelScore;
  const finalScore = Math.min(100, Math.round(rawScore));
  const passed = simulationResult.reachedDestination;

  return {
    passed: passed,
    score: passed ? finalScore : 0,
    breakdown: {
      itemWeight: itemWeight.toFixed(1),
      clearedObs: `${clearedObstacles}/${totalObstacles}`,
      vowels: `${visitedVowelsCount}/${totalVowels}`
    },
    reason: passed ? "Mission Completed!" : "FAILED: Did not end on Destination (🏁)."
  };
}

// UI Popup & Banner Controllers
export function showGradeModal(evalData) {
  const modal = document.getElementById('gradeModal');
  const body = document.getElementById('modalBody');
  const logoInput = document.getElementById('logoInput');

  // PDF Print target elements
  const printLogoCode = document.getElementById('printLogoCode');
  const printGradeBody = document.getElementById('printGradeBody');

  if (!modal || !body) {
    console.error("Grade modal elements missing in index.html!");
    return;
  }

  const tagClass = evalData.passed ? 'tag-pass' : 'tag-fail';
  const tagText = evalData.passed ? 'PASSED' : 'FAILED';

  const contentHtml = `
    <div style="text-align: center; margin-bottom: 12px;">
      <span class="status-tag ${tagClass}">${tagText}</span>
      <h1 style="font-size: 32px; margin: 8px 0; color: #0f172a;">${evalData.score}%</h1>
      <p style="font-size: 12px; color: #64748b; margin: 0;">${evalData.reason}</p>
    </div>

    <table class="eval-table">
      <tr><td>Weight per Item:</td><td>${evalData.breakdown.itemWeight}% each</td></tr>
      <tr><td>Obstacles Avoided:</td><td>${evalData.breakdown.clearedObs}</td></tr>
      <tr><td>Checkpoints Visited:</td><td>${evalData.breakdown.vowels}</td></tr>
    </table>
  `;

  // 2. Render on-screen Modal
  body.innerHTML = contentHtml;
  modal.style.display = 'flex';

  // 3. Render PDF Print Summary
  if (printLogoCode && logoInput) {
    printLogoCode.innerText = logoInput.value.trim() || 'No code executed.';
  }
  if (printGradeBody) {
    printGradeBody.innerHTML = contentHtml;
  }
}

function updateGradeBanner(evalData) {
  const banner = document.getElementById('gradeSummaryBanner');
  const text = document.getElementById('bannerGradeText');
  if (banner && text) {
    banner.style.display = 'flex';
    const tag = evalData.passed ? 'PASS' : 'FAIL';
    text.innerText = `${evalData.score}% (${tag})`;
    text.style.color = evalData.passed ? '#15803d' : '#b91c1c';
  }
}

export function openLastGradeModal() {
  if (lastEvaluationResult){
    showGradeModal(lastEvaluationResult);
  } 
  else {
    alert("No simulation run evaluated yet!");
  }
}