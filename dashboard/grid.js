// grid.js
import { sendData } from './bluetooth.js';
import { parseAndExecuteLOGO, setVariable } from './logointerpreter.js';

const COLS = ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T'];
const ROWS = 20;

// Board State Tracking
let boardState = {}; // Format: { "B14": { type: "start", symbol: "🚀" } }
let startCoord = null;
let destCoord = null;
let hoveredCellKey = null;
let container = null;
let selectedCellKey = null; // Tracks currently clicked/selected grid cell

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
        selectCell(`${col}${r}`);
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
    coordInput.addEventListener('input', (e) => {
      const val = e.target.value.trim().toUpperCase();
      if (val.match(/^([A-T])([1-9]|1[0-9]|20)$/)) {
        selectCell(val);
      }
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

//Check if required commands are missing from user code
export function checkMissingRequiredCommands(commandString) {
  const missing = [];
  const lowerCode = commandString.toLowerCase();

  // Mapping checkbox IDs/labels to command keywords
  const requiredChecks = [
    { id: 'include-while', label: 'while', keywords: ['while'] },
    { id: 'include-ifelse', label: 'ifelse', keywords: ['ifelse'] },
    { id: 'include-repeat', label: 'repeat', keywords: ['repeat'] },
    { id: 'include-forward', label: 'forward (fd)', keywords: ['fd'] },
    { id: 'include-backward', label: 'backward (bk)', keywords: ['bk'] },
    { id: 'include-left', label: 'left turn (lt)', keywords: ['lt'] },
    { id: 'include-right', label: 'right turn (rt)', keywords: ['rt'] }
  ];

  requiredChecks.forEach(item => {
    const checkbox = document.getElementById(item.id);
    if (checkbox && checkbox.checked) {
      // Check if code contains at least one of the matching command keywords
      const hasCmd = item.keywords.some(kw => new RegExp(`\\b${kw}\\b`, 'i').test(lowerCode));
      if (!hasCmd) {
        missing.push(item.label);
      }
    }
  });

  return missing;
}

// Helper to validate code against "Must Exclude" checkboxes
function checkExcludedCommands(codeText) {
  const excludedBoxes = document.querySelectorAll('.game-checkboxes input.cross-checkbox:checked');
  const upperCode = codeText.toUpperCase();
  const violations = [];

  excludedBoxes.forEach(box => {
    const cmd = box.dataset.command;
    let pattern;

    if (cmd === 'left') pattern = /\b(LT|LEFT)\b/;
    else if (cmd === 'right') pattern = /\b(RT|RIGHT)\b/;
    else if (cmd === 'forward') pattern = /\b(FD|FORWARD)\b/;
    else if (cmd === 'backward') pattern = /\b(BK|BACKWARD)\b/;
    else pattern = new RegExp(`\\b${cmd.toUpperCase()}\\b`);

    if (pattern.test(upperCode)) {
      violations.push(cmd);
    }
  });

  return violations;
}

export async function executeLogoCommands(commandString) {
  if (!startCoord) {
    alert("Please place a Start point (🚀) on the grid first!");
    return;
  }

  //Validate Must Exclude constraints
  const violations = checkExcludedCommands(commandString);
  if (violations.length > 0) {
    alert(`❌ Code Execution Blocked!\nYour code contains excluded command(s): ${violations.join(', ')}`);
    return;
  }

  resetHighlights();
  clearTrail();
  syncRobotToStart();

  // Create clean tracker object for this run
  const runTracker = {
    hitBorder: false,
    reachedDestination: false,
    hitObstacles: new Set(),
    visitedVowels: new Set(),
    hasSyntaxError: false,
    syntaxErrorMsg: "",
    hasConstraintViolation: false,
    constraintMsg: ""
  };

  // CHECK MUST INCLUDE / MUST EXCLUDE CONSTRAINTS
  const missingIncludes = checkMissingRequiredCommands(commandString); 
  const forbiddenExcludes = checkExcludedCommands(commandString); 

  if (missingIncludes.length > 0) {
    runTracker.hasConstraintViolation = true;
    runTracker.constraintMsg = `FAILED: Code missing required command(s): ${missingIncludes.join(', ')}.`;
  } else if (forbiddenExcludes.length > 0) {
    runTracker.hasConstraintViolation = true;
    runTracker.constraintMsg = `FAILED: Code uses forbidden command(s): ${forbiddenExcludes.join(', ')}.`;
  }
  
  // RUN SIMULATION WITH SYNTAX ERROR CATCHING
  if (!runTracker.hasConstraintViolation) {
    try {
      await parseAndExecuteLOGO(commandString, async (cmd, val) => {
        let hitBorder = false;

        if (cmd === 'fd') {
          hitBorder = await moveRobotForward(val, runTracker);
        } else if (cmd === 'bk') {
          hitBorder = await moveRobotForward(-val, runTracker);
        } else if (cmd === 'rt') {
          robotState.heading = (robotState.heading + val + 360) % 360;
          highlightCurrentRobotCell(robotState.colIdx, robotState.rowIdx);
          await new Promise(r => setTimeout(r, 150));
        } else if (cmd === 'lt') {
          robotState.heading = (robotState.heading - val + 360) % 360;
          highlightCurrentRobotCell(robotState.colIdx, robotState.rowIdx);
          await new Promise(r => setTimeout(r, 150));
        }

        return hitBorder;
      });
    } catch (err) {
      console.error("LOGO Syntax/Runtime Error:", err);
      runTracker.hasSyntaxError = true;
      runTracker.syntaxErrorMsg = `FAILED: Code Syntax Error (${err.message}).`;
    }
  }

  //CHECK DESTINATION
  const finalKey = `${COLS[robotState.colIdx]}${robotState.rowIdx + 1}`;
  if (destCoord && finalKey === destCoord) {
    runTracker.reachedDestination = true;
  } else if (!destCoord) {
    // If no destination exists on grid, mark as reached automatically
    runTracker.reachedDestination = true; 
  }

  // Evaluate final score & pop modal
  lastEvaluationResult = evaluateMissionRun(runTracker);
  showGradeModal(lastEvaluationResult);
  updateGradeBanner(lastEvaluationResult);

  console.log("🏁 Final Cell Key:", `${COLS[robotState.colIdx]}${robotState.rowIdx + 1}`);
  console.log("🎯 Required Dest:", destCoord);
  console.log("✅ Reached Dest?:", runTracker.reachedDestination);
}

async function moveRobotForward(steps, runTracker) {
  const dir = steps >= 0 ? 1 : -1;
  const count = Math.abs(steps);

  for (let step = 0; step < count; step++) {
    markCellTrail(robotState.colIdx, robotState.rowIdx);

    let nextCol = robotState.colIdx;
    let nextRow = robotState.rowIdx;

    const normHeading = ((robotState.heading % 360) + 360) % 360;
    switch (normHeading) {
      case 0:   nextRow -= dir; break;                  // North
      case 45:  nextRow -= dir; nextCol += dir; break;  // North-East ↗️
      case 90:  nextCol += dir; break;                  // East
      case 135: nextRow += dir; nextCol += dir; break;  // South-East ↘️
      case 180: nextRow += dir; break;                  // South
      case 225: nextRow += dir; nextCol -= dir; break;  // South-West ↙️
      case 270: nextCol -= dir; break;                  // West
      case 315: nextRow -= dir; nextCol -= dir; break;  // North-West ↖️
    }

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
    
    if (boardState[currentKey]) {
      const cellType = boardState[currentKey].type;
      if (cellType === 'obstacle' && runTracker) runTracker.hitObstacles.add(currentKey);
      
      if (['A','E','I','O','U'].includes(cellType)) {
        if (runTracker) runTracker.visitedVowels.add(cellType);
        setVariable('vowel', cellType); // <-- ADD THIS LINE: Sets variable to 'A', 'E', 'I', 'O', or 'U'
      } else {
        setVariable('vowel', "NONE"); // <-- ADD THIS LINE: Resets to 0 for non-vowel items
      }
    } else {
      setVariable('vowel', "NONE"); // <-- ADD THIS LINE: Resets to 0 for empty grid cells
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

    const normHeading = ((robotState.heading % 360) + 360) % 360;
    cell.innerHTML = `<span class="turtle-icon turtle-facing-${normHeading}">🐢</span>`;

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
  if (!container) return;

  container.querySelectorAll('.grid-cell').forEach(cell => {
    cell.classList.remove('path-trail', 'robot-current', 'cell-selected');
    cell.style.removeProperty('background-color');
    cell.style.removeProperty('border-color');
    cell.style.backgroundColor = '';
    cell.style.borderColor = '';

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

  const key = `${match[1]}${parseInt(match[2], 10)}`;
  placeItem(key, itemSelect.value);
  updateUI();
}

// Place one item (start/dest/obstacle/vowel) at a key like "B14"
function placeItem(key, selectedType) {
  const col = key[0];
  const row = key.slice(1);

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
}

// Load a preset board from templates.js, replacing the current board.
export function loadTemplate(t) {
  clearEntireGrid();
  placeItem(t.start, 'start');
  if (t.dest) placeItem(t.dest, 'dest');
  (t.obstacles || []).forEach(key => placeItem(key, 'obstacle'));
  Object.entries(t.vowels || {}).forEach(([vowel, key]) => placeItem(key, vowel));

  startDir = t.dir;
  if (startDirSelect) startDirSelect.value = String(t.dir);
  syncRobotToStart();

  setConstraints(t.include || [], t.exclude || []);
  updateUI();
}

// Reset all Must Include / Must Exclude boxes, then tick the template's ones.
// Dispatching 'change' reuses the mutual-exclusion logic in main.js.
function setConstraints(include, exclude) {
  const boxes = document.querySelectorAll('.checkbox-row-wrapper input[type="checkbox"]');
  boxes.forEach(box => {
    box.checked = false;
    box.disabled = false;
    const label = box.closest('.print-checkbox-item');
    if (label) label.classList.remove('disabled-item');
  });
  boxes.forEach(box => {
    const wanted = box.classList.contains('cross-checkbox') ? exclude : include;
    if (wanted.includes(box.dataset.command)) {
      box.checked = true;
      box.dispatchEvent(new Event('change'));
    }
  });
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

function selectCell(key) {
  // Clear previous selection highlight
  if (selectedCellKey) {
    const prevCell = container.querySelector(`[data-col="${selectedCellKey[0]}"][data-row="${selectedCellKey.slice(1)}"]`);
    if (prevCell) prevCell.classList.remove('cell-selected');
  }

  selectedCellKey = key;

  // Add blue highlight to newly selected cell
  const newCell = container.querySelector(`[data-col="${key[0]}"][data-row="${key.slice(1)}"]`);
  if (newCell) {
    newCell.classList.add('cell-selected');
  }
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
  clearTrail();
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
  const dirMap = { 
    0: '0° ⬆️', 45: '45° ↗️', 90: '90° ➡️', 135: '135° ↘️',
    180: '180° ⬇️', 225: '225° ↙️', 270: '270° ⬅️', 315: '315° ↖️' 
  };
  const normDir = ((startDir % 360) + 360) % 360;
  startDisplay.innerText = `${startCoord} (${dirMap[normDir] || normDir + '°'})`;
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

  // 4. Gray out PDF button if Start is missing
  if (pdfBtn) {
    const isComplete = Boolean(startCoord);
    pdfBtn.disabled = !isComplete;
    
    // Add tooltip explaining why it's grayed out
    if (!isComplete) {
      pdfBtn.title = "Set Start (🚀) to unlock PDF export.";
    } else {
      pdfBtn.removeAttribute('title');
    }
  }
}

export function resetHighlights() {
  if (!container) return;
  container.querySelectorAll('.grid-cell').forEach(cell => {
    cell.classList.remove('path-trail', 'robot-current', 'cell-selected');
    cell.style.backgroundColor = '';
    cell.style.borderColor = '';

    const key = `${cell.dataset.col}${cell.dataset.row}`;

    // Teacher view ALWAYS sees vowel symbols on screen grid
    if (boardState[key]) {
      cell.innerText = boardState[key].symbol;
    } else {
      cell.innerText = '';
    }
  });
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
  // Safely extract Set sizes with fallbacks to avoid TypeErrors
  const hitObsCount = simulationResult?.hitObstacles?.size || 0;
  const visitedVowelsCount = simulationResult?.visitedVowels?.size || 0;

  // Calculate grid counts dynamically
  const totalObstacles = Object.keys(boardState).filter(k => boardState[k].type === 'obstacle').length;
  const totalVowels = Object.keys(boardState).filter(k => ['A','E','I','O','U'].includes(boardState[k].type)).length;
  const totalItems = totalObstacles + totalVowels;

  // OVERRIDE 1: Constraint Violation (Must Include/Exclude)
  if (simulationResult.hasConstraintViolation) {
    return {
      passed: false,
      score: 0,
      breakdown: { itemWeight: "0.0", clearedObs: `0/${totalObstacles}`, vowels: `0/${totalVowels}` },
      reason: simulationResult.constraintMsg || "FAILED: Rule constraint violation.",
      obstaclesAvoided: `${hitObsCount === 0 ? 1 : 0}/1`,
      checkpointsVisited: `${visitedVowelsCount}/${totalVowels}`
    };
  }

  // OVERRIDE 2: Syntax Error
  if (simulationResult.hasSyntaxError) {
    return {
      passed: false,
      score: 0,
      breakdown: { itemWeight: "0.0", clearedObs: "0/0", vowels: "0/0" },
      reason: simulationResult.syntaxErrorMsg || "FAILED: Syntax Error.",
      obstaclesAvoided: "0/1",
      checkpointsVisited: `0/${totalVowels}`
    };
  }

  // OVERRIDE 3: Border Crash
  if (simulationResult.hitBorder) {
    return {
      passed: false,
      score: 0,
      breakdown: { itemWeight: "0.0", clearedObs: "0/0", vowels: "0/0" },
      reason: "CRASH: Robot hit the grid border and stopped!",
      obstaclesAvoided: "0/1",
      checkpointsVisited: `${visitedVowelsCount}/${totalVowels}`
    };
  }

  // OVERRIDE 4: Did not reach destination
  if (destCoord && !simulationResult.reachedDestination) {
    return {
      passed: false,
      score: 0,
      breakdown: { itemWeight: "0.0", clearedObs: "0/0", vowels: "0/0" },
      reason: "FAILED: Did not end on Destination (🏁).",
      obstaclesAvoided: `${hitObsCount === 0 ? 1 : 0}/1`,
      checkpointsVisited: `${visitedVowelsCount}/${totalVowels}`
    };
  }

  if (totalItems === 0) {
    const passed = Boolean(simulationResult.reachedDestination);
    return {
      passed: passed,
      score: passed ? 100 : 0,
      breakdown: { itemWeight: "0.0", clearedObs: "N/A", vowels: "N/A" },
      reason: passed ? "Mission Successful!" : "FAILED: Did not end on Destination (🏁)."
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

  const vowelScore = totalVowels > 0 ? (visitedVowelsCount / totalVowels) * itemWeight : itemWeight;

  // 6. Aggregate Final Score
  const rawScore = obstacleScore + vowelScore;
  const finalScore = Math.min(100, Math.round(rawScore));
  const passed = destCoord ? Boolean(simulationResult.reachedDestination) : true;

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