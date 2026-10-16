// logoInterpreter.js

// Variable memory storage for 'make' declarations
const variables = {};

export function setVariable(name, val) {
  variables[name] = val;
}

export function getVariable(name) {
  return variables[name];
}


function evaluateExpr(exprStr) {
  let substituted = String(exprStr).trim();
  if (!substituted) return 0;

  // Replace variable names with their active value
  Object.keys(variables).forEach(varName => {
    const val = variables[varName];
    const formattedVal = typeof val === 'string' ? `"${val}"` : val;
    const regex = new RegExp(`\\b${varName}\\b`, 'gi');
    substituted = substituted.replace(regex, formattedVal);
  });

  // Convert single LOGO '=' to JS '=='
  substituted = substituted.replace(/([^!=<>])=([^=])/g, '$1==$2');

  try {
    return Function(`"use strict"; return (${substituted})`)();
  } catch (err) {
    console.error("Expression evaluation error:", exprStr, err);
    return 0;
  }
}

// Extract bracketed blocks
function extractBlock(tokens, startIndex) {
  let depth = 0;
  let blockTokens = [];
  let i = startIndex;

  if (tokens[i] !== '[') return { blockTokens: [], nextIndex: startIndex };

  for (; i < tokens.length; i++) {
    const token = tokens[i];
    if (token === '[') {
      depth++;
      if (depth === 1) continue;
    } else if (token === ']') {
      depth--;
      if (depth === 0) return { blockTokens, nextIndex: i + 1 };
    }

    if (depth > 0) {
      blockTokens.push(token);
    }
  }
  return { blockTokens, nextIndex: i };
}

// Main recursive execution parser
export async function parseAndExecuteLOGO(programText, stepCallback) {
  const VALID_KEYWORDS = [
    'make', 'while', 'repeat', 'ifelse', 
    'fd', 'bk', 
    'lt', 'rt', 
    '[', ']', 'vowel'
  ];

  const declaredVars = new Set();

  for (let i = 0; i < tokens.length; i++) {
  const token = tokens[i];
  const cmd = token.toLowerCase();

  // 1. If previous token was 'make', register this token as a valid variable name
  if (i > 0 && tokens[i - 1].toLowerCase() === 'make') {
    declaredVars.add(cmd);
    continue; // Skip keyword check for the variable name itself
  }

  const isKeyword = VALID_KEYWORDS.includes(cmd);
  const isNumber = !isNaN(token);
  const isStringLiteral = token.startsWith('"');
  const isOperator = ['==', '=', '!=', '>', '<', '>=', '<=', '+', '-', '*', '/'].includes(cmd);
  const isDeclaredVar = declaredVars.has(cmd) || (cmd in variables);

  if (!isKeyword && !isNumber && !isStringLiteral && !isOperator && !isDeclaredVar) {
    throw new Error(`Syntax Error: Unrecognized command '${token}'.`);
  }

  // Clear scope variables for fresh execution
  Object.keys(variables).forEach(k => delete variables[k]);

  // Check for unmatched brackets before running
  const openBrackets = (programText.match(/\[/g) || []).length; 
  const closeBrackets = (programText.match(/\]/g) || []).length;
  if (openBrackets !== closeBrackets) {
    throw new Error(`Syntax Error: Unmatched brackets ([${openBrackets}] vs ]${closeBrackets}]).`);
  }

  // Normalize code string and break into tokens
  const tokens = programText
    .replace(/\[/g, ' [ ') .replace(/\]/g, ' ] ')
    .trim()
    .split(/\s+/)
    .filter(t => t.length > 0);

  // Validate command tokens
  const validCmds = ['make', 'while', 'repeat', 'ifelse', 'fd', 'bk', 'lt', 'rt', '[', ']', 'vowel'];

  const declaredVars = new Set();
  for (let i = 0; i < tokens.length; i++) {
    if (tokens[i].toLowerCase() === 'make' && i + 1 < tokens.length) {
        declaredVars.add(tokens[i + 1].toLowerCase());
    }
  }

  // 3. FULL PRE-PASS VALIDATION (Scans ALL tokens regardless of runtime branches)
  for (let i = 0; i < tokens.length; i++) {
    const token = tokens[i];
    const cmd = token.toLowerCase();

    const isKeyword = VALID_KEYWORDS.includes(cmd);
    const isNumber = !isNaN(token);
    const isStringLiteral = token.startsWith('"');
    const isOperator = ['==', '=', '!=', '>', '<', '>=', '<='].includes(cmd);
    const isDeclaredVar = declaredVars.has(cmd);

    // Check if it's a variable name preceded by 'make' or defined during run
    const isMakeVar = i > 0 && tokens[i - 1].toLowerCase() === 'make';

    if (!isKeyword && !isNumber && !isStringLiteral && !isOperator && !isMakeVar) {
      // Throw syntax error immediately before running anything!
      throw new Error(`Syntax Error: Unrecognized command '${token}'.`);
    }
  }

  async function runTokens(tokenList) {
    let ptr = 0;
    const VALID_KEYWORDS = [
        'make', 'while', 'repeat', 'ifelse', 
        'fd', 'bk', 'backward', 
        'lt', 'rt', 
        '[', ']', 'vowel'
    ];

    while (ptr < tokenList.length) {
        const token = tokenList[ptr];
        const cmd = token.toLowerCase();

        // 1. Check if token is an unrecognized command/word
        const isKeyword = VALID_KEYWORDS.includes(cmd);
        const isNumber = !isNaN(token);
        const isStringLiteral = token.startsWith('"');
        const isOperator = ['==', '=', '!=', '>', '<', '>=', '<='].includes(cmd);
        const isVariable = cmd in variables;

        if (!isKeyword && !isNumber && !isStringLiteral && !isOperator && !isVariable) {
            throw new Error(`Syntax Error: Unrecognized command '${token}'.`);
        }

        // 1. MAKE VARNAME EXPR
        if (cmd === 'make') {
            const varName = tokenList[ptr + 1];
            ptr += 2;
            let exprTokens = [];
            
            while (
            ptr < tokenList.length && 
            !['make','while','repeat','ifelse','fd','bk','lt','rt','[', 'vowel'].includes(tokenList[ptr].toLowerCase())
            ) {
            exprTokens.push(tokenList[ptr]);
            ptr++;
            }
            
            const val = evaluateExpr(exprTokens.join(' '));
            variables[varName] = val;
            continue;
        }

        // 2. REPEAT EXPR [ STATEMENTS ]
        if (cmd === 'repeat') {
            const countExpr = tokenList[ptr + 1];
            const times = evaluateExpr(countExpr);
            ptr += 2;

            const { blockTokens, nextIndex } = extractBlock(tokenList, ptr);
            ptr = nextIndex;

            for (let r = 0; r < times; r++) {
            await runTokens(blockTokens);
            }
            continue;
        }

        // 3. WHILE EXPR [ STATEMENTS ]
        if (cmd === 'while') {
            ptr++;
            let exprTokens = [];
            while (ptr < tokenList.length && tokenList[ptr] !== '[') {
            exprTokens.push(tokenList[ptr]);
            ptr++;
            }
            const exprStr = exprTokens.join(' ');

            const { blockTokens, nextIndex } = extractBlock(tokenList, ptr);
            ptr = nextIndex;

            let maxGuard = 2000; // Protect against infinite loops
            while (evaluateExpr(exprStr) && maxGuard > 0) {
            await runTokens(blockTokens);
            maxGuard--;
            }
            continue;
        }

        // 4. IFELSE EXPR [ TRUE_BLOCK ] [ FALSE_BLOCK ]
        if (cmd === 'ifelse') {
            ptr++;
            let exprTokens = [];
            while (ptr < tokenList.length && tokenList[ptr] !== '[') {
            exprTokens.push(tokenList[ptr]);
            ptr++;
            }
            const exprStr = exprTokens.join(' ');

            const trueBlock = extractBlock(tokenList, ptr);
            ptr = trueBlock.nextIndex;

            const falseBlock = extractBlock(tokenList, ptr);
            ptr = falseBlock.nextIndex;

            if (evaluateExpr(exprStr)) {
            await runTokens(trueBlock.blockTokens);
            } else {
            await runTokens(falseBlock.blockTokens);
            }
            continue;
        }

        // 5. TURTLE MOTIONS (FD, BK, LT, RT)
        if (['fd', 'bk', 'lt', 'rt'].includes(cmd)) {
            const val = evaluateExpr(tokenList[ptr + 1] || '1');
            await stepCallback(cmd, val);
            ptr += 2;
            continue;
        }

        ptr++;
        }
    }
  await runTokens(tokens);
}