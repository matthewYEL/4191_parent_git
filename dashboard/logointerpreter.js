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
    Object.keys(variables).forEach(k => delete variables[k]);

    const openBrackets = (programText.match(/\[/g) || []).length; 
    const closeBrackets = (programText.match(/\]/g) || []).length;
    if (openBrackets !== closeBrackets) {
        throw new Error(`Syntax Error: Unmatched brackets ([${openBrackets}] vs ]${closeBrackets}]).`);
    }

    const tokens = programText
    .replace(/\[/g, ' [ ') .replace(/\]/g, ' ] ')
    .trim()
    .split(/\s+/)
    .filter(t => t.length > 0);

    const VALID_KEYWORDS = [
        'make', 'while', 'repeat', 'ifelse', 
        'fd', 'bk', 
        'lt', 'rt', 
        '[', ']', 'vowel'
    ];

    // Pre-register all variable names declared via 'make'
    const declaredVars = new Set();
    for (let i = 0; i < tokens.length; i++) {
        if (tokens[i].toLowerCase() === 'make' && i + 1 < tokens.length) {
            declaredVars.add(tokens[i + 1].toLowerCase());
        }
    }

    //FULL PRE-PASS VALIDATION (Scans ALL tokens regardless of runtime branches)
    for (let i = 0; i < tokens.length; i++) {
        const token = tokens[i];
        const cmd = token.toLowerCase();

        const isKeyword = VALID_KEYWORDS.includes(cmd);
        const isNumber = !isNaN(Number(token));
        const isStringLiteral = token.startsWith('"') || token.endsWith('"');
        const isOperator = ['==', '=', '!=', '>', '<', '>=', '<=', '+', '-', '*', '/'].includes(cmd);
        const isDeclaredVar = declaredVars.has(cmd);
        const isMakeVar = i > 0 && tokens[i - 1].toLowerCase() === 'make';

        if (!isKeyword && !isNumber && !isStringLiteral && !isOperator && !isDeclaredVar && !isMakeVar) {
        throw new Error(`Syntax Error: Unrecognized command '${token}'.`);
        }
    }
    async function runTokens(tokenList) {
        let ptr = 0;

        while (ptr < tokenList.length) {
        const token = tokenList[ptr];
        const cmd = token.toLowerCase();

        // Runtime command validation
        const isKeyword = VALID_KEYWORDS.includes(cmd);
        const isNumber = !isNaN(Number(token));
        const isStringLiteral = token.startsWith('"') || token.endsWith('"');
        const isOperator = ['==', '=', '!=', '>', '<', '>=', '<=', '+', '-', '*', '/'].includes(cmd);
        const isVariable = cmd in variables || declaredVars.has(cmd);

        if (!isKeyword && !isNumber && !isStringLiteral && !isOperator && !isVariable) {
            throw new Error(`Syntax Error: Unrecognized command '${token}'.`);
        }

        // 6.1 MAKE VARNAME EXPR
        if (cmd === 'make') {
            const varName = tokenList[ptr + 1];
            if (!varName) throw new Error("Syntax Error: 'make' requires a variable name.");
            ptr += 2;
            
            let exprTokens = [];
            while (
            ptr < tokenList.length && 
            !VALID_KEYWORDS.includes(tokenList[ptr].toLowerCase())
            ) {
            exprTokens.push(tokenList[ptr]);
            ptr++;
            }
            
            const val = evaluateExpr(exprTokens.join(' '));
            variables[varName] = val;
            continue;
        }

      // 6.2 REPEAT EXPR [ STATEMENTS ]
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

        // 6.3 WHILE EXPR [ STATEMENTS ]
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

            let maxGuard = 2000;
            while (evaluateExpr(exprStr) && maxGuard > 0) {
            const hitBorder = await runTokens(blockTokens);
            if (hitBorder) break;
            maxGuard--;
            }
            continue;
        }

        // 6.4 IFELSE EXPR [ TRUE_BLOCK ] [ FALSE_BLOCK ]
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

        // 6.5 TURTLE MOTIONS
        if (['fd', 'bk', 'lt', 'rt'].includes(cmd)) {
            const val = evaluateExpr(tokenList[ptr + 1] || '1');
            const stepResult = await stepCallback(cmd, val);
            ptr += 2;
            if (stepResult === true) return true; // Stop execution early if robot crashes into border
            continue;
        }

        ptr++;
        }
    }

    await runTokens(tokens);
}