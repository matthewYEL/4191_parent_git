// templates.js
// Preset boards for teachers. Rows: 1 = top. dir: 0=N, 90=E, 180=S, 270=W.
// robotOnly: uses repeat/while/ifelse, which the on-screen simulator cannot grade.

export const TEMPLATES = [
  // ---------- Beginner ----------
  {
    id: 'first-steps', level: 'Beginner', name: 'First Steps',
    goal: 'Drive straight to the flag.',
    start: 'J18', dir: 0, dest: 'J12',
    include: ['forward'], exclude: ['left', 'right'],
    solution: 'fd 6'
  },
  {
    id: 'simple-game', level: 'Beginner', name: 'Simple Game: Obstacle + Vowel',
    goal: 'Avoid the obstacle, touch the vowel, finish on the flag.',
    start: 'D16', dir: 0, dest: 'K9',
    obstacles: ['D12'], vowels: { A: 'H16' },
    include: ['forward', 'left', 'right'],
    solution: 'rt 90 fd 4 lt 90 fd 7 rt 90 fd 3'
  },

  // ---------- Intermediate ----------
  {
    id: 'the-wall', level: 'Intermediate', name: 'The Wall',
    goal: 'Get around the wall without touching it.',
    start: 'B10', dir: 90, dest: 'S10',
    obstacles: ['J8', 'J9', 'J10', 'J11', 'J12'],
    solution: 'lt 90 fd 4 rt 90 fd 17 rt 90 fd 4'
  },
  {
    id: 'no-left-turns', level: 'Intermediate', name: 'No Left Turns',
    goal: 'Reach the flag using only right turns.',
    start: 'H10', dir: 0, dest: 'D6',
    exclude: ['left'],
    solution: 'fd 4 rt 270 fd 4'
  },
  {
    id: 'diagonal-dash', level: 'Intermediate', name: 'Diagonal Dash',
    goal: 'Both L-shaped routes are blocked. Go diagonally! (Untested with IR line-stop.)',
    start: 'C18', dir: 0, dest: 'H13',
    obstacles: ['C15', 'F18'],
    solution: 'rt 45 fd 5'
  },

  // ---------- Advanced ----------
  {
    id: 'vowel-trail', level: 'Advanced', name: 'Vowel Trail',
    goal: 'Visit A, E and I, then finish on the flag.',
    start: 'B19', dir: 0, dest: 'J10',
    vowels: { A: 'B15', E: 'F15', I: 'F10' },
    solution: 'fd 4 rt 90 fd 4 lt 90 fd 5 rt 90 fd 4'
  },
  {
    id: 'staircase', level: 'Advanced', name: 'Staircase (repeat)',
    goal: 'Climb the stairs to the flag using a loop.',
    start: 'D17', dir: 0, dest: 'J11',
    obstacles: ['D14', 'G17'],
    include: ['repeat'], exclude: ['while'],
    solution: 'repeat 6 [ fd 1 rt 90 fd 1 lt 90 ]',
    robotOnly: false
  },
  {
    id: 'find-vowel', level: 'Advanced', name: 'Find the Vowel (while)',
    goal: 'Drive until the camera sees a vowel. Move the card between rounds: the same code still works.',
    start: 'J18', dir: 0,
    vowels: { O: 'J8' },
    include: ['while'],
    solution: 'make vowel "NONE"\nwhile vowel == "NONE" [ fd 1 ]',
    robotOnly: false
  },
  {
    id: 'fork-road', level: 'Advanced', name: 'Fork in the Road (ifelse)',
    goal: 'Card A means go left, anything else means go right. Flag is set for A; move it to N14 if the card is E.',
    start: 'J18', dir: 0, dest: 'F14',
    obstacles: ['J13'], vowels: { A: 'J14' },
    include: ['ifelse'],
    solution: 'make vowel "NONE"\nfd 4 \nifelse vowel == "A" [ lt 90 fd 4 ] [ rt 90 fd 4 ]',
    robotOnly: false
  },
  {
    id: 'seek-decide', level: 'Advanced', name: 'Seek and Decide (while + ifelse)',
    goal: 'Find the vowel, then turn right if it is E, otherwise left. (while then ifelse untested on robot.)',
    start: 'J18', dir: 0, dest: 'M10',
    vowels: { E: 'J10' },
    include: ['while', 'ifelse'],
    solution: 'make vowel "NONE"\nwhile vowel == "NONE" [ fd 1 ]\nifelse vowel == "E" [ rt 90 fd 3 ] [ lt 90 fd 3 ]',
    robotOnly: false
  }
];
