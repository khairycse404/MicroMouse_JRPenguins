#include "Melody.h"

// ==================== Notes ====================
#define C4   262
#define CS4  277
#define D4   294
#define DS4  311
#define E4   330
#define F4   349
#define FS4  370
#define G4   392
#define GS4  415
#define A4   440
#define AS4  466
#define B4   494

#define C5   523
#define CS5  554
#define D5   587
#define DS5  622
#define E5   659
#define F5   698
#define FS5  740
#define G5   784
#define GS5  831
#define A5   880
#define AS5  932
#define B5   988

#define REST 0

// ==================== Tempo Setup ====================
const int tempo = 114;
const int quarterNote = (60000 / tempo);

const int DOTTED_HALF = quarterNote * 3;
const int HALF        = quarterNote * 2;
const int DOTTED_QTR  = quarterNote * 1.5;
const int QTR         = quarterNote;
const int DOTTED_8TH  = quarterNote * 0.75;
const int EIGHTH      = quarterNote / 2;
const int SIXTEENTH   = quarterNote / 4;
const int TRIPLET     = quarterNote / 3;

// ==================== Play Function ====================
void playNote(uint8_t pin, int note, int duration) {
  if (note == REST) {
    delay(duration);
  } else {
    int playTime = duration * 0.85;
    int pauseTime = duration * 0.15;
    
    tone(pin, note, playTime);
    delay(playTime);
    noTone(pin);
    delay(pauseTime);
  }
}

// ==================== Melody ====================
void playVictoryMelody(uint8_t buzzerPin) {
  // Measures 1-2 (Intro pickup)
  playNote(buzzerPin, REST, HALF);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, D5, EIGHTH);

  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, D5, EIGHTH);
  playNote(buzzerPin, E5, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);

  // Measure 3-6
  playNote(buzzerPin, D5, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, G4, EIGHTH);
  playNote(buzzerPin, G4, EIGHTH);

  playNote(buzzerPin, FS4, EIGHTH);
  playNote(buzzerPin, E4, EIGHTH);
  playNote(buzzerPin, D4, QTR);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, D5, EIGHTH);

  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, D5, EIGHTH);
  playNote(buzzerPin, E5, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);

  playNote(buzzerPin, D5, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, G4, EIGHTH);
  playNote(buzzerPin, G4, EIGHTH);

  playNote(buzzerPin, FS4, EIGHTH);
  playNote(buzzerPin, E4, EIGHTH);
  playNote(buzzerPin, D4, HALF);

  // Measures 7-16 (Main Theme repeated)
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, D5, EIGHTH);

  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, D5, EIGHTH);
  playNote(buzzerPin, E5, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);

  playNote(buzzerPin, D5, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, G4, EIGHTH);
  playNote(buzzerPin, G4, EIGHTH);

  playNote(buzzerPin, FS4, EIGHTH);
  playNote(buzzerPin, E4, EIGHTH);
  playNote(buzzerPin, D4, QTR);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, D5, EIGHTH);

  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, D5, EIGHTH);
  playNote(buzzerPin, E5, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);

  playNote(buzzerPin, D5, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, G4, EIGHTH);
  playNote(buzzerPin, G4, EIGHTH);

  playNote(buzzerPin, FS4, EIGHTH);
  playNote(buzzerPin, E4, EIGHTH);
  playNote(buzzerPin, D4, QTR);

  // Measure 17-23 (Middle Section)
  playNote(buzzerPin, FS4, QTR);
  playNote(buzzerPin, G4, QTR);
  playNote(buzzerPin, E4, QTR);
  playNote(buzzerPin, G4, QTR);
  playNote(buzzerPin, FS4, HALF);

  playNote(buzzerPin, FS4, QTR);
  playNote(buzzerPin, G4, QTR);
  playNote(buzzerPin, E4, QTR);
  playNote(buzzerPin, G4, QTR);
  playNote(buzzerPin, FS4, HALF);

  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, FS4, EIGHTH);

  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, FS4, EIGHTH);

  playNote(buzzerPin, D4, EIGHTH);
  playNote(buzzerPin, D4, EIGHTH);
  playNote(buzzerPin, E4, QTR);
  playNote(buzzerPin, FS4, HALF);

  playNote(buzzerPin, D4, EIGHTH);
  playNote(buzzerPin, FS4, EIGHTH);
  playNote(buzzerPin, A4, QTR);
  playNote(buzzerPin, E4, EIGHTH);
  playNote(buzzerPin, FS4, QTR + EIGHTH);

  // Measure 25-31 (High Peak)
  playNote(buzzerPin, B4, QTR);
  playNote(buzzerPin, FS5, QTR);
  playNote(buzzerPin, B5, QTR);
  playNote(buzzerPin, A5, QTR);
  playNote(buzzerPin, FS5, HALF);

  playNote(buzzerPin, FS5, QTR);
  playNote(buzzerPin, E5, HALF);
  playNote(buzzerPin, E5, EIGHTH);
  playNote(buzzerPin, FS5, EIGHTH);

  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, D5, EIGHTH);

  playNote(buzzerPin, E5, EIGHTH);
  playNote(buzzerPin, D5, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);

  playNote(buzzerPin, A5, QTR);
  playNote(buzzerPin, FS5, QTR);
  playNote(buzzerPin, A5, QTR);
  playNote(buzzerPin, E5, QTR);

  playNote(buzzerPin, GS5, HALF);
  playNote(buzzerPin, REST, EIGHTH);
  playNote(buzzerPin, A5, DOTTED_QTR);

  playNote(buzzerPin, D5, EIGHTH);
  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, D4, EIGHTH);

  // Measure 34-38 (Triplets Section - measure 35 to 37)
  playNote(buzzerPin, FS4, QTR);
  playNote(buzzerPin, G4, QTR);
  playNote(buzzerPin, E4, QTR);
  playNote(buzzerPin, G4, QTR);
  playNote(buzzerPin, FS4, HALF);

  // Triplets (from page 2)
  playNote(buzzerPin, FS5, TRIPLET);
  playNote(buzzerPin, E5, TRIPLET);
  playNote(buzzerPin, DS5, TRIPLET);
  playNote(buzzerPin, CS5, TRIPLET);
  playNote(buzzerPin, B4, TRIPLET);
  playNote(buzzerPin, CS5, TRIPLET);

  playNote(buzzerPin, FS5, TRIPLET);
  playNote(buzzerPin, E5, TRIPLET);
  playNote(buzzerPin, DS5, TRIPLET);
  playNote(buzzerPin, CS5, TRIPLET);
  playNote(buzzerPin, B4, TRIPLET);
  playNote(buzzerPin, CS5, TRIPLET);

  playNote(buzzerPin, FS5, TRIPLET);
  playNote(buzzerPin, E5, TRIPLET);
  playNote(buzzerPin, DS5, TRIPLET);
  playNote(buzzerPin, CS5, TRIPLET);
  playNote(buzzerPin, B4, TRIPLET);
  playNote(buzzerPin, CS5, TRIPLET);

  // Key Modulation / Second Section (E-major transition)
  playNote(buzzerPin, E4, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, E5, EIGHTH);
  playNote(buzzerPin, FS5, EIGHTH);
  playNote(buzzerPin, DS5, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, DS5, EIGHTH);
  playNote(buzzerPin, E5, EIGHTH);

  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, G4, EIGHTH);
  playNote(buzzerPin, G4, EIGHTH);
  playNote(buzzerPin, FS4, EIGHTH);
  playNote(buzzerPin, E4, EIGHTH);

  playNote(buzzerPin, E5, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, E5, EIGHTH);
  playNote(buzzerPin, FS5, EIGHTH);
  playNote(buzzerPin, DS5, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, DS5, EIGHTH);
  playNote(buzzerPin, E5, EIGHTH);

  playNote(buzzerPin, CS5, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, B4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, A4, EIGHTH);
  playNote(buzzerPin, G4, EIGHTH);
  playNote(buzzerPin, G4, EIGHTH);
  playNote(buzzerPin, FS4, EIGHTH);
  playNote(buzzerPin, E4, HALF);
}