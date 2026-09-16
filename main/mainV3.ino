#include "N20motors.h"
#include "TOFSensors.h"
#include "MousePID.h"
#include "Melody.h"
#include <Wire.h>
#include <math.h> // Needed for round()

// ==========================================
// 1. MAZE & CELL CONFIGURATION (8x8 CENTER GOAL)
// ==========================================
#define MAZE_W 8
#define MAZE_H 8
const float CELL_SIZE_MM = 175.0f; // Updated cell size to 18cm

int start_x = 0;  
int start_y = 0;  

const uint32_t DEBUG_PAUSE_MS = 100; 

// ==========================================
// 2. HARDWARE CONFIGURATION
// ==========================================
#define LEFT_IN1  21
#define LEFT_IN2  17
#define LEFT_ENCA 38 
#define LEFT_ENCB 40
#define RIGHT_IN1 34
#define RIGHT_IN2 36
#define RIGHT_ENCA 8 
#define RIGHT_ENCB 6

// --- LED & BUZZER PINS ---
#define LED_BLUE   15    // Right Opening Indicator
#define LED_RED    33    // Left Opening Indicator
#define LED_ORANGE 35    
#define BUZZER_PIN 11    // Victory Buzzer Pin

#define LEFT_ENC_POLARITY 1
#define RIGHT_ENC_POLARITY 1 

N20Motor leftMotor(LEFT_IN1, LEFT_IN2, LEFT_ENCA, LEFT_ENCB);
N20Motor rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_ENCA, RIGHT_ENCB);
TOFSensors tof;

const float WHEEL_DIAMETER = 19.0f;   
const float TICKS_PER_REV = 350.0f;   
const float MM_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;
const float TRACK_WIDTH_MM = 80.0f; 

const float FRONT_STOP_DIST_MM = 55.0f; 
const float TARGET_WALL_DIST_MM = 50.0f; // Adjusted target distance for 18cm cell
const float BRAKING_DIST_MM = 60.0f; 
const float SIDE_WALL_THRESHOLD_MM = 160.0f; 
const float TOO_CLOSE_WALL_MM = 20.0f; 

const float MAX_SCAN_DIST_MM = 135.0f; 
bool isFirstCell = true;

// --- ALIGNMENT TUNING VARIABLES ---
const float FRONT_WALL_MAX_DIST_MM = 800.0f; // Max reliable TOF distance
const float ALIGN_TOLERANCE_MM = 10.0f;      // Minimum error deadband (1cm)
const float ALIGN_OFFSET_MM = 20.0f;         // Offset to prevent over-reversing

float Kp = 3.5f, Ki = 0.001f, Kd = 0.08f, K_heading = 0.5f; 

MousePID stoppingPID(1.5f, 0.0f, 0.0f, 150.0f);   
MousePID centerPID(0.8f, 0.0f, 0.2f, 35.0f);      

float baseSpeed = 110.0f; 
float MIN_MOTOR_SPEED = 80.0f; 

void isrLeft()  { leftMotor.handleInterrupt(); }
void isrRight() { rightMotor.handleInterrupt(); }

void setMotorOutputs(float forwardSpeed, float turnSpeed) {
  float leftCmd  = forwardSpeed + turnSpeed;
  float rightCmd = -forwardSpeed + turnSpeed;
  leftMotor.setSpeed((int)leftCmd);
  rightMotor.setSpeed((int)rightCmd);
}

void moveDistanceMM(float distanceMM, int speed) {
  leftMotor.resetTicks();
  rightMotor.resetTicks();
  float targetTicks = distanceMM / MM_PER_TICK;
  
  while (true) {
    long leftTicks = abs(leftMotor.getTicks() * LEFT_ENC_POLARITY);
    long rightTicks = abs(rightMotor.getTicks() * RIGHT_ENC_POLARITY);
    long avgTicks = (leftTicks + rightTicks) / 2;

    if (avgTicks >= targetTicks) break;
    setMotorOutputs(speed, 0);
    delay(10);
  }
  setMotorOutputs(0, 0);
}

// Modular alignment function to center in current cell before taking a side turn
void alignToFrontWall() {
  tof.update();
  float frontDist = tof.getFrontMm();

  if (frontDist > 40.0f && frontDist < FRONT_WALL_MAX_DIST_MM) {
    int cellsAhead = round(frontDist / CELL_SIZE_MM);
    if (cellsAhead < 1) cellsAhead = 1;
    
    float targetDistMM = (cellsAhead * CELL_SIZE_MM) + (CELL_SIZE_MM / 2.0f) - ALIGN_OFFSET_MM;
    float errorMM = targetDistMM - frontDist;

    if (abs(errorMM) > ALIGN_TOLERANCE_MM) {
      if (errorMM > 0) {
         // Robot overshot cell center -> Move backwards
         leftMotor.resetTicks();
         rightMotor.resetTicks();
         float targetTicks = errorMM / MM_PER_TICK;
         while (true) {
             long leftT = abs(leftMotor.getTicks() * LEFT_ENC_POLARITY);
             long rightT = abs(rightMotor.getTicks() * RIGHT_ENC_POLARITY);
             if ((leftT + rightT) / 2 >= targetTicks) break;
             setMotorOutputs(-80, 0); 
             delay(10);
         }
         setMotorOutputs(0, 0);
      } else {
         // Robot undershot cell center -> Move forward
         moveDistanceMM(abs(errorMM), 80);
      }
      delay(50);
    }
  }
}

// ==========================================
// 3. FLOOD FILL ALGORITHM (8x8 CENTER)
// ==========================================
class FloodFill {
public:
    uint8_t dist[MAZE_W][MAZE_H];
    bool walls[MAZE_W][MAZE_H][4]; 

    int dx[4] = {0, 1, 0, -1};
    int dy[4] = {1, 0, -1, 0};

    FloodFill() {
        for (int i = 0; i < MAZE_W; i++) {
            for (int j = 0; j < MAZE_H; j++) {
                for (int k = 0; k < 4; k++) walls[i][j][k] = false;
                if (i == 0) walls[i][j][3] = true;
                if (i == MAZE_W - 1) walls[i][j][1] = true;
                if (j == 0) walls[i][j][2] = true;
                if (j == MAZE_H - 1) walls[i][j][0] = true;
            }
        }
    }

    bool is_out_of_bounds(int x, int y, int dir) {
        int nx = x + dx[dir];
        int ny = y + dy[dir];
        return (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H);
    }

    void add_wall(int x, int y, int dir) {
        if (x >= 0 && x < MAZE_W && y >= 0 && y < MAZE_H) {
            walls[x][y][dir] = true;
            int nx = x + dx[dir], ny = y + dy[dir];
            if (nx >= 0 && nx < MAZE_W && ny >= 0 && ny < MAZE_H) {
                walls[nx][ny][(dir + 2) % 4] = true;
            }
        }
    }

    void remove_wall(int x, int y, int dir) {
        if (x == 0 && dir == 3) return; 
        if (x == MAZE_W - 1 && dir == 1) return; 
        if (y == 0 && dir == 2) return; 
        if (y == MAZE_H - 1 && dir == 0) return; 

        if (x >= 0 && x < MAZE_W && y >= 0 && y < MAZE_H) {
            walls[x][y][dir] = false;
            int nx = x + dx[dir], ny = y + dy[dir];
            if (nx >= 0 && nx < MAZE_W && ny >= 0 && ny < MAZE_H) {
                walls[nx][ny][(dir + 2) % 4] = false;
            }
        }
    }

    bool is_target(int x, int y) {
        return (x == 3 || x == 4) && (y == 3 || y == 4);
    }

    void recalculate_distances() {
        for (int i = 0; i < MAZE_W; i++) {
            for (int j = 0; j < MAZE_H; j++) {
                dist[i][j] = 255;
            }
        }

        int qx[MAZE_W * MAZE_H], qy[MAZE_W * MAZE_H];
        int head = 0, tail = 0;

        int centers[4][2] = {{3,3}, {3,4}, {4,3}, {4,4}};
        for (int c = 0; c < 4; c++) {
            int cx = centers[c][0];
            int cy = centers[c][1];
            dist[cx][cy] = 0;
            qx[tail] = cx; qy[tail] = cy; tail++;
        }

        while (head < tail) {
            int x = qx[head]; int y = qy[head]; head++;
            int d = dist[x][y];

            for (int i = 0; i < 4; i++) {
                if (!walls[x][y][i]) {
                    int nx = x + dx[i], ny = y + dy[i];
                    if (nx >= 0 && nx < MAZE_W && ny >= 0 && ny < MAZE_H) {
                        if (dist[nx][ny] == 255) {
                            dist[nx][ny] = d + 1;
                            qx[tail] = nx; qy[tail] = ny; tail++;
                        }
                    }
                }
            }
        }
    }

    int get_best_move(int x, int y, int current_heading) {
        if (is_target(x, y)) return current_heading;

        int min_d = 255;
        for (int i = 0; i < 4; i++) {
            if (!walls[x][y][i]) {
                int nx = x + dx[i], ny = y + dy[i];
                if (nx >= 0 && nx < MAZE_W && ny >= 0 && ny < MAZE_H) {
                    if (dist[nx][ny] < min_d) min_d = dist[nx][ny];
                }
            }
        }

        if (min_d == 255) return (current_heading + 2) % 4;

        int prefs[4] = {current_heading, (current_heading + 1) % 4, (current_heading + 3) % 4, (current_heading + 2) % 4};
        for (int p = 0; p < 4; p++) {
            int h = prefs[p];
            if (!walls[x][y][h]) {
                int nx = x + dx[h], ny = y + dy[h];
                if (nx >= 0 && nx < MAZE_W && ny >= 0 && ny < MAZE_H) {
                    if (dist[nx][ny] == min_d) return h;
                }
            }
        }
        return current_heading;
    }
};

// ==========================================
// 4. MAZE STATE MACHINE & VARIABLES
// ==========================================
enum RobotState {
  EVALUATE_CELL,
  DEBUG_PAUSE,
  TURNING,
  MOVING_FORWARD,
  BRAKING,
  FINISHED
};

RobotState currentState = EVALUATE_CELL;
RobotState nextState = MOVING_FORWARD;

FloodFill maze;
int grid_x = start_x;
int grid_y = start_y;
int current_heading = 0; 

float targetAngleDegrees = 0.0f;
uint32_t pauseStartTime = 0;
uint32_t lastTime = 0;

long targetEncoderDiff = 0;
bool useEncoderHeading = false;

// --- OPENING LATCH FLAGS ---
bool rightOpeningLatched = false;
bool leftOpeningLatched = false;

void turnAnglePID(float targetAngle, int maxSpeed) {
  leftMotor.resetTicks();
  rightMotor.resetTicks();
  float targetTicks = ((PI * TRACK_WIDTH_MM) * (targetAngle / 360.0f)) / MM_PER_TICK;
  
  float integral = 0.0f, lastError = targetTicks;
  uint32_t turnLastTime = micros(), turnStartTime = millis();
  int settledCount = 0;

  while (true) {
    uint32_t currentTime = micros();
    float dt = (currentTime - turnLastTime) / 1000000.0f;
    if (dt >= 0.01f) {
      turnLastTime = currentTime;
      long leftTicks = leftMotor.getTicks() * LEFT_ENC_POLARITY;
      long rightTicks = rightMotor.getTicks() * RIGHT_ENC_POLARITY;
      
      float turnError = targetTicks - ((leftTicks - rightTicks) / 2.0f);
      if (abs(turnError) < 2.0f) { if (++settledCount >= 5) break; } else settledCount = 0;
      if (millis() - turnStartTime > 3000) break; 

      integral = constrain(integral + (turnError * dt), -100.0f, 100.0f); 
      float derivative = (turnError - lastError) / dt;
      lastError = turnError;

      float turnSpeed = constrain((Kp * turnError) + (Ki * integral) + (Kd * derivative), -maxSpeed, maxSpeed);
      if (abs(turnSpeed) < 90.0f && abs(turnError) > 2.0f) turnSpeed = (turnSpeed > 0) ? 90.0f : -90.0f;

      float forwardSpeed = (0 - ((leftTicks + rightTicks) / 2.0f)) * K_heading;
      setMotorOutputs(forwardSpeed, turnSpeed);
    }
  }
  setMotorOutputs(0, 0);
}

void setup() {
  Wire.begin(7, 5);
  Serial.begin(115200);

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  leftMotor.begin();
  rightMotor.begin();
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCA), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA), isrRight, RISING);

  tof.begin();
  lastTime = micros();
  Serial.println("8x8 Maze Solver Ready. Starting in 3 seconds...");
  delay(3000); 
}

void loop() {
  if (currentState == FINISHED) {
      setMotorOutputs(0, 0);
      digitalWrite(LED_BLUE, HIGH); 
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_ORANGE, HIGH);
      
      playVictoryMelody(BUZZER_PIN);
      return; 
  }
  
  if (currentState == DEBUG_PAUSE) {
      setMotorOutputs(0, 0);
      if (millis() - pauseStartTime >= DEBUG_PAUSE_MS) {
          if (nextState == MOVING_FORWARD) {
              leftMotor.resetTicks();
              rightMotor.resetTicks();
              centerPID.reset();
              useEncoderHeading = false;
              targetEncoderDiff = 0;
              
              rightOpeningLatched = false;
              leftOpeningLatched = false;
              digitalWrite(LED_BLUE, LOW);
              digitalWrite(LED_RED, LOW);
              digitalWrite(LED_ORANGE, LOW);
          }
          currentState = nextState;
          lastTime = micros();
      }
      return;
  }

  if (currentState == TURNING) {
      turnAnglePID(targetAngleDegrees, 150); 
      leftMotor.resetTicks(); 
      rightMotor.resetTicks();
      centerPID.reset();
      useEncoderHeading = false;
      targetEncoderDiff = 0;
      
      rightOpeningLatched = false;
      leftOpeningLatched = false;
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_ORANGE, LOW);

      currentState = MOVING_FORWARD;
      lastTime = micros();
      return; 
  }

  tof.update();
  uint32_t currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0f;
  
  if (dt >= 0.01f) { 
    lastTime = currentTime;
    
    float frontDist = tof.getFrontMm();
    float leftDist  = tof.getLeftLatMm();
    float rightDist = tof.getRightLatMm();

    long leftTicks = leftMotor.getTicks() * LEFT_ENC_POLARITY;
    long rightTicks = rightMotor.getTicks() * RIGHT_ENC_POLARITY;
    float dist_moved = ((leftTicks + rightTicks) / 2.0f) * MM_PER_TICK;
    long currentEncoderDiff = rightTicks - leftTicks;

    if (dist_moved <= MAX_SCAN_DIST_MM) {
        if (rightDist >= SIDE_WALL_THRESHOLD_MM) {
            rightOpeningLatched = true;
            digitalWrite(LED_BLUE, HIGH); 
        }

        if (leftDist >= SIDE_WALL_THRESHOLD_MM) {
            leftOpeningLatched = true;
            digitalWrite(LED_RED, HIGH); 
        }
    }
    
    float forwardSpeed = 0.0f;
    float angular_correction = 0.0f;

    // ========================================================
    // STATE 1: EVALUATE CELL
    // ========================================================
    if (currentState == EVALUATE_CELL) {
        bool wereInFirstCell = isFirstCell; // Save first cell status

        if (maze.is_target(grid_x, grid_y)) {
            Serial.println("MAZE CENTER REACHED!");
            currentState = FINISHED;
            return;
        }

        if (leftDist < TOO_CLOSE_WALL_MM) {
            turnAnglePID(-20.0f, 120); 
            moveDistanceMM(40.0f, 90);  
        } else if (rightDist < TOO_CLOSE_WALL_MM) {
            turnAnglePID(20.0f, 120);  
            moveDistanceMM(40.0f, 90);  
        }

        bool hasFrontWall = (frontDist < 150.0f);
        bool hasRightWall, hasLeftWall;

        if (isFirstCell) {
            hasRightWall = (rightDist < SIDE_WALL_THRESHOLD_MM);
            hasLeftWall  = (leftDist < SIDE_WALL_THRESHOLD_MM);
            isFirstCell = false; 
        } else {
            hasRightWall = !rightOpeningLatched;
            hasLeftWall  = !leftOpeningLatched;
        }

        int front_dir = current_heading;
        int right_dir = (current_heading + 1) % 4;
        int left_dir  = (current_heading + 3) % 4;

        bool detectedUnreachableOpening = false;
        if (!hasFrontWall && maze.is_out_of_bounds(grid_x, grid_y, front_dir)) detectedUnreachableOpening = true;
        if (!hasRightWall && maze.is_out_of_bounds(grid_x, grid_y, right_dir)) detectedUnreachableOpening = true;
        if (!hasLeftWall  && maze.is_out_of_bounds(grid_x, grid_y, left_dir))  detectedUnreachableOpening = true;

        if (detectedUnreachableOpening) {
            Serial.println("DEBUG: Saw opening outside maze bounds!");
            digitalWrite(LED_ORANGE, HIGH);
            delay(1000); 
            digitalWrite(LED_ORANGE, LOW);
        }

        if (hasFrontWall) maze.add_wall(grid_x, grid_y, front_dir);
        else maze.remove_wall(grid_x, grid_y, front_dir);

        if (hasRightWall) maze.add_wall(grid_x, grid_y, right_dir);
        else maze.remove_wall(grid_x, grid_y, right_dir);

        if (hasLeftWall) maze.add_wall(grid_x, grid_y, left_dir);
        else maze.remove_wall(grid_x, grid_y, left_dir);

        maze.recalculate_distances();
        int best_dir = maze.get_best_move(grid_x, grid_y, current_heading);

        int diff = (best_dir - current_heading + 4) % 4;

        // --- STRICT CONDITIONAL ALIGNMENT ---
        // 1. Not the first cell
        // 2. Turning required into a side opening (diff != 0)
        // 3. No wall immediately ahead (frontDist >= 250.0f)
        if (!wereInFirstCell && diff != 0 && frontDist >= 250.0f) {
            alignToFrontWall();
        }

        if (diff == 0) {
            nextState = MOVING_FORWARD;
        } else if (diff == 1) {
            targetAngleDegrees = -90.0f;
            nextState = TURNING;
        } else if (diff == 3) {
            targetAngleDegrees = 90.0f;
            nextState = TURNING;
        } else if (diff == 2) {
            targetAngleDegrees = 180.0f;
            nextState = TURNING;
        }

        current_heading = best_dir; 
        
        pauseStartTime = millis();
        currentState = DEBUG_PAUSE; 
    }
    
    // ========================================================
    // STATE 2: MOVING FORWARD
    // ========================================================
    else if (currentState == MOVING_FORWARD) {
        forwardSpeed = baseSpeed;

        bool hasLeftWall  = (leftDist < SIDE_WALL_THRESHOLD_MM);
        bool hasRightWall = (rightDist < SIDE_WALL_THRESHOLD_MM);

        if (hasLeftWall && hasRightWall) {
            float error = leftDist - rightDist; 
            angular_correction = centerPID.compute(error, dt);

            targetEncoderDiff = currentEncoderDiff;
            useEncoderHeading = false;
        } 
        else if (hasLeftWall && !hasRightWall) {
            float error = (leftDist - TARGET_WALL_DIST_MM) * 2.0f;
            angular_correction = centerPID.compute(error, dt);

            targetEncoderDiff = currentEncoderDiff;
            useEncoderHeading = false;
        } 
        else if (!hasLeftWall && hasRightWall) {
            float error = (TARGET_WALL_DIST_MM - rightDist) * 2.0f;
            angular_correction = centerPID.compute(error, dt);

            targetEncoderDiff = currentEncoderDiff;
            useEncoderHeading = false;
        } 
        else {
            if (!useEncoderHeading) {
                useEncoderHeading = true;
                centerPID.reset(); 
            }
            float headingError = currentEncoderDiff - targetEncoderDiff;
            angular_correction = headingError * K_heading; 
        }

        bool wallAheadDetected = (frontDist <= 250.0f);
        bool encoderDistanceReached = (dist_moved >= (CELL_SIZE_MM - BRAKING_DIST_MM));

        if (wallAheadDetected || encoderDistanceReached) {
            currentState = BRAKING;
            stoppingPID.reset();
        }
    }
    
    // ========================================================
    // STATE 3: BRAKING
    // ========================================================
    else if (currentState == BRAKING) {
        float headingError = currentEncoderDiff - targetEncoderDiff;
        angular_correction = headingError * K_heading; 
        
        float linearError = 0.0f;
        
        if (frontDist <= 250.0f) {
            linearError = frontDist - FRONT_STOP_DIST_MM;
        } else {
            linearError = CELL_SIZE_MM - dist_moved;
        }
        
        float pidOutput = stoppingPID.compute(linearError, dt);
        forwardSpeed = min(baseSpeed, max(0.0f, pidOutput));

        if (forwardSpeed > 0.0f && forwardSpeed < MIN_MOTOR_SPEED) {
            forwardSpeed = MIN_MOTOR_SPEED;
        }

        bool frontWallStop = (frontDist <= 250.0f) && (linearError <= 8.0f);
        bool encoderStop   = (dist_moved >= CELL_SIZE_MM - 5.0f);

        if (frontWallStop || encoderStop) {
            forwardSpeed = 0.0f;
            setMotorOutputs(0, 0); 
            delay(150); 
            
            grid_x += maze.dx[current_heading];
            grid_y += maze.dy[current_heading];
            
            currentState = EVALUATE_CELL; 
        }
    }

    setMotorOutputs(forwardSpeed, angular_correction);
  }
}