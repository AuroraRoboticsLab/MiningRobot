/*
 Drive L293D motor shield from serial commands.
*/
#include <AFMotor.h> // Adafruit V1 Motor Shield library

// Create motor objects
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);

void setup() {
  Serial.begin(9600);
  Serial.println("Motor commands");
}

int s = 0; // speed
int ldir = RELEASE; // left wheels
int rdir = RELEASE; // right side motors spin opposite way

const char *dirnames[5]={"nul","forward","backward","brake","release"};

void loop() {

  while (Serial.available()>0) {
    bool set=false;
    char c=Serial.read();
    switch(c) {
    case 'L': set=true; s=255; ldir=BACKWARD; break;
    case 'l': set=true; s=255; ldir=FORWARD; break;

    // right side motors need the opposite drive command
    case 'R': set=true; s=255; rdir=FORWARD; break;
    case 'r': set=true; s=255; rdir=BACKWARD; break;
    
    // spacebar stop
    case ' ': set=true; s=0; ldir = rdir = RELEASE; break; 
    
    case '\n': case '\r': break;
    default:
      Serial.println("Unknown command.  Valid commands: lr (forward) LR (backward) lR (turn right) Lr (turn left) etc.");
      break;
    }
    if (set) {
      Serial.print("Now   s ");
      Serial.print(s);
      Serial.print("   l ");
      Serial.print(dirnames[ldir]);
      Serial.print("   r ");
      Serial.println(dirnames[rdir]);
    }
  }

  M1.setSpeed(s); M1.run(ldir); // left side
  M4.setSpeed(s); M4.run(ldir);

  M2.setSpeed(s); M2.run(rdir); // right side
  M3.setSpeed(s); M3.run(rdir);
  
}

