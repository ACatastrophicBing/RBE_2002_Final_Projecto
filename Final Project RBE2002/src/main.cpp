#include <Arduino.h>
#include "Behaviors.h"

Behaviors FinalProject;

void setup() {
  FinalProject.Init();
}

void loop() {
  FinalProject.Run();
}