#include "MotorControl.h"
#include <Arduino.h>

MotorControl::MotorControl(int pin1, int pin2, int pin3, int pin4) {
  _pin1 = pin1;
  _pin2 = pin2;
  _pin3 = pin3;
  _pin4 = pin4;
  pinMode(_pin1, OUTPUT);
  pinMode(_pin2, OUTPUT);
  pinMode(_pin3, OUTPUT);
  pinMode(_pin4, OUTPUT);
}

void MotorControl::forward() {
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, LOW);
  digitalWrite(_pin3, HIGH);
  digitalWrite(_pin4, LOW);
}

void MotorControl::backward() {
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, HIGH);
  digitalWrite(_pin3, LOW);
  digitalWrite(_pin4, HIGH);
}

void MotorControl::left90() {
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, LOW);
  digitalWrite(_pin3, LOW);
  digitalWrite(_pin4, HIGH);
}

void MotorControl::right90() {
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, HIGH);
  digitalWrite(_pin3, HIGH);
  digitalWrite(_pin4, LOW);
}

void MotorControl::left() {
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, LOW);
  digitalWrite(_pin3, LOW);
  digitalWrite(_pin4, HIGH);
}

void MotorControl::right() {
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, HIGH);
  digitalWrite(_pin3, HIGH);
  digitalWrite(_pin4, LOW);
}

void MotorControl::stop() {
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, LOW);
  digitalWrite(_pin3, LOW);
  digitalWrite(_pin4, LOW);
}