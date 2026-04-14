#ifndef MotorControl_h
#define MotorControl_h

class MotorControl {
  public:
    MotorControl(int pin1, int pin2, int pin3, int pin4);
    void forward();
    void backward();
    void left90();
    void right90();
    void left();
    void right();
    void stop();
  private:
    int _pin1; //motor1 clock
    int _pin2; //motor1 anti
    int _pin3; //motor2 clock
    int _pin4; //motor2 anti
};

#endif