/**
 * Wrapper for the hopper subsystem, allows user to drop x number of balls
*/

#ifndef Hopper_h
#define Hopper_h
#include <Servo.h>
#include <inttypes.h>
class Hopper {
  public:
    Hopper(); // Only to initialize at first
    Hopper(Servo &servo, uint8_t servo_port);
    /**
     * NOTE: These functions use delay to get the correct timing (not the best practice)
    */
    void dropTwoBalls();
    void dropAllBalls();
    bool isInitialized();
  private:
    Servo _servo;
    bool _is_attached;
};

#endif
