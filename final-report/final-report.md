# State Diagrams

The overall strategy of our robot was to turn towards the IR beacon, stopping when we saw a certain amplitude of the filtered signal, then follow the line towards the bad press, followed by following the line to the good press and depositing two balls at each of those.

Our dispensing mechanism was a tube of four press, with one servo at the bottom which prevented the press from falling out. We could drop two press only by opening and closing the servo at a specific time. Originally, our finite state machines assumed that we would have two servos to form a kind of airlock system, having only one press to drop at a time.

We had also wanted to use an IMU for more consistent turning, but removed it to avoid additional points of failure on our robot. Instead, we turned for approximate amounts of time and depended on sensing the tape.

The following FSMs are our preliminary ones, the final code was streamlined and relied more on timers rather than only sensor inputs.

## Exiting Studio
For our final code, we did the following:
- Start in the studio
- Turn until the ir beacon reached a threshold
- Drive forward for some time (hoping to exit the box surrounded by black tape)
- Follow the red tape

Original FSM included turning 360 degrees to read the highest IR value, then turn towards it, and then detect the black tape to determine when we had left the studio
![Exit studio FSM](exit-studio.jpg)

The following image is our initial FSM for leaving the studio.

## Dispensing Press

For our final code, we opened the bottom servo (only one required) for a set number of milliseconds, then closed the servo.

The following image is our initial FSM for dispesning one press
![Dispensing Press FSM](dispense-press.jpg)

## Line Follow
Our final code used a simplified approach to line follow:
- If the difference between left and right line sensors are between a threshold, go forward with half power
- Otherwise, if the left reading was more than the right reading (left sensor was over the red tape, right was not), set left motor to 0, right motor to half power to turn left
- Otherwise, turn right by setting left motor to half power, right motor to 0

Here is our original FSM, which employed P-only closed loop control (using difference between the left/right and center sensors as the error)
![Line Follow FSM](line-follow.jpg)

## Full FSM for Checkoff
Again, this image is the FSM that references the above state machines, to ultimately complete the goal of dispensing two press in the good press, and then two in the bad press

For our final code, we ended up traveling to the bad press first to throw off opponents who needed the press scales to be level for scoring. We also used timers to replace the gyro, and a simpler line follow method, as described in the line follow section.

![Final FSM](checkoff-fsm.jpg)
