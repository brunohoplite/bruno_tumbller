
#pragma once

class Motor {
    unsigned aInPin;
    unsigned pwmAPin;
    unsigned bInPin;
    unsigned pwmBPin;
    unsigned standbyPin;

  public:
    Motor(unsigned, unsigned, unsigned, unsigned, unsigned);
    void forward(unsigned);
    void backward(unsigned);
    void stop(void);
};