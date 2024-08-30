# Zigler-Nichols

Ku is the Proportional gain that, with 0 Integral and Derivative gains, cause the system to stably oscillated at period Tu

Ki = Kp/Ti

Kd = Kp*Td

| Control Type |   Kp    |   Ti    |    Td    |     Ki     |      Kd       |
|:------------:|:-------:|:-------:|:--------:|:----------:|:-------------:|
|      P       | 0.5*Ku  |    -    |    -     |     -      |       -       |
|      PI      | 0.45*Ku | 0.83*Tu |    -     | 0.54*Ku/Tu |       -       |
|      PD      | 0.8*Ku  |    -    | 0.125*Tu |     -      |  0.10\*Ku*Tu  |
| classic PID  | 0.6*Ku  | 0.5*Tu  | 0.125*Tu | 1.2*Ku/Tu  |  0.10\*Ku*Tu  |
| no overshoot | 0.2*Ku  | 0.5*Tu  | (1/3)*Tu | 0.40*Ku/Tu | (2/30)\*Ku*Tu |

Reference: https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

## no overshoot:

Kp: Ku = 650 => Kp = 0.2*Ku = 130 

Ki: Ku = 650, Tu = 300ms = 0.3s => Ki = 0.4*Ku/Tu = 0.4*650*.3 = 78

Kd: Ku = 650, Tu = 300ms = 0.3s => Kd = (2/30)\*Ku*Tu = (1/15)\*650*.3 = 13
