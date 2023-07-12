Example of MTU6050 usage.

Multiple branch for absolute position of sensor
Every branch has a pic of the sensor and his position according to surface

Basically need to change according to absolute position of sensor related to surface  
if u need to rotate over the Z just add - before the value so it will look like it rotated of 180°
```
  acceleration[2] = AccX / norm_g;
  acceleration[1] = AccY/norm_g;
  acceleration[0] = AccZ/norm_g;
```

![lateral](https://github.com/meuhle/mpu6050/assets/99694191/e28da181-5f1e-43bf-bcaf-d099b9e11b22)
