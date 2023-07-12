Example of MTU6050 usage.

Multiple branch for absolute position of sensor
Every branch has a pic of the sensor and his position according to surface

Basically need to change according to absolute position of sensor related to surface  
if u need to rotate over the Z just add - before the value so it will look like it rotated of 180°
```
  acceleration[0] = -AccX / norm_g;
  acceleration[2] = AccY/norm_g;
  acceleration[1] = -AccZ/norm_g;
```
![stand](https://github.com/meuhle/mpu6050/assets/99694191/c1a31894-6ca5-49dc-b725-afabce2d7319)
