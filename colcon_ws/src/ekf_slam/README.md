# ekf_slam
## How to launch ekm slam?
```
cd ~/colcon_ws/src/ekf_slam/ekf_slam/src
python3 main.py
```
## How to get the 3D position of a point relative to the drone?
```py
# Read the depth value (Z) at (u,v)
d = float(depth[self.v, self.u])

# Back‑project to camera frame
X = (self.u - self.cx) * d / self.fx
Y = (self.v - self.cy) * d / self.fy
Z = d
```

# projected points
Not transformed

(   x    -y
    y -> -x
    z -> drone_alt - z
)
