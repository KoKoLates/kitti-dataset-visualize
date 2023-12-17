https://navoshta.com/kitti-lidar/
https://github.com/charlesq34/frustum-pointnets

* The rotation matrix from yaw

$R=\begin{bmatrix}cos(yaw) & 0 & sin(yaw) \\ 0 & 1 & 0 \\ -sin(yaw) & 0 & cos(yaw)\end{bmatrix}$

transformation(`3x4`) = rotation(`3x3`) + translation(`3x1`)

---
### calculate the previous coordinates


---
### Multi-Object Tracking (MOT)
Tracking-by-detector

tracker: all objects should detect <br>
center: current detected object

if center's object in tracker -> update information, else -> create a new object in tracker.
if tracker' object not in center -> update previous information, but not current