---
marp: false
theme: uncover
class: invert
math: mathjax
---

# Localization & Control & Behavior

---

## Images
![bg right height:10cm](img/perigo.jpg)
1. 1
1.1. 2
2. 2


---
## Maquina de Estado
```python
class Control():
	def __init__(self):
        self.robot_state = "procura" # Estado atual

        self.robot_machine = {
            "procura": self.procura,
            "aproxima": self.aproxima,
            "para": self.para
        }

    def procura(self) -> None:
        ...
        self.robot_state = "aproxima"

    def aproxima(self) -> None:
        ...
        self.robot_state = "para"

    def para(self) -> None:
        ...
```

---
## Controle Proporcional
$u(t) = K_p*e(t)$
```python
class Follower:
    def __init__(self):
        self.cx = -1
        self.kp = 100
        ...
    def image_callback(self, msg):
        ...
        self.w = image.shape[2]
        ...
        self.cx = int(M['m10']/M['m00'])
    
    def control(self):
        err = self.w/2 - self.cx # e(t)
        self.twist.angular.z = float(err) / self.kp
```

---
## Odometria
Tópico: `/odom`
```python
	def odom_callback(self, data: Odometry):
		self.position = data.pose.pose.position
		
		orientation_list = [data.pose.pose.orientation.x,
							data.pose.pose.orientation.y,
							data.pose.pose.orientation.z,
							data.pose.pose.orientation.w]

		self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)

		# converter o angulo yaw de [-pi, pi] para [0, 2pi]
		self.yaw = self.yaw % (2*np.pi)

```