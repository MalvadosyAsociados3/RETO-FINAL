import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from time import sleep
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class Controller(Node):
    def __init__(self):
        super().__init__("control_node")

        # ---Subscribers---
        # Odometry
        self.odomSub = self.create_subscription(Odometry, "odom", self.odomCB, qos_profile_sensor_data)
        # Current goal
        self.goalSub = self.create_subscription(PoseStamped, "current_goal", self.goalCB, 10)

        # ---Publishers---
        # Control command
        self.velPub = self.create_publisher(Twist, "cmd_vel", 10)
        # Goal reached flag
        self.goalFlagPub = self.create_publisher(Empty, "goal_reached", 10)

        # ---Controller---
        self.control = self.generateFuzz()

        # ---Robot state---
        # Current position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Goals
        self.goalLinTol = 0.10
        self.goalAngtol = 0.05  # ya no se usa para arrival, solo historico
        self.goalReached = False
        self.xGoal = None
        self.yGoal = None

        # ---Timers---
        timerPeriod = 0.05 #s
        self.timer = self.create_timer(timerPeriod, self.timerCB)
        self.stabilizeTime = 0.3 #s
        self.inTolSince = None

        # ---Exit---


    def generateFuzz(self):
        # ---Universes---
        
        dist = ctrl.Antecedent(np.linspace(0, 2, 200), 'distance')
        angle = ctrl.Antecedent(np.linspace(-np.pi, np.pi, 200), 'angle')
        # Outputs: v maxima 0.25 m/s 
        v = ctrl.Consequent(np.linspace(0, 0.25, 200), 'v')
        w = ctrl.Consequent(np.linspace(-1, 1, 200), 'w')

        # ---Membership functions---
        # Distance
        dist['goal']   = fuzz.gaussmf(dist.universe, 0.0, 0.12)
        dist['near']   = fuzz.gaussmf(dist.universe, 0.45, 0.30)
        dist['medium'] = fuzz.gaussmf(dist.universe, 1.10, 0.40)
        dist['far']    = fuzz.gaussmf(dist.universe, 2.0,  0.40)
        
        # al 60-70% incluso con aligned=0.9, y el defuzz mete un w residual que
        # curva los segmentos rectos)
        angle['very_right'] = fuzz.gaussmf(angle.universe, -np.pi, 0.5)
        angle['right']      = fuzz.gaussmf(angle.universe, -1.047, 0.5)
        angle['aligned']    = fuzz.gaussmf(angle.universe, 0.0, 0.25)
        angle['left']       = fuzz.gaussmf(angle.universe, 1.047, 0.5)
        angle['very_left']  = fuzz.gaussmf(angle.universe, np.pi, 0.5)
        # Linear velocity (universo 0-0.25 m/s)
        v['stop']   = fuzz.gaussmf(v.universe, 0.0,  0.03)
        v['slow']   = fuzz.gaussmf(v.universe, 0.07, 0.03)
        v['medium'] = fuzz.gaussmf(v.universe, 0.15, 0.035)
        v['fast']   = fuzz.gaussmf(v.universe, 0.22, 0.03)
        # Angular velocity
        w['hard_right'] = fuzz.gaussmf(w.universe, -0.872, 0.1305)
        w['right']      = fuzz.gaussmf(w.universe, -0.436, 0.174)
        w['zero']       = fuzz.gaussmf(w.universe, 0.0, 0.087)
        w['left']       = fuzz.gaussmf(w.universe, 0.436, 0.174)
        w['hard_left']  = fuzz.gaussmf(w.universe, 0.872, 0.1305)

        # ---Rules---
        rules = []

        # Goal reached → full stop
        rules.append(ctrl.Rule(dist['goal'], (v['stop'], w['zero'])))
         # Aligned motion
        rules += [
            ctrl.Rule(dist['near']   & angle['aligned'], (v['slow'],   w['zero'])),
            ctrl.Rule(dist['medium'] & angle['aligned'], (v['medium'], w['zero'])),
            ctrl.Rule(dist['far']    & angle['aligned'], (v['fast'],   w['zero']))
        ]
        # Right corrections: angulo desalineado -> giro en sitio (v=stop)
        # para que las esquinas salgan como esquinas y no como arcos.
        rules += [
            ctrl.Rule(dist['near']   & angle['right'], (v['stop'], w['right'])),
            ctrl.Rule(dist['medium'] & angle['right'], (v['stop'], w['right'])),
            ctrl.Rule(dist['far']    & angle['right'], (v['slow'], w['right']))
        ]
        # Left corrections: idem para lados opuestos
        rules += [
            ctrl.Rule(dist['near']   & angle['left'], (v['stop'], w['left'])),
            ctrl.Rule(dist['medium'] & angle['left'], (v['stop'], w['left'])),
            ctrl.Rule(dist['far']    & angle['left'], (v['slow'], w['left']))
        ]
        # Extreme angles
        rules += [
            ctrl.Rule(angle['very_right'], (v['stop'], w['hard_right'])),
            ctrl.Rule(angle['very_left'],  (v['stop'], w['hard_left']))
        ]

        # ---System---
        system = ctrl.ControlSystem(rules)
        sim = ctrl.ControlSystemSimulation(system)

        return sim

    def computeError(self):
        # Position difference
        dx = self.xGoal - self.x
        dy = self.yGoal - self.y

        # Linear distance to goal
        dist = np.hypot(dx, dy)

        # Angular difference to goal
        desired = np.arctan2(dy, dx)
        eAng = desired - self.theta
        # Norm to [-Pi, +Pi]
        eAng = np.arctan2(np.sin(eAng), np.cos(eAng))

        # Return linear and angular errors
        return dist, eAng
    
    def computeFuzzy(self, dist, eAng):
        # Compute control signal from inputs
        try:
            self.control.input['distance'] = float(dist)
            self.control.input['angle'] = float(eAng)

            self.control.compute()

            v = self.control.output['v']
            w = self.control.output['w']

        # If any error occurs, guarantee the robot stops
        except Exception as e: 
            self.get_logger().warn(f"Fuzzy error: {e}")
            return 0.0, 0.0
        if np.isnan(v) or np.isnan(w):
            self.get_logger().warn("NaN detected in fuzzy output")
            return 0.0, 0.0

        # Clip output to a max of 1 in any velocity
        v = float(np.clip(v, 0.0, 1.0))
        w = float(np.clip(w, -1.0, 1.0))

        # Deadband: suprime w residual del defuzz en tramos rectos.
        # Cualquier giro intencional excede este umbral facilmente.
        if abs(w) < 0.08:
            w = 0.0

        return v, w
    
    def publishCmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.velPub.publish(msg)

    # ---Callbacks---
    def odomCB(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        # yaw (Z) from quaternion
        sinyCosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosyCosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = np.arctan2(sinyCosp, cosyCosp)

    def goalCB(self, msg):
        newX = msg.pose.position.x
        newY = msg.pose.position.y

        # If no goal, just accept it
        if self.xGoal is None:
            self.xGoal, self.yGoal = newX, newY
            self.goalReached = False
            return 

        # ---Distance between goals---
        # Linear distance
        d = np.hypot(newX - self.xGoal, newY - self.yGoal)
        # Angular distance
        angOld = np.arctan2(self.yGoal - self.y, self.xGoal - self.x)
        angNew = np.arctan2(newY - self.y, newX - self.x)
        dAng = np.arctan2(np.sin(angNew - angOld), np.cos(angNew - angOld))
        # Only accept goal if enough different from previous
        if d < self.goalLinTol and abs(dAng) < self.goalAngtol:
            return
        
        self.xGoal, self.yGoal = newX, newY
        self.goalReached = False


    def timerCB(self):
        # If no goal, do nothing
        if self.xGoal is None:
            return
        
        # Errors
        linErr, angErr = self.computeError()

        # Verify if in tolerance
        # Solo usamos la distancia lineal: cerca del goal el angulo se vuelve
        # hipersensible (5 cm desviado -> 30-90 deg) y nunca cumpliria la
        # tolerancia angular, haciendo que el robot orbite sin confirmar goal.
        inTol = (linErr < self.goalLinTol)
        now = self.get_clock().now().nanoseconds * 1e-9
        if inTol:
            if self.inTolSince is None:
                self.inTolSince = now
            elif (now - self.inTolSince) >= self.stabilizeTime:
                # Arrived while stable
                if not self.goalReached:
                    self.goalReached = True
                    self.goalFlagPub.publish(Empty())
                
                self.velPub.publish(Twist())
                return
        else: # Reset tolerance 
            self.inTolSince = None
            self.goalReached = False
        
        # Control action
        v, w = self.computeFuzzy(linErr, angErr)
        self.publishCmd(v, w)

def main(args=None):
    rclpy.init(args=args)

    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stopMsg = Twist()
        node.velPub.publish(stopMsg)
        sleep(0.1)

        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()