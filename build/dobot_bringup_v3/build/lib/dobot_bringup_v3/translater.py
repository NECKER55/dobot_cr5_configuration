'''
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from trajectory_msgs.msg import JointTrajectory
from dobot_msgs_v3.srv import JointMovJ, EnableRobot, ClearError, ServoJ, SpeedJ, CP, SpeedFactor, AccJ
import time
import math


class FakeController(Node):

    def __init__(self):
        super().__init__('fake_trajectory_controller')

        self.server = ActionServer(
            self,  # se stesso (action server in questo nodo)
            FollowJointTrajectory, # messaggio che contiene: goal, feedback, result
            '/cr5_group_controller/follow_joint_trajectory',  # deve combaciare col nome nel controller.yaml, canale che espone l'action server (diverso da un topic)
            self.execute_callback # funzione che esegue ogni volta che arriva un goal
        )

        self.joint_movj_cli = self.create_client(JointMovJ, '/dobot_bringup_v3/srv/JointMovJ')
        while not self.joint_movj_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for JointMovJ service...')
        
        self.servoj_cli = self.create_client(ServoJ, '/dobot_bringup_v3/srv/ServoJ')
        while not self.servoj_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ServoJ service...')

        self.enable_robot_cli = self.create_client(EnableRobot, '/dobot_bringup_v3/srv/EnableRobot')
        while not self.enable_robot_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for EnableRobot service...')

        self.clear_error_cli = self.create_client(ClearError, '/dobot_bringup_v3/srv/ClearError')
        while not self.clear_error_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ClearError service...')


        self.speed_factor_cli = self.create_client(SpeedFactor, '/dobot_bringup_v3/srv/SpeedFactor')
        while not self.speed_factor_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SpeedFactor service...')

        self.CP_cli = self.create_client(CP, '/dobot_bringup_v3/srv/CP')
        while not self.CP_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for CP service...')

        self.speed_cli = self.create_client(SpeedJ, '/dobot_bringup_v3/srv/SpeedJ')
        while not self.speed_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SpeedJ service...')
        
        self.acc_cli = self.create_client(AccJ, '/dobot_bringup_v3/srv/AccJ')
        while not self.acc_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for AccJ service...')


        self.robot_enabled = False

    def _call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def execute_callback(self, goal_handle):  # goal_handle goal effettivo
        self.get_logger().info('Received trajectory from MoveIt!')
        start_time = time.time()

        # Chiama i servizi essenziali all'inizio di ogni esecuzione della traiettoria
        if not self.robot_enabled:
            clear_error_request = ClearError.Request()
            clear_error_result = self._call_service(self.clear_error_cli, clear_error_request)
            if clear_error_result is not None and clear_error_result.res == 0:
                self.get_logger().info('Cleared robot errors.')
            else:
                self.get_logger().warn('Failed to clear robot errors.')
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            enable_robot_request = EnableRobot.Request()
            enable_robot_request.load = 1.0  # Potrebbe essere un altro valore
            enable_robot_result = self._call_service(self.enable_robot_cli, enable_robot_request)
            if enable_robot_result is not None and enable_robot_result.res == 0:
                self.get_logger().info('Robot enabled.')
                self.robot_enabled = True
            else:
                self.get_logger().error('Failed to enable robot.')
                goal_handle.abort()
                return FollowJointTrajectory.Result()
        else:
            self.get_logger().info('Robot is already enabled.')
        
        req_SpeedFactor = SpeedFactor.Request()
        req_SpeedFactor.ratio = 100
        future = self.speed_factor_cli.call_async(req_SpeedFactor)  # chiamata asincrona per non bloccare il nodo (tanto non ne viene effettuata un'altra prima che venga completata)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is not None:
            self.get_logger().info(f"Response from SpeedFactor: {res}")
        else:
            self.get_logger().warn("SpeedFactor service call failed.")
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        
        req_CP = CP.Request()
        req_CP.r = 100
        future = self.CP_cli.call_async(req_CP)  # chiamata asincrona per non bloccare il nodo (tanto non ne viene effettuata un'altra prima che venga completata)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is not None:
            self.get_logger().info(f"Response from CP: {res}")
        else:
            self.get_logger().warn("CP service call failed.")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        req_Speed = SpeedJ.Request()
        req_Speed.r = 100
        future = self.speed_cli.call_async(req_Speed)  # chiamata asincrona per non bloccare il nodo (tanto non ne viene effettuata un'altra prima che venga completata)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is not None:
            self.get_logger().info(f"Response from SpeedJ: {res}")
        else:
            self.get_logger().warn("SpeedJ service call failed.")
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        

        req_Acc = AccJ.Request()
        req_Acc.r = 100
        future = self.acc_cli.call_async(req_Acc)  # chiamata asincrona per non bloccare il nodo (tanto non ne viene effettuata un'altra prima che venga completata)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is not None:
            self.get_logger().info(f"Response from AccJ: {res}")
        else:
            self.get_logger().warn("AccJ service call failed.")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        traj = goal_handle.request.trajectory  # estrazione traiettoria dal goal

        for idx, point in enumerate(traj.points):


            self.get_logger().info("VELOCITY" + str(point.velocities))
            self.get_logger().info("TIMESTAMP" + str(point.time_from_start))

            pos = list(point.positions)

            if len(pos) < 6:
                self.get_logger().error(f"Point {idx}: Expected 6 joints, got {len(pos)}")
                continue
            

            for i in range(len(pos)):
                pos[i] = math.degrees(pos[i])
                
            # Crea e invia la richiesta di movimento
            req = JointMovJ.Request()
            req.j1, req.j2, req.j3, req.j4, req.j5, req.j6 = pos
            
            speed_value = "100"  
            accel_value = "100"  

            req.param_value = [f"SpeedJ={speed_value}", f"AccJ={accel_value}"]
            
            self.get_logger().info(str(pos))
            self.get_logger().info(str(req))
            future = self.joint_movj_cli.call_async(req)  # chiamata asincrona per non bloccare il nodo (tanto non ne viene effettuata un'altra prima che venga completata)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()

            # 🔧 Log del risultato
            if res is not None:
                self.get_logger().info(f"Response from JointMovJ: {res}")
            else:
                self.get_logger().warn("JointMovJ service call failed.")
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            #time.sleep(0.5) # attesa ANDIAMO SUL SICURO

        goal_handle.succeed() # notifica il client avvisandolo che il goal è stato completato
        return FollowJointTrajectory.Result()  # restituisce il risultato dell'azione (varie info, errori ecc)

def main(args=None):
    rclpy.init(args=args) # inizializza ros2
    node = FakeController() # crea il nodo
    rclpy.spin(node) # lancia il nodo e lo fa girare fino a quando chiamiamo

    # in questo caso mai usate qui solo per buona pratica
    node.destroy_node() # libera le risorse usate dal nodo
    rclpy.shutdown() # interrompe il nodo (generalmente lo si mette dentro al nodo per farlo terminare per poi distruggerlo fuori dal nodo)

if __name__ == '__main__': # python quando importa un file assegna alla var __name__ il nome del modulo, se lo esegue invece assegna __name__ == '__main__', questo if serve a eseguire un file solo se necessario e non ad ogni importazione
    main()

'''



















import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from trajectory_msgs.msg import JointTrajectory
from dobot_msgs_v3.srv import JointMovJ, EnableRobot, ClearError, ServoJ, SpeedJ, CP, SpeedFactor, AccJ
import time
import math


class FakeController(Node):

    def __init__(self):
        super().__init__('fake_trajectory_controller')

        self.server = ActionServer(
            self,  # se stesso (action server in questo nodo)
            FollowJointTrajectory, # messaggio che contiene: goal, feedback, result
            '/cr5_group_controller/follow_joint_trajectory',  # deve combaciare col nome nel controller.yaml, canale che espone l'action server (diverso da un topic)
            self.execute_callback # funzione che esegue ogni volta che arriva un goal
        )

        self.joint_movj_cli = self._create_service_client(JointMovJ, '/dobot_bringup_v3/srv/JointMovJ')
        self.servoj_cli = self._create_service_client(ServoJ, '/dobot_bringup_v3/srv/ServoJ')
        self.enable_robot_cli = self._create_service_client(EnableRobot, '/dobot_bringup_v3/srv/EnableRobot')
        self.clear_error_cli = self._create_service_client(ClearError, '/dobot_bringup_v3/srv/ClearError')
        self.speed_factor_cli = self._create_service_client(SpeedFactor, '/dobot_bringup_v3/srv/SpeedFactor')
        self.CP_cli = self._create_service_client(CP, '/dobot_bringup_v3/srv/CP')
        self.speed_cli = self._create_service_client(SpeedJ, '/dobot_bringup_v3/srv/SpeedJ')
        self.acc_cli = self._create_service_client(AccJ, '/dobot_bringup_v3/srv/AccJ')

        self.primary_setup() # pulisce dagli errori, attiva il dobot, setta speed_factor a 100, attiva il CP

    def _create_service_client(self, srv_type, srv_name):
        cli = self.create_client(srv_type, srv_name)
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {srv_name} service...')
        return cli

    def primary_setup(self):
        clear_error_request = ClearError.Request()
        self._call_service(self.clear_error_cli, clear_error_request, "clear_error")

        enable_robot_request = EnableRobot.Request()
        enable_robot_request.load = 1.0  # Potrebbe essere un altro valore
        self._call_service(self.enable_robot_cli, enable_robot_request, "enable_robot")

        req_SpeedFactor = SpeedFactor.Request() # fattore di riduzione di tutte le velocità o accellerazioni
        req_SpeedFactor.ratio = 100
        self._call_service(self.speed_factor_cli, req_SpeedFactor,"speed_factor")

        req_CP = CP.Request()  # continous path
        req_CP.r = 100
        self._call_service(self.CP_cli, req_CP,"CP")

        req_Speed = SpeedJ.Request() # fattore velocità joint
        req_Speed.r = 100
        self._call_service(self.speed_cli, req_Speed,"speedJ")

        req_acc = AccJ.Request() # fattore accellerazione joint
        req_acc.r = 100
        self._call_service(self.acc_cli, req_acc,"accJ")

    def _call_service(self, client, request, service_name): # self passato automaticamente
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is not None:
            self.get_logger().info(f"Response from {service_name}: {result}")
            return True
        else:
            self.get_logger().warn(f'Fallita l\'esecuzione di {service_name}.')
            return False
        
    def execute_callback(self, goal_handle):  # goal_handle goal effettivo
        self.get_logger().info('Received trajectory from MoveIt!')


        traj = goal_handle.request.trajectory  # estrazione traiettoria dal goal

        for idx, point in enumerate(traj.points):

            pos = list(point.positions)

            if len(pos) < 6:
                self.get_logger().error(f"Point {idx}: Expected 6 joints, got {len(pos)}")
                return FollowJointTrajectory.Result()
            
            pos = [math.degrees(p) for p in point.positions[:6]]
                
            # Crea e invia la richiesta di movimento
            req = JointMovJ.Request()
            req.j1, req.j2, req.j3, req.j4, req.j5, req.j6 = pos
            
            
            self.get_logger().info("richiesta inviata: " + str(req))

            result = self._call_service(self.joint_movj_cli, req,"jointMovJ")
            if not result:
                return FollowJointTrajectory.Result()

        goal_handle.succeed() # notifica il client avvisandolo che il goal è stato completato
        return FollowJointTrajectory.Result()  # restituisce il risultato dell'azione (varie info, errori ecc)

def main(args=None):
    rclpy.init(args=args) # inizializza ros2
    node = FakeController() # crea il nodo
    rclpy.spin(node) # lancia il nodo e lo fa girare fino a quando chiamiamo

    # in questo caso mai usate qui solo per buona pratica
    node.destroy_node() # libera le risorse usate dal nodo
    rclpy.shutdown() # interrompe il nodo (generalmente lo si mette dentro al nodo per farlo terminare per poi distruggerlo fuori dal nodo)

if __name__ == '__main__': # python quando importa un file assegna alla var __name__ il nome del modulo, se lo esegue invece assegna __name__ == '__main__', questo if serve a eseguire un file solo se necessario e non ad ogni importazione
    main()











