#!/usr/bin/env python
import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from path_planner.srv import *
from tf.transformations import *
from moveit_msgs.msg import Grasp
from math import pi

class Planner(): # Clase que se encarga de la ejecucion de los movimientos del brazo

  def __init__(self):
    
    # Inicializando move it
    moveit_commander.roscpp_initialize(sys.argv) 
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Se declaran los grupos de links a mover del robot
    group_name = "xarm6"
    group_name_gripper = "xarm_gripper"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group_gripper = moveit_commander.MoveGroupCommander(group_name_gripper)
    planning_frame = move_group.get_planning_frame()
    planning_frame_gripper = move_group_gripper.get_planning_frame()
    eef_link = move_group_gripper.get_end_effector_link()
    group_names = robot.get_group_names()

    # Varibles internas
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.move_group_gripper = move_group_gripper
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    
  # Funcion auxiliar para verificar el acoplamiento adecuado  
  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):
    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      is_known = box_name in scene.get_known_object_names()

      if (box_is_attached == is_attached) and (box_is_known == is_known):
          return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()

      return False

  # Funcion que agrega las cajas y depositos dentro de Rviz
  def addObstacles(self): 
    rospy.sleep(2)
    scene = self.scene
    collision_object = moveit_msgs.msg.CollisionObject()
    
    # Cajas a mover
    targets = ["RedBox",
                "BlueBox",
                "GreenBox"]
    # Depositos
    boxes = ["DepositBoxGreen",
                "DepositBoxRed",
                "DepositBoxBlue"]
    # Medidas
    box_size = [0.04, 0.04, 0.04]
    depo_size = [0.359, 0.172, 0.11]
    
		# Creacion de cajas
    for i in range(3):
        box_pose = PoseStamped()
        box_pose.header.frame_id = targets[i]
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.z = 0
    	box_name = targets[i]
    	scene.add_box(box_name, box_pose , box_size)
    
    # Creacion de depositos
    for i in range(3):
        depo_pose = PoseStamped()
        depo_pose.header.frame_id = boxes[i]
        depo_pose.pose.orientation.w = 1
        depo_pose.pose.position.z = 0
    	depo_name = boxes[i]
    	scene.add_box(depo_name, depo_pose , depo_size)

    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True,box_name=box_name, timeout=0.5)
  
  def removeObjects(self):
    collision_object = moveit_msgs.msg.CollisionObject()
    scene = self.scene
    # Cajas a remover
    targets = ["RedBox",
                "BlueBox",
                "GreenBox"]
    # Depositos a remover
    boxes = ["DepositBoxGreen",
                "DepositBoxRed",
                "DepositBoxBlue"]
    # scene.remove_world_object(box_name)
    for i in range(3):
    	box_name = targets[i]
        rospy.sleep(0.5)
    	scene.remove_world_object(box_name)
        depo_name = boxes[i]    
        rospy.sleep(0.5)	
    	scene.remove_world_object(depo_name)
    return 

  # Funcion para dar la indicacion a moveit del movimiento del brazo 
  def goToPose(self, pose_goal):
    move_group = self.move_group
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose    
    return self.wait_for_state_update(box_is_known=True,box_name=self.box_name, timeout=0.5)

  # Funcion que hace que la caja siga la referencia del gripper, hacemos uso de los servicios dados por Manchester
  def detachBox(self,box_name):
    Attach = rospy.ServiceProxy('AttachObject', AttachObject)
    Attach(0,box_name)

    return self.wait_for_state_update(box_is_known=True,box_name=box_name, timeout=0.5)

	# Funcion que hace que la caja deje de seguir la referencia del gripper, hacemos uso de los servicios dados por Manchester
  def attachBox(self,box_name):
    Attach = rospy.ServiceProxy('AttachObject', AttachObject)
    Attach(1,box_name)

    return self.wait_for_state_update(box_is_known=True,box_name=box_name, timeout=0.5)
 
  # Funcion auxiliar que nos ayuda a hacer que el gripper se abra o cierre tanto en rviz como en gazebo
  def efector(self, action):
    # Si recibe la accion "place" se abre el gripper con un angulo especifico
    if (action == 'place'):
      move_group_gripper = self.move_group_gripper
      joint_goal = move_group_gripper.get_current_joint_values()
      joint_goal[0] = 0.1
      joint_goal[1] = 0.1
      joint_goal[2] = 0.1
      joint_goal[3] = 0.1
      joint_goal[4] = 0.1
      joint_goal[5] = 0.1

      move_group_gripper.go(joint_goal, wait = True)
      move_group_gripper.stop()
    # De otro modo, se cierra el gripper
    else:
      move_group_gripper = self.move_group_gripper
      joint_goal = move_group_gripper.get_current_joint_values()
      joint_goal[0] = 0.2
      joint_goal[1] = 0.2
      joint_goal[2] = 0.2
      joint_goal[3] = 0.2
      joint_goal[4] = 0.2
      joint_goal[5] = 0.2
      move_group_gripper.go(joint_goal, wait = True)
      move_group_gripper.stop()
      self.wait_for_state_update(box_is_known=True,box_name=self.box_name, timeout=0.5)
    return 



class myNode(): # Clase que se encarga de la planificacion del movimiento
  
  def __init__(self):
    # Inicializa el nodo
    rospy.init_node('solution_template', anonymous=True)
    # Espera de servicios
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')
    
  def __del__(self):
    move_group = self.planner.move_group
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/9
    joint_goal[2] = -17*pi/60
    joint_goal[3] = 0
    joint_goal[4] = 71*pi/180
    joint_goal[5] = 0
    move_group.go(joint_goal, wait = True)
    move_group.stop()
    self.planner.removeObjects()
    return

  # Funcion que llama al servicio RequestGoal para obtener la pose a la que queremos ir
  def getGoal(self, action):
    Request_Goal = rospy.ServiceProxy('RequestGoal', RequestGoal)
    pose_goal = Request_Goal(action)

    return self.tf_goal(pose_goal, action)

  # Funcion que nos devuelve la transformada desde la base del robot hasta el objetivo 
  def tf_goal(self, goal, action): 
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    trans = None
    self.planner = Planner()
    
    while not trans:
      try:
        trans = tfBuffer.lookup_transform('link_base', goal.goal, rospy.Time())
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue
    # Conversion a cuaternion que establece la pose final
    pose=geometry_msgs.msg.Pose()
    pose.orientation.x = 1
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 0
    pose.position.x = trans.transform.translation.x
    pose.position.y = trans.transform.translation.y
    # Ajustes de altura para evitar colisones
    if action == 'pick':
      pose.position.z = trans.transform.translation.z + 0.01
    else:
      pose.position.z = trans.transform.translation.z + 0.25
    return pose
    

  # Funcion prinicipal de la clase que contiene la secuencia de acciones del robot
  def main(self):
    self.planner = Planner()
    self.planner.addObstacles()
    self.nodo = myNode()
    
    ############# Caja Roja ###############
    goal0 = self.nodo.getGoal("pick")  ##### va a la caja
    self.planner.goToPose(goal0)
    self.planner.efector("pick")    ##### cierra el gripper
    self.planner.attachBox("RedBox") #### attach 
    
    goal1 = self.nodo.getGoal("place")  #### va al deposito
    self.planner.goToPose(goal1)
    self.planner.efector("place")   #### abre el gripper
    self.planner.detachBox("RedBox")   #### detach
    ############# Caja Azul ##############
    goal0 = self.nodo.getGoal("pick")  #### va hacia la caja 
    self.planner.goToPose(goal0)
    self.planner.efector("pick")       #### cierra el gripper
    self.planner.attachBox("BlueBox")   ### attach
  
    goal1 = self.nodo.getGoal("place")  ### va hacia el deposito
    self.planner.goToPose(goal1)
    self.planner.efector("place")    ##### abre el gripper  
    self.planner.detachBox("BlueBox")   #### detach
    ############# Caja Verde ##############
    goal0 = self.nodo.getGoal("pick")  #### va hacia la caja
    self.planner.goToPose(goal0)
    self.planner.efector("pick")    #### cierra el gripper 
    self.planner.attachBox("GreenBox")   ### attach
    
    goal1 = self.nodo.getGoal("place")  #### va hacia el deposito
    self.planner.goToPose(goal1)    
    self.planner.efector("place")    ##### abre el gripper
    self.planner.detachBox("GreenBox")  #### detach
     
  
    


#Funcion principal del programa
if __name__ == '__main__':
  # Se instancia un objeto de clase myNode y mandamos a llamar el main para dar inicio al programa.
  try:
    node = myNode()
    node.main()
    del node
    rospy.signal_shutdown("Task Completed")

  except rospy.ROSInterruptException:
    pass
