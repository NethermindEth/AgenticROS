# agentic_ros/core.py

import rospy
from std_msgs.msg import String
from crewai import Agent, Task, Crew
from typing import List, Dict, Any
import json
from functools import wraps

class ContextManager:
    """Maintains system state and context"""
    def __init__(self):
        self.state = {}
        self.history = []
        
    def update_state(self, new_state: Dict[str, Any]):
        self.state.update(new_state)
        self.history.append(new_state)
        
    def get_context(self) -> str:
        """Returns formatted context for LLM consumption"""
        return f"Current State: {json.dumps(self.state)}\nRecent History: {self.history[-5:]}"

class CognitiveLayer:
    """Handles LLM-based reasoning"""
    def __init__(self):
        self.task_analyzer = Agent(
            role='Task Analyzer',
            goal='Break down high-level tasks into specific robot actions',
            backstory='Expert in robotics task planning and decomposition'
        )
        
        self.behavior_generator = Agent(
            role='Behavior Generator',
            goal='Generate specific robot behaviors and actions',
            backstory='Specialist in converting plans into executable robot behaviors'
        )
        
        self.crew = Crew(
            agents=[self.task_analyzer, self.behavior_generator],
            tasks=[]
        )
    
    def analyze_task(self, task_description: str, context: str) -> Dict:
        """Analyze task and generate execution plan"""
        analysis_task = Task(
            description=f"""
            Analyze this robot task: {task_description}
            Context: {context}
            Break it down into specific steps and required actions.
            Return JSON with 'steps' and 'required_capabilities'.
            """
        )
        
        result = self.crew.kickoff()
        return json.loads(result)

class ROSTranslator:
    """Translates high-level commands to ROS actions"""
    def __init__(self):
        self.publishers = {}
        self.action_clients = {}
        
    def register_publisher(self, topic: str, msg_type):
        self.publishers[topic] = rospy.Publisher(topic, msg_type, queue_size=10)
        
    def to_ros_message(self, command: Dict) -> Dict:
        """Convert high-level command to ROS message"""
        # In POC, just forward the command
        return command

class ActionExecutor:
    """Executes ROS actions and monitors results"""
    def __init__(self, translator: ROSTranslator):
        self.translator = translator
        self.active_tasks = {}
        
    def execute(self, command: Dict):
        """Execute a command through ROS"""
        ros_msg = self.translator.to_ros_message(command)
        # In POC, just publish to command topic
        self.translator.publishers['robot_commands'].publish(String(json.dumps(ros_msg)))

class AgenticROSNode:
    """Main AgenticROS node"""
    def __init__(self):
        rospy.init_node('agentic_ros_node')
        
        self.context = ContextManager()
        self.cognitive = CognitiveLayer()
        self.translator = ROSTranslator()
        self.executor = ActionExecutor(self.translator)
        
        # Set up basic ROS communication
        self.translator.register_publisher('robot_commands', String)
        rospy.Subscriber('robot_status', String, self.status_callback)
        
    def status_callback(self, msg):
        """Handle robot status updates"""
        try:
            status = json.loads(msg.data)
            self.context.update_state(status)
        except json.JSONDecodeError:
            rospy.logwarn(f"Invalid status message: {msg.data}")
    
    def execute_task(self, task_description: str):
        """Main entry point for task execution"""
        rospy.loginfo(f"Executing task: {task_description}")
        
        # Get current context
        context = self.context.get_context()
        
        # Analyze task
        plan = self.cognitive.analyze_task(task_description, context)
        rospy.loginfo(f"Generated plan: {plan}")
        
        # Execute plan
        for step in plan.get('steps', []):
            self.executor.execute(step)

def agentic_task(func):
    """Decorator for easy task definition"""
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        task_description = func.__doc__.format(*args, **kwargs)
        return self.execute_task(task_description)
    return wrapper

# Example robot implementation
class AgenticRobot:
    def __init__(self):
        self.node = AgenticROSNode()
    
    @agentic_task
    def pick_and_place(self, object_name: str, location: str):
        """
        Pick up the {0} and place it at {1}.
        Be careful with the object and avoid any obstacles.
        """
        pass
    
    @agentic_task
    def navigate_to(self, location: str):
        """
        Navigate to {0}.
        Use the most efficient path while ensuring safety.
        """
        pass

# Usage example
if __name__ == '__main__':
    try:
        robot = AgenticRobot()
        
        # Execute tasks using natural language
        robot.pick_and_place("red cube", "table A")
        robot.navigate_to("charging station")
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass