# AgenticROS

AgenticROS is a framework that uses AI agents (via CrewAI) to bridge natural language task definitions with ROS robotics operations, enabling more intuitive and intelligent robot development.

## Features

- 🤖 Natural language robot task definition
- 🧠 LLM-powered task decomposition and planning
- 🔄 Seamless ROS integration
- 📊 Context-aware decision making
- 🛠️ CrewAI-based cognitive architecture

## Installation

1. Install ROS (tested with ROS Noetic)
```bash
# Follow ROS installation instructions for your system
# http://wiki.ros.org/noetic/Installation
```

2. Install Python dependencies
```bash
pip install crewai openai
```

3. Clone AgenticROS
```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/agentic_ros.git
cd ..
catkin_make
```

4. Set up your OpenAI API key
```bash
export OPENAI_API_KEY='your-api-key-here'
```

## Quick Start

1. Source your workspace
```bash
source ~/catkin_ws/devel/setup.bash
```

2. Run the example
```bash
# Terminal 1
roscore

# Terminal 2
rosrun agentic_ros test_agentic_ros.py

# Terminal 3
rosrun agentic_ros agentic_ros.py
```

## Usage Example

```python
from agentic_ros import AgenticRobot

# Initialize robot
robot = AgenticRobot()

# Define tasks using natural language
@agentic_task
def pick_and_place(object_name: str, location: str):
    """
    Pick up the {0} and place it at {1}.
    Be careful with the object and avoid any obstacles.
    """
    pass

# Execute tasks
robot.pick_and_place("red cube", "table A")
```

## Architecture

```
AgenticROS
├── Cognitive Layer (LLM-based)
│   ├── TaskAnalyzer
│   ├── ContextManager
│   └── BehaviorGenerator
├── Bridge Layer
│   ├── ROSTranslator
│   ├── StateTracker
│   └── FeedbackProcessor
└── Execution Layer
    ├── ActionExecutor
    ├── SafetyMonitor
    └── PerformanceTracker
```

## Contributing

We welcome contributions! Please check out our contributing guidelines and code of conduct.

## License

MIT License - see LICENSE file for details

## Acknowledgments

- ROS Community
- CrewAI Framework
- OpenAI

## Citation

If you use AgenticROS in your research, please cite:

```bibtex
@misc{AgenticROS,
  author = {Kirill Balakhonov},
  title = {AgenticROS: Bridging LLMs and ROS for Intelligent Robotics},
  year = {2024},
  publisher = {GitHub},
  url = {https://github.com/NethermindEth/AgenticROS}
}
```

## Contact

For questions and support, please open an issue or contact [kirill.balakhonov@nethermind.io]
